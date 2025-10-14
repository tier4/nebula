// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/functional_safety.hpp"
#include "nebula_ros/hesai/diagnostics/functional_safety_diagnostic_task.hpp"

#include <diagnostic_msgs/msg/detail/key_value__struct.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/range/algorithm/find.hpp>
#include <boost/range/algorithm/find_if.hpp>
#include <boost/tokenizer.hpp>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace nebula::ros
{

using std::string_literals::operator""s;

struct ErrorDefinition
{
  uint16_t code;
  std::string description;
  drivers::FunctionalSafetySeverity severity;

  bool operator==(const ErrorDefinition & other) const
  {
    return code == other.code && description == other.description && severity == other.severity;
  }

  friend std::ostream & operator<<(std::ostream & os, const ErrorDefinition & error_definition)
  {
    os << "0x" << std::hex << std::setw(4) << std::setfill('0') << error_definition.code << " ("
       << error_definition.severity << ")";
    return os;
  }
};

/// Convert severity string to enum value
inline drivers::FunctionalSafetySeverity string_to_severity(const std::string & severity_str)
{
  std::string lower_str = boost::algorithm::to_lower_copy(severity_str);

  if (lower_str == "warning") {
    return drivers::FunctionalSafetySeverity::OK;
  }

  if (
    lower_str.find("untrusted") != std::string::npos ||
    lower_str.find("shutdown") != std::string::npos ||
    lower_str.find("pre-shutdown") != std::string::npos) {
    return drivers::FunctionalSafetySeverity::ERROR;
  }

  if (lower_str.find("performance degradation") != std::string::npos) {
    return drivers::FunctionalSafetySeverity::WARNING;
  }

  throw std::runtime_error("Unknown severity level: " + severity_str);
}

/// Read error definitions from CSV file
inline std::vector<ErrorDefinition> read_error_definitions_from_csv(const std::string & file_path)
{
  constexpr int hex_base = 16;
  std::vector<ErrorDefinition> error_definitions;
  std::ifstream file(file_path);

  if (!file.is_open()) {
    throw std::runtime_error("Failed to open CSV file: " + file_path);
  }

  std::string line;

  auto make_exception = [&line](const std::string & reason) {
    return std::runtime_error("Failed to parse line (" + reason + "): " + line);
  };

  bool got_header = false;
  // Track first seen definition per code to detect duplicates and conflicts while
  // preserving file order in error_definitions
  std::unordered_map<uint16_t, ErrorDefinition> first_definition_by_code;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    // Use boost::tokenizer to split the CSV line on semicolons
    boost::escaped_list_separator<char> separator('\\', ';', '"');
    boost::tokenizer<boost::escaped_list_separator<char>> tokenizer(line, separator);

    std::vector<std::string> fields;
    for (const auto & token : tokenizer) {
      fields.push_back(token);
    }

    // Validate that we have exactly 3 fields
    if (fields.size() != 3) {
      throw make_exception("expected exactly 3 fields (code;description;severity)");
    }

    if (
      !got_header &&
      (fields[0] == "Fault Code" && fields[1] == "Fault" && fields[2] == "Lidar State")) {
      got_header = true;
      continue;
    }

    // Parse fault code
    const std::string & code_field = fields[0];
    uint16_t code = 0;
    if (code_field.size() < 2 || code_field.substr(0, 2) != "0x") {
      throw make_exception("expected fault code of shape 0xABCD");
    }

    {
      const uint64_t parsed = std::stoul(code_field.substr(2), nullptr, hex_base);
      if (parsed > std::numeric_limits<uint16_t>::max()) {
        throw make_exception("fault code does not fit in uint16");
      }
      code = static_cast<uint16_t>(parsed);
    }

    // Parse description and severity
    const std::string & description = fields[1];
    const std::string & severity_str = fields[2];
    drivers::FunctionalSafetySeverity severity = string_to_severity(severity_str);

    const ErrorDefinition current{code, description, severity};

    const auto it = first_definition_by_code.find(code);
    if (it == first_definition_by_code.end()) {
      first_definition_by_code.emplace(code, current);
      error_definitions.emplace_back(current);
    } else {
      // Duplicate code found. If identical definition, ignore; otherwise it's a conflict
      if (!(it->second == current)) {
        throw make_exception("conflicting duplicate definition for fault code");
      }
      // Identical duplicate -> ignore (do not append again)
    }
  }

  return error_definitions;
}

class FunctionalSafetyAdvanced : public FunctionalSafetyStatusProcessor
{
public:
  FunctionalSafetyAdvanced(
    const std::vector<ErrorDefinition> & error_definitions,
    const std::vector<uint16_t> & exempted_codes)
  : error_definitions_(error_definitions), exempted_codes_(exempted_codes)
  {
  }

  void populate_status(
    drivers::FunctionalSafetySeverity severity,
    const drivers::FunctionalSafetyErrorCodes & error_codes,
    diagnostic_msgs::msg::DiagnosticStatus & inout_status) override
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "Diagnostic codes";
    kv.value = detail::error_codes_to_string(error_codes);
    inout_status.values.push_back(kv);

    if (error_codes.empty()) {
      // If there are no error codes, report the severity reported by the sensor
      inout_status.level = detail::severity_to_diagnostic_status_level(severity);
      inout_status.message = detail::status_to_string(severity, 0);
      inout_status.values.push_back(kv);
      return;
    }

    auto [non_exempted_error_codes, exempted_error_codes] =
      split_error_codes(error_codes, exempted_codes_);

    drivers::FunctionalSafetySeverity max_severity = drivers::FunctionalSafetySeverity::OK;
    for (const auto & error_code : non_exempted_error_codes) {
      // If the error code is not found in the definitions, use the severity reported by the
      // sensor as the worst-case assumption.
      const auto error_definition =
        get_error_definition(error_code)
          .value_or(ErrorDefinition{error_code, "Unknown error", severity});

      max_severity = std::max(max_severity, error_definition.severity);
      auto error_kv = to_key_value(error_definition);
      inout_status.values.push_back(error_kv);
    }

    for (const auto & error_code : exempted_error_codes) {
      const auto error_definition =
        get_error_definition(error_code)
          .value_or(ErrorDefinition{error_code, "Unknown error", severity});
      auto error_kv = to_key_value(error_definition);
      error_kv.value = "[Exempted] " + error_kv.value;
      inout_status.values.push_back(error_kv);
    }

    inout_status.level = detail::severity_to_diagnostic_status_level(max_severity);

    // When the effective severity is OK, report nominal operation regardless of code count
    const size_t n_errors_for_message =
      (max_severity == drivers::FunctionalSafetySeverity::OK) ? 0 : non_exempted_error_codes.size();
    inout_status.message = detail::status_to_string(max_severity, n_errors_for_message);
  }

private:
  static std::tuple<drivers::FunctionalSafetyErrorCodes, drivers::FunctionalSafetyErrorCodes>
  split_error_codes(
    const drivers::FunctionalSafetyErrorCodes & error_codes,
    const std::vector<uint16_t> & exempted_codes)
  {
    drivers::FunctionalSafetyErrorCodes non_exempted_error_codes;
    drivers::FunctionalSafetyErrorCodes exempted_error_codes;
    for (const auto & error_code : error_codes) {
      if (boost::range::find(exempted_codes, error_code) == exempted_codes.end()) {
        non_exempted_error_codes.push_back(error_code);
      } else {
        exempted_error_codes.push_back(error_code);
      }
    }
    return std::make_tuple(non_exempted_error_codes, exempted_error_codes);
  }

  [[nodiscard]] std::optional<ErrorDefinition> get_error_definition(uint16_t error_code) const
  {
    auto it = boost::range::find_if(
      error_definitions_, [error_code](const ErrorDefinition & error_definition) {
        return error_definition.code == error_code;
      });

    if (it != error_definitions_.end()) {
      return *it;
    }

    return std::nullopt;
  }

  static diagnostic_msgs::msg::KeyValue to_key_value(const ErrorDefinition & error_definition)
  {
    diagnostic_msgs::msg::KeyValue kv;
    std::stringstream ss;
    ss << "0x" << std::hex << std::setw(4) << std::setfill('0') << error_definition.code;
    kv.key = ss.str();

    char prefix{};
    switch (error_definition.severity) {
      case drivers::FunctionalSafetySeverity::OK:
        prefix = 'O';
        break;
      case drivers::FunctionalSafetySeverity::WARNING:
        prefix = 'W';
        break;
      case drivers::FunctionalSafetySeverity::ERROR:
        prefix = 'E';
        break;
      default:
        throw std::runtime_error(
          "Unknown severity level: " + std::to_string(static_cast<int>(error_definition.severity)));
    }
    kv.value = "["s + prefix + "] " + error_definition.description;
    return kv;
  }

  std::vector<ErrorDefinition> error_definitions_;
  std::vector<uint16_t> exempted_codes_;
};

}  // namespace nebula::ros
