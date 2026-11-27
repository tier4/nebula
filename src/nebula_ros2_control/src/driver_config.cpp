// Copyright 2026 TIER IV, Inc.
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

#include "nebula_ros2_control/driver_config.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace nebula::ros2_control
{

namespace
{

std::string trim(const std::string & value)
{
  const auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
  auto begin = std::find_if_not(value.begin(), value.end(), is_space);
  auto end = std::find_if_not(value.rbegin(), value.rend(), is_space).base();
  if (begin >= end) {
    return {};
  }
  return {begin, end};
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

bool parse_bool(const std::string & name, const std::string & value)
{
  std::string lower = trim(value);
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (lower == "true" || lower == "1" || lower == "yes" || lower == "on") {
    return true;
  }
  if (lower == "false" || lower == "0" || lower == "no" || lower == "off") {
    return false;
  }
  throw ConfigError("Parameter '" + name + "' must be a boolean, got '" + value + "'");
}

std::size_t parse_positive_size(const std::string & name, const std::string & value)
{
  const std::string trimmed = trim(value);
  if (trimmed.empty() || !std::all_of(trimmed.begin(), trimmed.end(), [](unsigned char c) {
        return std::isdigit(c) != 0;
      })) {
    throw ConfigError("Parameter '" + name + "' must be a positive integer, got '" + value + "'");
  }
  std::size_t result = 0;
  try {
    result = std::stoul(trimmed);
  } catch (const std::exception &) {
    throw ConfigError("Parameter '" + name + "' is out of range: '" + value + "'");
  }
  if (result == 0) {
    throw ConfigError("Parameter '" + name + "' must be at least 1");
  }
  return result;
}

std::pair<std::string, std::string> parse_remapping(const std::string & rule)
{
  const auto separator = rule.find(":=");
  if (separator == std::string::npos) {
    throw ConfigError("Remapping '" + rule + "' must have the form 'from:=to'");
  }
  std::string from = trim(rule.substr(0, separator));
  std::string to = trim(rule.substr(separator + 2));
  if (from.empty() || to.empty()) {
    throw ConfigError("Remapping '" + rule + "' must have the form 'from:=to'");
  }
  return {from, to};
}

/// Collect the `ros.*` parameters (plus `frame_id` for sensors) in a deterministic order.
void collect_ros_parameters(
  const std::unordered_map<std::string, std::string> & parameters,
  std::map<std::string, std::string> & out)
{
  const std::string prefix = k_ros_parameter_prefix;
  for (const auto & [name, value] : parameters) {
    if (!starts_with(name, prefix)) {
      continue;
    }
    std::string ros_name = name.substr(prefix.size());
    if (ros_name.empty()) {
      throw ConfigError("Parameter '" + name + "' has an empty ROS parameter name");
    }
    out[ros_name] = value;
  }
}

std::uint32_t read_u32(const std::uint8_t * data, bool little_endian)
{
  if (little_endian) {
    return static_cast<std::uint32_t>(data[0]) | (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) |
           (static_cast<std::uint32_t>(data[3]) << 24U);
  }
  return static_cast<std::uint32_t>(data[3]) | (static_cast<std::uint32_t>(data[2]) << 8U) |
         (static_cast<std::uint32_t>(data[1]) << 16U) |
         (static_cast<std::uint32_t>(data[0]) << 24U);
}

}  // namespace

StateInterfaceKind parse_state_interface_kind(const std::string & name)
{
  static const std::unordered_map<std::string, StateInterfaceKind> kinds{
    {"driver_active", StateInterfaceKind::driver_active},
    {"message_count", StateInterfaceKind::message_count},
    {"message_rate", StateInterfaceKind::message_rate},
    {"message_age", StateInterfaceKind::message_age},
    {"header_stamp", StateInterfaceKind::header_stamp},
  };
  auto it = kinds.find(name);
  if (it == kinds.end()) {
    throw ConfigError(
      "Unknown state interface '" + name +
      "'. Supported: driver_active, message_count, message_rate, message_age, header_stamp");
  }
  return it->second;
}

bool requires_monitor_topic(StateInterfaceKind kind)
{
  return kind != StateInterfaceKind::driver_active;
}

DriverConfig parse_driver_config(
  const std::string & hardware_name,
  const std::unordered_map<std::string, std::string> & hardware_parameters,
  const std::unordered_map<std::string, std::string> & sensor_parameters)
{
  DriverConfig config;
  // Not the bare hardware name: on Jazzy and later, ros2_control already creates a node with that
  // name for every hardware component.
  config.node_name = hardware_name + "_driver";

  const std::string prefix = k_ros_parameter_prefix;

  for (const auto & [name, raw_value] : hardware_parameters) {
    if (starts_with(name, prefix)) {
      continue;
    }
    const std::string value = trim(raw_value);
    if (name == "plugin") {
      config.plugin = value;
    } else if (name == "package") {
      config.package = value;
    } else if (name == "library") {
      config.library = value;
    } else if (name == "node_name") {
      config.node_name = value;
    } else if (name == "namespace") {
      config.node_namespace = value;
    } else if (name == "params_file" || name == "params_files") {
      for (auto & file : split_list(value)) {
        config.params_files.push_back(std::move(file));
      }
    } else if (name == "allow_substs") {
      config.allow_substs = parse_bool(name, value);
    } else if (name == "remappings") {
      for (const auto & rule : split_list(value)) {
        config.remappings.push_back(parse_remapping(rule));
      }
    } else if (name == "use_intra_process_comms") {
      config.use_intra_process_comms = parse_bool(name, value);
    } else if (name == "executor_threads") {
      config.executor_threads = parse_positive_size(name, value);
    } else if (name == "monitor_topic") {
      config.monitor_topic = value;
    } else if (name == "monitor_topic_type") {
      config.monitor_topic_type = value;
    } else {
      throw ConfigError(
        "Unknown hardware parameter '" + name + "'. To pass a ROS parameter to the driver, use '" +
        prefix + name + "'");
    }
  }

  if (config.plugin.empty()) {
    throw ConfigError(
      "Hardware parameter 'plugin' is required (the driver component class, e.g. "
      "'HesaiRosWrapper')");
  }
  if (config.node_name.empty()) {
    throw ConfigError("Hardware parameter 'node_name' must not be empty");
  }
  if (!config.node_namespace.empty() && config.node_namespace.front() != '/') {
    config.node_namespace.insert(config.node_namespace.begin(), '/');
  }
  if (!config.monitor_topic.empty() && config.monitor_topic_type.empty()) {
    throw ConfigError("Hardware parameter 'monitor_topic_type' must not be empty");
  }

  // Hardware-level ROS parameters first, then sensor-level ones, which take precedence.
  std::map<std::string, std::string> ros_parameters;
  collect_ros_parameters(hardware_parameters, ros_parameters);

  for (const auto & [name, value] : sensor_parameters) {
    if (name == "frame_id") {
      ros_parameters["frame_id"] = trim(value);
    } else if (!starts_with(name, prefix)) {
      throw ConfigError(
        "Unknown sensor parameter '" + name + "'. Supported: 'frame_id' and '" + prefix +
        "<ros parameter>'");
    }
  }
  collect_ros_parameters(sensor_parameters, ros_parameters);

  config.parameters.assign(ros_parameters.begin(), ros_parameters.end());
  return config;
}

std::vector<std::string> build_ros_arguments(const DriverConfig & config)
{
  std::vector<std::string> arguments{"--ros-args", "-r", "__node:=" + config.node_name};
  if (!config.node_namespace.empty()) {
    arguments.insert(arguments.end(), {"-r", "__ns:=" + config.node_namespace});
  }
  for (const auto & file : config.params_files) {
    arguments.insert(arguments.end(), {"--params-file", file});
  }
  for (const auto & [name, value] : config.parameters) {
    // rcl parses the value as YAML; an empty value would be YAML null, which is not a valid
    // parameter value, so pass it as an empty string instead.
    const std::string yaml_value = trim(value).empty() ? std::string{"''"} : value;
    arguments.insert(arguments.end(), {"-p", name + ":=" + yaml_value});
  }
  for (const auto & [from, to] : config.remappings) {
    arguments.insert(arguments.end(), {"-r", from + ":=" + to});
  }
  return arguments;
}

std::string component_factory_class_name(const std::string & plugin)
{
  return "rclcpp_components::NodeFactoryTemplate<" + plugin + ">";
}

std::optional<double> parse_header_stamp(const std::uint8_t * data, std::size_t size)
{
  // Encapsulation header: 2-byte representation identifier + 2-byte options.
  constexpr std::size_t k_encapsulation_size = 4;
  if (data == nullptr || size < k_encapsulation_size || data[0] != 0x00) {
    return std::nullopt;
  }

  std::size_t offset = k_encapsulation_size;
  bool little_endian = false;
  switch (data[1]) {
    case 0x00:  // CDR_BE
    case 0x06:  // PLAIN_CDR2_BE (final types, the ROS default)
      break;
    case 0x01:  // CDR_LE
    case 0x07:  // PLAIN_CDR2_LE
      little_endian = true;
      break;
    case 0x08:  // DELIMITED_CDR2_BE (appendable types): a 4-byte DHEADER precedes the struct
      offset += 4;
      break;
    case 0x09:  // DELIMITED_CDR2_LE
      little_endian = true;
      offset += 4;
      break;
    default:  // Parameter-list encodings are not supported
      return std::nullopt;
  }

  // builtin_interfaces/Time: int32 sec, uint32 nanosec
  if (size < offset + 8) {
    return std::nullopt;
  }
  const auto sec = static_cast<std::int32_t>(read_u32(data + offset, little_endian));
  const std::uint32_t nanosec = read_u32(data + offset + 4, little_endian);
  return static_cast<double>(sec) + static_cast<double>(nanosec) * 1e-9;
}

std::string apply_substitutions(const std::string & text, const SubstitutionResolver & resolver)
{
  std::string result;
  result.reserve(text.size());
  std::size_t position = 0;
  while (true) {
    const auto start = text.find("$(", position);
    if (start == std::string::npos) {
      result.append(text, position, std::string::npos);
      return result;
    }
    const auto end = text.find(')', start + 2);
    if (end == std::string::npos) {
      throw ConfigError("Unterminated substitution in '" + text + "'");
    }
    const std::string body = trim(text.substr(start + 2, end - start - 2));
    const auto separator = body.find_first_of(" \t");
    const std::string kind = body.substr(0, separator);
    const std::string argument =
      separator == std::string::npos ? std::string{} : trim(body.substr(separator + 1));
    if (kind.empty()) {
      throw ConfigError("Empty substitution in '" + text + "'");
    }

    result.append(text, position, start - position);
    result += resolver(kind, argument);
    position = end + 1;
  }
}

std::string resolve_var_substitution(const DriverConfig & config, const std::string & name)
{
  for (const auto & [parameter_name, value] : config.parameters) {
    if (parameter_name != name) {
      continue;
    }
    std::string unquoted = trim(value);
    if (
      unquoted.size() >= 2 && (unquoted.front() == '"' || unquoted.front() == '\'') &&
      unquoted.back() == unquoted.front()) {
      unquoted = unquoted.substr(1, unquoted.size() - 2);
    }
    return unquoted;
  }
  throw ConfigError(
    "Cannot resolve $(var " + name + "): set it with <param name=\"" +
    std::string{k_ros_parameter_prefix} + name + "\">...</param>");
}

std::vector<std::string> split_list(const std::string & value, char delimiter)
{
  std::vector<std::string> result;
  std::size_t start = 0;
  while (start <= value.size()) {
    auto end = value.find(delimiter, start);
    if (end == std::string::npos) {
      end = value.size();
    }
    std::string item = trim(value.substr(start, end - start));
    if (!item.empty()) {
      result.push_back(std::move(item));
    }
    start = end + 1;
  }
  return result;
}

}  // namespace nebula::ros2_control
