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

#pragma once

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <nebula_core_ros/diagnostics/render.hpp>
#include <nebula_hesai_decoders/decoders/functional_safety.hpp>
#include <nebula_hesai_decoders/diagnostics/functional_safety_monitor.hpp>
#include <nebula_hesai_decoders/diagnostics/packet_loss_monitor.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

/// @file
/// Functions that write the reports of the Hesai decoder's monitors into the statuses of a
/// `diagnostic_updater`.

namespace nebula::ros
{

namespace detail
{

inline std::string error_code_to_string(uint16_t error_code)
{
  std::stringstream ss;
  ss << "0x" << std::hex << std::setw(4) << std::setfill('0') << error_code;
  return ss.str();
}

inline std::string error_codes_to_string(const drivers::FunctionalSafetyErrorCodes & error_codes)
{
  if (error_codes.empty()) {
    return "No error codes";
  }

  std::stringstream ss;
  for (auto it = error_codes.begin(); it != error_codes.end(); ++it) {
    if (it != error_codes.begin()) {
      ss << ", ";
    }
    ss << error_code_to_string(*it);
  }

  return ss.str();
}

inline std::string status_to_string(drivers::FunctionalSafetySeverity severity, size_t n_errors)
{
  if (n_errors == 0) {
    switch (severity) {
      case drivers::FunctionalSafetySeverity::OK:
        return "Operating nominally";
      case drivers::FunctionalSafetySeverity::WARNING:
        return "Unknown warning";
      case drivers::FunctionalSafetySeverity::ERROR:
        return "Unknown error";
    }
  }

  std::string n_errors_str = std::to_string(n_errors);

  switch (severity) {
    case drivers::FunctionalSafetySeverity::OK:
      return "Sensor reports OK, but sent " + n_errors_str + " error codes.";
    case drivers::FunctionalSafetySeverity::WARNING:
      return "Sensor reports " + n_errors_str + " warnings";
    default:
    case drivers::FunctionalSafetySeverity::ERROR:
      return "Sensor reports " + n_errors_str + " errors";
  }
}

inline std::string fault_to_string(const drivers::FunctionalSafetyFault & fault)
{
  char prefix{};
  switch (fault.severity) {
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
        "Unknown severity level: " + std::to_string(static_cast<int>(fault.severity)));
  }

  return std::string("[") + prefix + "] " + fault.description;
}

}  // namespace detail

inline void render(
  const drivers::FunctionalSafetyStatus & report,
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  status.add("Diagnostic codes", detail::error_codes_to_string(report.error_codes));

  if (!report.faults) {
    status.summary(
      to_diagnostic_status_level(report.level()),
      detail::status_to_string(report.severity, report.error_codes.size()));
    return;
  }

  size_t n_non_exempted = 0;
  for (const auto & fault : *report.faults) {
    if (!fault.is_exempted) {
      status.add(detail::error_code_to_string(fault.code), detail::fault_to_string(fault));
      ++n_non_exempted;
    }
  }

  for (const auto & fault : *report.faults) {
    if (fault.is_exempted) {
      status.add(
        detail::error_code_to_string(fault.code), "[Exempted] " + detail::fault_to_string(fault));
    }
  }

  // When the effective severity is OK, report nominal operation regardless of code count
  const size_t n_errors_for_message =
    (report.severity == drivers::FunctionalSafetySeverity::OK) ? 0 : n_non_exempted;
  status.summary(
    to_diagnostic_status_level(report.level()),
    detail::status_to_string(report.severity, n_errors_for_message));
}

/// @brief Render liveness, status and operation status like a
/// `diagnostic_updater::CompositeDiagnosticTask` of the three would.
inline void render(
  const drivers::FunctionalSafetyMonitor::Report & report,
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  diagnostic_updater::DiagnosticStatusWrapper combined;
  auto merge = [&](const diagnostic_updater::DiagnosticStatusWrapper & part) {
    status.values.insert(status.values.end(), part.values.begin(), part.values.end());
    combined.mergeSummary(part);
  };

  diagnostic_updater::DiagnosticStatusWrapper liveness;
  render(report.liveness, liveness);
  merge(liveness);

  diagnostic_updater::DiagnosticStatusWrapper functional_safety_status;
  render_latched(
    report.status, functional_safety_status,
    [](const drivers::FunctionalSafetyStatus & latched, auto & out) { render(latched, out); });
  merge(functional_safety_status);

  diagnostic_updater::DiagnosticStatusWrapper operation_status;
  render_latched(report.is_stuck, operation_status, [](bool is_stuck, auto & out) {
    if (is_stuck) {
      out.summary(DiagnosticStatus::ERROR, "Sensor functional safety system is stuck");
    } else {
      out.summary(DiagnosticStatus::OK, "Sensor functional safety system is running");
    }
  });
  merge(operation_status);

  status.summary(combined);
}

inline void render(
  const drivers::PacketLossMonitor::Report & report,
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using seconds = std::chrono::duration<double>;

  status.add("Lost packets", report.lost_packets);
  status.add("Error threshold", report.error_threshold);
  status.add(
    "Time since last update [s]", std::to_string(seconds(report.time_since_last_report).count()));

  switch (report.level()) {
    case diagnostics::Level::OK:
      status.summary(to_diagnostic_status_level(report.level()), "No packet loss");
      break;
    case diagnostics::Level::WARN:
      status.summary(to_diagnostic_status_level(report.level()), "Slight packet loss");
      break;
    default:
      status.summary(to_diagnostic_status_level(report.level()), "Severe packet loss");
      break;
  }
}

}  // namespace nebula::ros
