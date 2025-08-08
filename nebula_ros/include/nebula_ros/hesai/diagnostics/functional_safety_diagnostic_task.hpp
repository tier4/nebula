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

#include "nebula_ros/common/diagnostics/liveness_monitor.hpp"
#include "nebula_ros/common/diagnostics/severity_latch.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_decoders/nebula_decoders_hesai/decoders/functional_safety.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <boost/algorithm/string.hpp>

#include <string>
#include <utility>

namespace nebula::ros
{

namespace detail
{

inline uint8_t severity_to_diagnostic_status_level(drivers::FunctionalSafetySeverity severity)
{
  switch (severity) {
    case drivers::FunctionalSafetySeverity::OK:
      return diagnostic_msgs::msg::DiagnosticStatus::OK;
    case drivers::FunctionalSafetySeverity::WARNING:
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    default:
    case drivers::FunctionalSafetySeverity::ERROR:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
}

inline std::string error_codes_to_string(const drivers::FunctionalSafetyErrorCodes & error_codes)
{
  std::stringstream ss;
  for (auto it = error_codes.begin(); it != error_codes.end(); ++it) {
    if (it != error_codes.begin()) {
      ss << ", ";
    }
    ss << "0x" << std::hex << std::setw(4) << std::setfill('0') << *it;
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

}  // namespace detail

using std::chrono_literals::operator""ms;

class FunctionalSafetyDiagnosticTask : public diagnostic_updater::CompositeDiagnosticTask
{
public:
  explicit FunctionalSafetyDiagnosticTask(rclcpp::Node * const parent_node)
  : CompositeDiagnosticTask("Functional safety status"),
    liveness_monitor_("Liveness", parent_node, 100ms),
    status_latch_("Status"),
    stuck_latch_("Sensor functional safety system operation status")
  {
    addTask(&liveness_monitor_);
    addTask(&status_latch_);
    addTask(&stuck_latch_);
  }

  void on_status(
    drivers::FunctionalSafetySeverity severity,
    const drivers::FunctionalSafetyErrorCodes & error_codes)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = detail::severity_to_diagnostic_status_level(severity);
    status.message = detail::status_to_string(severity, error_codes.size());

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "Diagnostic codes";
    kv.value = detail::error_codes_to_string(error_codes);
    status.values.push_back(kv);

    status_latch_.submit(status);
  }

  void on_alive() { liveness_monitor_.tick(); }

  void on_stuck(bool is_stuck)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;

    if (is_stuck) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Sensor functional safety system is stuck";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Sensor functional safety system is running";
    }

    stuck_latch_.submit(status);
  }

private:
  LivenessMonitor liveness_monitor_;
  SeverityLatch status_latch_;
  SeverityLatch stuck_latch_;
};

}  // namespace nebula::ros
