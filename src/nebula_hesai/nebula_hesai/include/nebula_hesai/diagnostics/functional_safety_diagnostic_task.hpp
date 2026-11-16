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

#include "nebula_core_ros/diagnostics/render.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_core_common/diagnostics/liveness_monitor.hpp>
#include <nebula_core_common/diagnostics/severity_latch.hpp>
#include <nebula_core_common/util/clock.hpp>
#include <nebula_hesai_decoders/decoders/functional_safety.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <boost/algorithm/string.hpp>

#include <memory>
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
  if (error_codes.empty()) {
    return "No error codes";
  }

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

class FunctionalSafetyStatusProcessor
{
public:
  virtual ~FunctionalSafetyStatusProcessor() = default;
  virtual void populate_status(
    drivers::FunctionalSafetySeverity severity,
    const drivers::FunctionalSafetyErrorCodes & error_codes,
    diagnostic_msgs::msg::DiagnosticStatus & inout_status) = 0;
};

class FunctionalSafetyDiagnosticTask : public diagnostic_updater::CompositeDiagnosticTask
{
public:
  explicit FunctionalSafetyDiagnosticTask(
    const std::shared_ptr<const util::Clock> & clock,
    std::unique_ptr<FunctionalSafetyStatusProcessor> status_processor)
  : CompositeDiagnosticTask("Functional safety status"),
    liveness_monitor_(clock, 100ms),
    liveness_task_(
      "Liveness",
      [this](diagnostic_updater::DiagnosticStatusWrapper & status) {
        render(liveness_monitor_.report(), status);
      }),
    status_task_(
      "Status",
      [this](diagnostic_updater::DiagnosticStatusWrapper & status) {
        render_latched(status_latch_.report(), status, copy_status);
      }),
    stuck_task_(
      "Sensor functional safety system operation status",
      [this](diagnostic_updater::DiagnosticStatusWrapper & status) {
        render_latched(stuck_latch_.report(), status, copy_status);
      }),
    status_processor_(std::move(status_processor))
  {
    addTask(&liveness_task_);
    addTask(&status_task_);
    addTask(&stuck_task_);
  }

  void on_status(
    drivers::FunctionalSafetySeverity severity,
    const drivers::FunctionalSafetyErrorCodes & error_codes)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status_processor_->populate_status(severity, error_codes, status);
    status_latch_.submit(static_cast<diagnostics::Level>(status.level), status);
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

    stuck_latch_.submit(static_cast<diagnostics::Level>(status.level), status);
  }

private:
  using StatusLatch = diagnostics::SeverityLatch<diagnostic_msgs::msg::DiagnosticStatus>;

  static void copy_status(
    const StatusLatch::Report & report, diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    status.values.insert(
      status.values.end(), report.status.values.begin(), report.status.values.end());
    status.summary(report.status.level, report.status.message);
  }

  diagnostics::LivenessMonitor liveness_monitor_;
  StatusLatch status_latch_;
  StatusLatch stuck_latch_;
  diagnostic_updater::FunctionDiagnosticTask liveness_task_;
  diagnostic_updater::FunctionDiagnosticTask status_task_;
  diagnostic_updater::FunctionDiagnosticTask stuck_task_;
  std::unique_ptr<FunctionalSafetyStatusProcessor> status_processor_;
};

}  // namespace nebula::ros
