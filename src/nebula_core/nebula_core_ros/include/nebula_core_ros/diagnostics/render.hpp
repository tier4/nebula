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
#include <nebula_core_common/diagnostics/level.hpp>
#include <nebula_core_common/diagnostics/liveness_monitor.hpp>
#include <nebula_core_common/diagnostics/rate_bound_status.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

/// @file
/// Functions that write the typed reports of Nebula's ROS-independent monitors into the statuses
/// of a `diagnostic_updater`. A monitor is registered with an updater like this:
///
/// @code
/// updater.add("Liveness", [this](diagnostic_updater::DiagnosticStatusWrapper & status) {
///   render(liveness_monitor_.report(), status);
/// });
/// @endcode

namespace nebula::ros
{

inline uint8_t to_diagnostic_status_level(diagnostics::Level level)
{
  return static_cast<uint8_t>(level);
}

inline void render(
  const diagnostics::LivenessMonitor::Report & report,
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using seconds = std::chrono::duration<double>;
  using milliseconds = std::chrono::duration<double, std::milli>;

  status.summary(to_diagnostic_status_level(report.level()), report.is_alive ? "Alive" : "Dead");
  status.add("Is alive", std::string(report.is_alive ? "true" : "false"));
  status.add("Last tick [s]", std::to_string(seconds(report.last_tick).count()));
  status.add("Lateness [ms]", std::to_string(milliseconds(report.lateness).count()));
}

inline void render(
  const diagnostics::RateBoundStatus::Report & report,
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  auto message = [](diagnostics::Level level) -> std::string {
    switch (level) {
      case diagnostics::Level::OK:
        return "Rate is reasonable";
      case diagnostics::Level::WARN:
        return "Rate is within warning range";
      case diagnostics::Level::ERROR:
        return "Rate is out of valid range";
      case diagnostics::Level::STALE:
        return "Topic has not been received yet";
      default:
        return "Undefined state";
    }
  };

  auto fixed = [](double value) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << value;
    return ss.str();
  };

  auto boolean = [](bool value) { return std::string(value ? "true" : "false"); };

  status.summary(to_diagnostic_status_level(report.level), message(report.level));

  status.add("Publish rate", fixed(report.rate_hz.value_or(0.0)));
  status.add("Effective rate status", diagnostics::to_string(report.level));
  status.add("Candidate rate status", diagnostics::to_string(report.candidate_level));
  status.add("Candidate status observed frames", report.candidate_num_observations);
  status.add("Assumed skipped frames", report.assumed_skipped_frames);
  status.add("Observed frames transition threshold", report.num_frame_transition);
  status.add("Minimum OK rate threshold", fixed(report.ok_params.min_frequency));
  if (report.ok_params.max_frequency) {
    status.add("Maximum OK rate threshold", fixed(report.ok_params.max_frequency.value()));
  }
  status.add("Minimum WARN rate threshold", fixed(report.warn_params.min_frequency));
  if (report.warn_params.max_frequency) {
    status.add("Maximum WARN rate threshold", fixed(report.warn_params.max_frequency.value()));
  }
  status.add("Immediate error report", boolean(report.immediate_error_report));
  status.add("Immediate relax state", boolean(report.immediate_relax_state));
}

/// @brief Render what a `diagnostics::SeverityLatch` reported: "No status available" as STALE while
/// nothing has been submitted yet, otherwise whatever `render_status` writes for the latched value.
/// @param latched The latch's report, or any optional derived from it
/// @param render_status Called as `render_status(*latched, status)`
template <typename T, typename RenderStatus>
void render_latched(
  const std::optional<T> & latched, diagnostic_updater::DiagnosticStatusWrapper & status,
  RenderStatus && render_status)
{
  if (!latched) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No status available");
    return;
  }

  render_status(*latched, status);
}

}  // namespace nebula::ros
