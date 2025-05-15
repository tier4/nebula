// Copyright 2024 TIER IV, Inc.
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
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.hpp>

#include <chrono>
#include <string>

namespace nebula::ros
{

/**
 * @brief Monitor whether a routine is alive (is running at least once in a given period).
 *
 * This can be used for getting notified when a certain routine gets stuck, e.g. when packets stop
 * arriving or when pointclouds stop being published.
 */
class LivenessMonitor
{
public:
  /**
   * @brief Create and activate a new liveness monitor.
   *
   * The monitored routine has to call `tick()` on every iteration to prove its liveness. If there
   * is no call for a length of `timeout`, the routine is declared dead.
   *
   * @param node The node used for creating the internal timer
   # @param diagnostic_updater The diagnostic updater to register a task with
   * @param timeout The time after the last call to `tick()` where liveliness is
   */
  LivenessMonitor(
    rclcpp::Node & node, diagnostic_updater::Updater & diagnostic_updater, const std::string & name,
    const std::chrono::nanoseconds & timeout)
  : timer_(node.create_wall_timer(timeout, [this] { on_timeout(); }))
  {
    diagnostic_updater.add(name, [this](diagnostic_updater::DiagnosticStatusWrapper & status) {
      report_liveness(status);
    });
  }

  /**
   * @brief Proves the liveness of a routine. Has to be called frequently.
   *
   * A call to `tick()` resets the internal timer. If the timer is not reset at least once before it
   * expires (within `timeout`), the monitored routine is pronounced dead.
   */
  void tick() { timer_->reset(); }

private:
  void on_timeout()
  {
    is_live_ = false;
    diagnostic_updater_->force_update();
  }

  void report_liveness(diagnostic_updater::DiagnosticStatusWrapper & status) const
  {
    using diagnostic_msgs::msg::DiagnosticStatus;

    status.summary(is_live_ ? DiagnosticStatus::OK : DiagnosticStatus::ERROR, "");
    status.add("alive", is_live_ ? "true" : "false");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater * diagnostic_updater_;
  bool is_live_{false};
};

}  // namespace nebula::ros
