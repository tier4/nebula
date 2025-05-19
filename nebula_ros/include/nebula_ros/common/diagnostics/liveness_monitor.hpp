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
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.hpp>

#include <chrono>
#include <cstdint>
#include <string>
#include <utility>

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
    rclcpp::Node::SharedPtr node, diagnostic_updater::Updater & diagnostic_updater,
    const std::string & name, const rclcpp::Duration & timeout)
  : timeout_(timeout), node_(std::move(node))
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
  void tick() { last_tick_ = node_->get_clock()->now(); }

private:
  void report_liveness(diagnostic_updater::DiagnosticStatusWrapper & status) const
  {
    using diagnostic_msgs::msg::DiagnosticStatus;

    rclcpp::Time now = node_->get_clock()->now();
    rclcpp::Duration lateness = now - last_tick_;
    bool is_live = lateness < timeout_;

    uint8_t severity = DiagnosticStatus::OK;
    std::string message = "Alive";
    std::string value = "true";

    if (!is_live) {
      severity = DiagnosticStatus::ERROR;
      message = "Dead";
      value = "false";
    }

    status.summary(severity, message);
    status.add("Is alive", value);
    status.add("Last tick", std::to_string(last_tick_.seconds()));
    status.add("Lateness", std::to_string(lateness.seconds()));
  }

  rclcpp::Time last_tick_;
  rclcpp::Duration timeout_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace nebula::ros
