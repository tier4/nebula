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
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace nebula::ros
{

/**
 * @brief Monitor whether a routine is alive (is running at least once in a given period).
 *
 * This can be used for getting notified when a certain routine gets stuck, e.g. when packets stop
 * arriving or when pointclouds stop being published.
 */
class LivenessMonitor : public diagnostic_updater::DiagnosticTask
{
public:
  /**
   * @brief Create and activate a new liveness monitor.
   *
   * The monitored routine has to call `tick()` on every iteration to prove its liveness. If there
   * is no call for a length of `timeout`, the routine is declared dead.
   *
   * @param name The name of the task
   * @param parent_node The node from which clock type and parameters are read.
   * @param timeout The time after the last call to `tick()` where the routine is declared dead
   */
  LivenessMonitor(
    const std::string & name, const rclcpp::Node * parent_node, const rclcpp::Duration & timeout)
  : DiagnosticTask(name), timeout_(timeout)
  {
    // select clock according to the use_sim_time paramter set to the parent
    bool use_sim_time = false;
    if (parent_node->has_parameter("use_sim_time")) {
      use_sim_time = parent_node->get_parameter("use_sim_time").as_bool();
    }
    if (use_sim_time) {
      clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    } else {
      clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    }

    last_tick_ = clock_->now();
  }

  /**
   * @brief Proves the liveness of a routine. Has to be called frequently.
   *
   * A call to `tick()` resets the internal timer. If the timer is not reset at least once before it
   * expires (within `timeout`), the monitored routine is pronounced dead.
   */
  void tick() { last_tick_ = clock_->now(); }

private:
  void run(diagnostic_updater::DiagnosticStatusWrapper & status) override
  {
    using diagnostic_msgs::msg::DiagnosticStatus;

    rclcpp::Time now = clock_->now();
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
    status.add("Last tick [s]", std::to_string(last_tick_.seconds()));
    status.add("Lateness [ms]", std::to_string(lateness.seconds() * 1000));
  }

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration timeout_;
  rclcpp::Time last_tick_;
};

}  // namespace nebula::ros
