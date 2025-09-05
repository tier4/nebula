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

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>

namespace nebula::ros
{

class WatchdogTimer
{
  using watchdog_cb_t = std::function<void(bool)>;

public:
  WatchdogTimer(
    rclcpp::Node & node, const std::chrono::microseconds & expected_update_interval,
    const watchdog_cb_t & callback)
  : node_(node),
    callback_(callback),
    expected_update_interval_ns_(
      std::chrono::duration_cast<std::chrono::nanoseconds>(expected_update_interval).count())
  {
    timer_ =
      node_.create_wall_timer(expected_update_interval, std::bind(&WatchdogTimer::on_timer, this));
  }

  void update() { last_update_ns_ = node_.get_clock()->now().nanoseconds(); }

private:
  void on_timer()
  {
    uint64_t now_ns = node_.get_clock()->now().nanoseconds();

    // As the clock is not steady, the update timestamp can be newer than clock.now().
    // Define that edge-case as not being late too.
    bool is_late = (last_update_ns_ > now_ns)
                     ? false
                     : (now_ns - last_update_ns_) > expected_update_interval_ns_;

    callback_(!is_late);
  }

  rclcpp::Node & node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<uint64_t> last_update_ns_;
  const watchdog_cb_t callback_;

  const uint64_t expected_update_interval_ns_;
};

}  // namespace nebula::ros
