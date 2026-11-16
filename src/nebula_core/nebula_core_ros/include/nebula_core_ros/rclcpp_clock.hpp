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

#include <nebula_core_common/util/clock.hpp>
#include <rclcpp/clock.hpp>

#include <chrono>
#include <memory>
#include <utility>

namespace nebula::ros
{

/// @brief Adapts an `rclcpp::Clock` to the ROS-independent clock interface.
class RclcppClock final : public util::Clock
{
public:
  explicit RclcppClock(rclcpp::Clock::SharedPtr underlying) : underlying_(std::move(underlying)) {}

  [[nodiscard]] std::chrono::nanoseconds now() const override
  {
    return std::chrono::nanoseconds(underlying_->now().nanoseconds());
  }

private:
  rclcpp::Clock::SharedPtr underlying_;
};

/**
 * @brief Make the clock that diagnostics of `node` measure rates and timeouts on.
 *
 * This is ROS time if the node has `use_sim_time` set, and steady time otherwise.
 */
template <typename NodeT>
std::shared_ptr<const util::Clock> make_diagnostics_clock(const NodeT & node)
{
  bool use_sim_time = false;
  if (node.has_parameter("use_sim_time")) {
    use_sim_time = node.get_parameter("use_sim_time").as_bool();
  }

  return std::make_shared<RclcppClock>(
    std::make_shared<rclcpp::Clock>(use_sim_time ? RCL_ROS_TIME : RCL_STEADY_TIME));
}

}  // namespace nebula::ros
