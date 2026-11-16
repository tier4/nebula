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

#include "nebula_core_ros/rclcpp_clock.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/parameter.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace nebula::ros
{

namespace
{

/// Stands in for a node, of which `make_diagnostics_clock` only reads `use_sim_time`.
struct FakeNode
{
  std::optional<bool> use_sim_time;

  [[nodiscard]] bool has_parameter(const std::string & name) const
  {
    return name == "use_sim_time" && use_sim_time.has_value();
  }

  [[nodiscard]] rclcpp::Parameter get_parameter(const std::string & name) const
  {
    return {name, use_sim_time.value()};
  }
};

std::chrono::nanoseconds now(rcl_clock_type_t type)
{
  return std::chrono::nanoseconds(rclcpp::Clock(type).now().nanoseconds());
}

/// Steady and system time are far apart on any machine that has been running for less than years
/// (steady time starts at boot), so the clock type can be told from the magnitude of `now()`.
bool is_close(std::chrono::nanoseconds a, std::chrono::nanoseconds b)
{
  return std::chrono::abs(a - b) < std::chrono::minutes(1);
}

}  // namespace

TEST(RclcppClock, ReportsTheTimeOfTheUnderlyingClock)
{
  auto underlying = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const RclcppClock clock(underlying);

  const auto before = std::chrono::nanoseconds(underlying->now().nanoseconds());
  const auto reported = clock.now();
  const auto after = std::chrono::nanoseconds(underlying->now().nanoseconds());

  EXPECT_LE(before, reported);
  EXPECT_LE(reported, after);
}

TEST(MakeDiagnosticsClock, UsesSteadyTimeWithoutSimTime)
{
  ASSERT_FALSE(is_close(now(RCL_STEADY_TIME), now(RCL_ROS_TIME)));

  EXPECT_TRUE(is_close(make_diagnostics_clock(FakeNode{})->now(), now(RCL_STEADY_TIME)));
  EXPECT_TRUE(is_close(make_diagnostics_clock(FakeNode{false})->now(), now(RCL_STEADY_TIME)));
}

TEST(MakeDiagnosticsClock, UsesRosTimeWithSimTime)
{
  ASSERT_FALSE(is_close(now(RCL_STEADY_TIME), now(RCL_ROS_TIME)));

  EXPECT_TRUE(is_close(make_diagnostics_clock(FakeNode{true})->now(), now(RCL_ROS_TIME)));
}

}  // namespace nebula::ros
