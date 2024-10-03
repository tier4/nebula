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

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/robosense/robosense_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace nebula::ros
{
class RobosenseHwInterfaceWrapper
{
public:
  explicit RobosenseHwInterfaceWrapper(
    rclcpp::Node * const parent_node,
    std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & config);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & new_config);

  nebula::Status status();

  std::shared_ptr<drivers::RobosenseHwInterface> hw_interface() const;

private:
  std::shared_ptr<drivers::RobosenseHwInterface> hw_interface_;
  rclcpp::Logger logger_;
  nebula::Status status_;
};

}  // namespace nebula::ros
