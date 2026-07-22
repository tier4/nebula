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

#include "nebula_hesai_hw_interfaces/hesai_cmd_response.hpp"

#include <nebula_hesai_common/hesai_common.hpp>
#include <nebula_hesai_hw_interfaces/hesai_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace nebula::ros
{
class HesaiHwInterfaceWrapper
{
public:
  HesaiHwInterfaceWrapper(
    rclcpp::Node * const parent_node,
    std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config,
    bool use_udp_only = false);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & new_config);

  nebula::Status status();

  [[nodiscard]] std::shared_ptr<drivers::HesaiHwInterface> hw_interface() const;

  [[nodiscard]] std::shared_ptr<const HesaiInventoryBase> inventory() const;

private:
  /// @brief Apply the sensor configuration via the HW interface, retrying on transient comms
  /// faults. Retries are bounded by g_hw_config_max_attempts when retry_hw is enabled (a single
  /// attempt otherwise).
  /// @throws std::runtime_error if the configuration cannot be applied after all attempts. Callers
  /// decide whether that is fatal: the constructor lets it propagate (aborting startup), while
  /// on_config_change catches it to keep a running node alive.
  void configure_sensor();

  std::shared_ptr<drivers::HesaiHwInterface> hw_interface_;
  std::shared_ptr<const HesaiInventoryBase> inventory_;

  rclcpp::Logger logger_;
  nebula::Status status_;
  bool setup_sensor_;
  bool use_udp_only_;
  bool retry_hw_;
};
}  // namespace nebula::ros
