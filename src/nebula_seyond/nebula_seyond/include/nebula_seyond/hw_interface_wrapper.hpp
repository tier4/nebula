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

#include <nebula_core_common/nebula_status.hpp>
#include <nebula_seyond_common/seyond_calibration_data.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_hw_interfaces/seyond_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace nebula::ros
{

class SeyondHwInterfaceWrapper
{
public:
  SeyondHwInterfaceWrapper(
    rclcpp::Node * parent_node,
    const std::shared_ptr<const nebula::drivers::SeyondSensorConfiguration> & config);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::SeyondSensorConfiguration> & new_config);

  [[nodiscard]] nebula::Status status() const;

  [[nodiscard]] std::shared_ptr<nebula::drivers::SeyondHwInterface> hw_interface() const;

  [[nodiscard]] std::shared_ptr<const nebula::drivers::SeyondCalibrationData> calibration() const;

private:
  std::shared_ptr<nebula::drivers::SeyondHwInterface> hw_interface_;
  std::shared_ptr<const nebula::drivers::SeyondSensorConfiguration> config_;
  std::shared_ptr<nebula::drivers::SeyondCalibrationData> calibration_;
  rclcpp::Logger logger_;
  nebula::Status status_;
};

}  // namespace nebula::ros
