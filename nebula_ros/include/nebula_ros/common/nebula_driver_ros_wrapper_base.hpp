// Copyright 2023 Tier IV, Inc.
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

#ifndef NEBULA_DRIVER_WRAPPER_BASE_H
#define NEBULA_DRIVER_WRAPPER_BASE_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <vector>

namespace nebula
{
namespace ros
{
/// @brief Base class for ros wrapper of each sensor driver
class NebulaDriverRosWrapperBase
{
public:
  NebulaDriverRosWrapperBase() = default;

  NebulaDriverRosWrapperBase(NebulaDriverRosWrapperBase && c) = delete;
  NebulaDriverRosWrapperBase & operator=(NebulaDriverRosWrapperBase && c) = delete;
  NebulaDriverRosWrapperBase(const NebulaDriverRosWrapperBase & c) = delete;
  NebulaDriverRosWrapperBase & operator=(const NebulaDriverRosWrapperBase & c) = delete;

private:
  /// @brief Virtual function for initializing ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  /// @return Resulting status
  virtual Status InitializeDriver(
    [[maybe_unused]] std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    [[maybe_unused]] std::shared_ptr<drivers::CalibrationConfigurationBase>
      calibration_configuration)
  {
    return Status::NOT_IMPLEMENTED;
  }

  //  status ReceiveScanMsgCallback(void * ScanMsg);  // ROS message callback for individual packet
  //  type

  /// @brief Point cloud publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace ros
}  // namespace nebula
#endif  // NEBULA_DRIVER_WRAPPER_BASE_H
