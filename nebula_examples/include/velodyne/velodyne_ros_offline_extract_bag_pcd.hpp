// Copyright 2023 Map IV, Inc.
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

#ifndef NEBULA_VelodyneRosOfflineExtractBag_H
#define NEBULA_VelodyneRosOfflineExtractBag_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/velodyne/velodyne_calibration_decoder.hpp"
#include "nebula_common/velodyne/velodyne_common.hpp"
#include "nebula_decoders/nebula_decoders_velodyne/velodyne_driver.hpp"
#include "nebula_ros/common/nebula_driver_ros_wrapper_base.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

#include <regex>

#include <memory>
#include <string>

namespace nebula
{
namespace ros
{
/// @brief Offline velodyne driver usage example (Output PCD data)
class VelodyneRosOfflineExtractBag final : public rclcpp::Node, NebulaDriverRosWrapperBase
{
  std::shared_ptr<drivers::VelodyneDriver> driver_ptr_;
  Status wrapper_status_;

  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_cfg_ptr_;
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr_;

  /// @brief Initializing ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  /// @return Resulting status
  Status InitializeDriver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) override;

  /// @brief Get configurations from ros parameters
  /// @param sensor_configuration Output of SensorConfiguration
  /// @param calibration_configuration Output of CalibrationConfiguration
  /// @return Resulting status
  Status GetParameters(
    drivers::VelodyneSensorConfiguration & sensor_configuration,
    drivers::VelodyneCalibrationConfiguration & calibration_configuration);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

public:
  explicit VelodyneRosOfflineExtractBag(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Read the specified bag file and output point clouds to PCD files
  Status ReadBag();

private:
  std::string bag_path;
  std::string storage_id;
  std::string out_path;
  std::string format;
  std::string target_topic;
  int out_num;
  int skip_num;
  bool only_xyz;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_VelodyneRosOfflineExtractBag_H
