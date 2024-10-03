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

#ifndef NEBULA_VelodyneRosDecoderTestVls128_H
#define NEBULA_VelodyneRosDecoderTestVls128_H

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/velodyne/velodyne_common.hpp>
#include <nebula_decoders/nebula_decoders_velodyne/velodyne_driver.hpp>
#include <rclcpp/rclcpp.hpp>

#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace nebula::ros
{
class VelodyneRosDecoderTest final : public rclcpp::Node
{
  std::shared_ptr<drivers::VelodyneDriver> driver_ptr_;
  Status wrapper_status_;

  std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> calibration_cfg_ptr_;
  std::shared_ptr<const drivers::VelodyneSensorConfiguration> sensor_cfg_ptr_;

  Status initialize_driver(
    std::shared_ptr<const drivers::VelodyneSensorConfiguration> sensor_configuration,
    std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> calibration_configuration);

  Status get_parameters(
    drivers::VelodyneSensorConfiguration & sensor_configuration,
    drivers::VelodyneCalibrationConfiguration & calibration_configuration);

  static inline std::chrono::nanoseconds seconds_to_chrono_nano_seconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

public:
  explicit VelodyneRosDecoderTest(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  void receive_scan_msg_callback(const velodyne_msgs::msg::VelodyneScan::SharedPtr scan_msg);
  Status get_status();
  void read_bag();

private:
  std::string bag_path_;
  std::string storage_id_;
  std::string format_;
  std::string target_topic_;
  std::string correction_file_path_;
};

}  // namespace nebula::ros

#endif  // NEBULA_VelodyneRosDecoderTestVls128_H
