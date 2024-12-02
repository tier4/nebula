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

#ifndef NEBULA_HesaiRosOfflineExtractBag_H
#define NEBULA_HesaiRosOfflineExtractBag_H

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp>
#include <nebula_ros/common/parameter_descriptors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <pandar_msgs/msg/pandar_packet.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>

#include <memory>
#include <string>

namespace nebula::ros
{
class HesaiRosOfflineExtractBag final : public rclcpp::Node
{
  std::shared_ptr<drivers::HesaiDriver> driver_ptr_;
  Status wrapper_status_;

  std::shared_ptr<drivers::HesaiCalibrationConfiguration> calibration_cfg_ptr_;
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr_;
  std::shared_ptr<drivers::HesaiCorrection> correction_cfg_ptr_;

  Status initialize_driver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::HesaiCalibrationConfigurationBase> calibration_configuration);

  Status get_parameters(
    drivers::HesaiSensorConfiguration & sensor_configuration,
    drivers::HesaiCalibrationConfiguration & calibration_configuration,
    drivers::HesaiCorrection & correction_configuration);

  static inline std::chrono::nanoseconds seconds_to_chrono_nano_seconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

public:
  explicit HesaiRosOfflineExtractBag(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  Status get_status();
  Status read_bag();

private:
  std::string bag_path_;
  std::string storage_id_;
  std::string out_path_;
  std::string format_;
  std::string input_topic_;
  std::string correction_file_path_;
  std::string frame_id_;
  std::string output_pointcloud_topic_;
  int out_num_;
  int skip_num_;
  bool only_xyz_;

  bool output_pcd_;
  bool output_rosbag_;
  bool forward_packets_to_rosbag_;
};

}  // namespace nebula::ros

#endif  // NEBULA_HesaiRosOfflineExtractBag_H
