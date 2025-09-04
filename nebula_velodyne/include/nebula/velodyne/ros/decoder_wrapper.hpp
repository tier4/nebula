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

#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/common/watchdog_timer.hpp"

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/util/expected.hpp>
#include <nebula_common/velodyne/velodyne_common.hpp>
#include <nebula_decoders/nebula_decoders_velodyne/velodyne_driver.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_velodyne/velodyne_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace nebula::ros
{
class VelodyneDecoderWrapper
{
  using get_calibration_result_t = nebula::util::expected<
    std::shared_ptr<drivers::VelodyneCalibrationConfiguration>, nebula::Status>;

public:
  VelodyneDecoderWrapper(
    rclcpp::Node * const parent_node,
    const std::shared_ptr<nebula::drivers::VelodyneHwInterface> & hw_interface,
    std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config);

  void process_cloud_packet(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & new_config);

  void on_calibration_change(
    const std::shared_ptr<const nebula::drivers::VelodyneCalibrationConfiguration> &
      new_calibration);

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & p);

  nebula::Status status();

private:
  /// @brief Load calibration data from file
  /// @param calibration_file_path The file to read from
  /// @return The calibration data if successful, or an error code if not
  get_calibration_result_t get_calibration_data(const std::string & calibration_file_path);

  void publish_cloud(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds seconds_to_chrono_nano_seconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  nebula::Status status_;
  rclcpp::Logger logger_;

  const std::shared_ptr<nebula::drivers::VelodyneHwInterface> hw_interface_;
  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> sensor_cfg_;

  std::string calibration_file_path_{};
  std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> calibration_cfg_ptr_{};

  std::shared_ptr<drivers::VelodyneDriver> driver_ptr_{};
  std::mutex mtx_driver_ptr_;

  rclcpp::Publisher<velodyne_msgs::msg::VelodyneScan>::SharedPtr packets_pub_{};
  velodyne_msgs::msg::VelodyneScan::UniquePtr current_scan_msg_{};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nebula_points_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_ex_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_base_pub_{};

  std::shared_ptr<WatchdogTimer> cloud_watchdog_;
};
}  // namespace nebula::ros
