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

#include "nebula_ros/common/watchdog_timer.hpp"

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/robosense/robosense_common.hpp>
#include <nebula_decoders/nebula_decoders_robosense/robosense_driver.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <robosense_msgs/msg/robosense_scan.hpp>

#include <chrono>
#include <memory>

namespace nebula::ros
{
/// @brief Ros wrapper of robosense driver
class RobosenseDecoderWrapper
{
public:
  explicit RobosenseDecoderWrapper(
    rclcpp::Node * const parent_node,
    const std::shared_ptr<nebula::drivers::RobosenseHwInterface> & hw_interface,
    const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & config,
    const std::shared_ptr<const nebula::drivers::RobosenseCalibrationConfiguration> & calibration);

  void process_cloud_packet(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & new_config);

  nebula::Status status();

private:
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

  std::shared_ptr<nebula::drivers::RobosenseHwInterface> hw_interface_;
  std::shared_ptr<const drivers::RobosenseSensorConfiguration> sensor_cfg_;
  std::shared_ptr<const drivers::RobosenseCalibrationConfiguration> calibration_cfg_ptr_{};

  std::shared_ptr<drivers::RobosenseDriver> driver_ptr_;
  std::mutex mtx_driver_ptr_;

  rclcpp::Publisher<robosense_msgs::msg::RobosenseScan>::SharedPtr packets_pub_{};
  robosense_msgs::msg::RobosenseScan::UniquePtr current_scan_msg_{};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nebula_points_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_ex_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_base_pub_{};

  std::shared_ptr<WatchdogTimer> cloud_watchdog_;
};

}  // namespace nebula::ros
