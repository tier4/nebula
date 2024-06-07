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

#include "nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp"
#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/common/watchdog_timer.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nebula_msgs/msg/nebula_packet.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace nebula
{
namespace ros
{
class HesaiDecoderWrapper
{
  using get_calibration_result_t = nebula::util::expected<
    std::shared_ptr<drivers::HesaiCalibrationConfigurationBase>, nebula::Status>;

public:
  HesaiDecoderWrapper(
    rclcpp::Node * const parent_node,
    const std::shared_ptr<nebula::drivers::HesaiHwInterface> & hw_interface,
    std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config);

  void ProcessCloudPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void OnConfigChange(
    const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & new_config);

  void OnCalibrationChange(
    const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> &
      new_calibration);

  rcl_interfaces::msg::SetParametersResult OnParameterChange(
    const std::vector<rclcpp::Parameter> & p);

  nebula::Status Status();

private:
  /// @brief Load calibration data from the best available source:
  /// 1. If sensor connected, download and save from sensor
  /// 2. If downloaded file available, load that file
  /// 3. Load the file given by `calibration_file_path`
  /// @param calibration_file_path The file to use if no better option is available
  /// @param ignore_others If true, skip straight so step 3 above, ignoring better calibration
  /// options
  /// @return The calibration data if successful, or an error code if not
  get_calibration_result_t GetCalibrationData(
    const std::string & calibration_file_path, bool ignore_others = false);

  void PublishCloud(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  nebula::Status status_;
  rclcpp::Logger logger_;

  const std::shared_ptr<nebula::drivers::HesaiHwInterface> hw_interface_;
  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> sensor_cfg_;

  std::string calibration_file_path_{};
  std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> calibration_cfg_ptr_{};

  std::shared_ptr<drivers::HesaiDriver> driver_ptr_{};
  std::mutex mtx_driver_ptr_;

  rclcpp::Publisher<pandar_msgs::msg::PandarScan>::SharedPtr packets_pub_{};
  pandar_msgs::msg::PandarScan::UniquePtr current_scan_msg_{};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nebula_points_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_ex_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_base_pub_{};

  std::shared_ptr<WatchdogTimer> cloud_watchdog_;
};
}  // namespace ros
}  // namespace nebula
