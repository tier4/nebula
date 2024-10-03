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

#ifndef NEBULA_HesaiRosOfflineExtractSample_H
#define NEBULA_HesaiRosOfflineExtractSample_H

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp>
#include <nebula_ros/common/parameter_descriptors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pandar_msgs/msg/pandar_packet.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>

#include <memory>
#include <string>

namespace nebula::ros
{
/// @brief Offline hesai driver usage example (Output PCD data)
class HesaiRosOfflineExtractSample final : public rclcpp::Node
{
  std::shared_ptr<drivers::HesaiDriver> driver_ptr_;
  Status wrapper_status_;

  std::shared_ptr<drivers::HesaiCalibrationConfiguration> calibration_cfg_ptr_;
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr_;
  std::shared_ptr<drivers::HesaiCorrection> correction_cfg_ptr_;

  /// @brief Initializing ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  /// @return Resulting status
  Status initialize_driver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::HesaiCalibrationConfigurationBase> calibration_configuration);

  /// @brief Get configurations from ros parameters
  /// @param sensor_configuration Output of SensorConfiguration
  /// @param calibration_configuration Output of CalibrationConfiguration
  /// @param correction_configuration Output of CorrectionConfiguration (for AT)
  /// @return Resulting status
  Status get_parameters(
    drivers::HesaiSensorConfiguration & sensor_configuration,
    drivers::HesaiCalibrationConfiguration & calibration_configuration,
    drivers::HesaiCorrection & correction_configuration);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds seconds_to_chrono_nano_seconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

public:
  explicit HesaiRosOfflineExtractSample(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  /// @brief Get current status of this driver
  /// @return Current status
  Status get_status();

  /// @brief Read the specified bag file and output point clouds to PCD files
  Status read_bag();

private:
  std::string bag_path_;
  std::string storage_id_;
  std::string out_path_;
  std::string format_;
  std::string target_topic_;
  std::string correction_file_path_;
};

}  // namespace nebula::ros

#endif  // NEBULA_HesaiRosOfflineExtractSample_H
