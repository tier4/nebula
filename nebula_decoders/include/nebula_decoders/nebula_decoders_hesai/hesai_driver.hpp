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

#ifndef NEBULA_HESAI_DRIVER_H
#define NEBULA_HESAI_DRIVER_H

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/point_types.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"

#include <nebula_common/loggers/logger.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <tuple>
#include <vector>

namespace nebula::drivers
{
/// @brief Hesai driver
class HesaiDriver
{
private:
  /// @brief Current driver status
  Status driver_status_;
  std::shared_ptr<loggers::Logger> logger_;
  /// @brief Decoder according to the model
  std::shared_ptr<HesaiScanDecoder> scan_decoder_;

  template <typename SensorT>
  std::shared_ptr<HesaiScanDecoder> initialize_decoder(
    const std::shared_ptr<const drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> &
      calibration_configuration);

public:
  HesaiDriver() = delete;
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver (either
  /// HesaiCalibrationConfiguration for sensors other than AT128 or HesaiCorrection for AT128)
  explicit HesaiDriver(
    const std::shared_ptr<const drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> &
      calibration_configuration,
    const std::shared_ptr<loggers::Logger> & logger);

  /// @brief Get current status of this driver
  /// @return Current status
  Status get_status();

  /// @brief Setting CalibrationConfiguration (not used)
  /// @param calibration_configuration
  /// @return Resulting status
  Status set_calibration_configuration(
    const HesaiCalibrationConfigurationBase & calibration_configuration);

  /// @brief Convert raw packet to pointcloud
  /// @param packet Packet to convert
  /// @return Tuple of pointcloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> parse_cloud_packet(
    const std::vector<uint8_t> & packet);
};

}  // namespace nebula::drivers

#endif  // NEBULA_HESAI_DRIVER_H
