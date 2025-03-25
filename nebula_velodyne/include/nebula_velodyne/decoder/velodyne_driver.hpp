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

#ifndef NEBULA_VELODYNE_DRIVER_H
#define NEBULA_VELODYNE_DRIVER_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/point_types.hpp"
#include "nebula_common/velodyne/velodyne_common.hpp"
#include "nebula_decoders/nebula_decoders_common/nebula_driver_base.hpp"
#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_scan_decoder.hpp"

#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace nebula::drivers
{
/// @brief Velodyne driver
class VelodyneDriver : NebulaDriverBase
{
private:
  /// @brief Current driver status
  Status driver_status_;
  /// @brief Decoder according to the model
  std::shared_ptr<drivers::VelodyneScanDecoder> scan_decoder_;

public:
  VelodyneDriver() = delete;
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  VelodyneDriver(
    const std::shared_ptr<const drivers::VelodyneSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> &
      calibration_configuration);

  /// @brief Setting CalibrationConfiguration (not used)
  /// @param calibration_configuration
  /// @return Resulting status
  Status set_calibration_configuration(
    const CalibrationConfigurationBase & calibration_configuration) override;

  /// @brief Get current status of this driver
  /// @return Current status
  Status get_status();

  /// @brief Convert VelodyneScan message to point cloud
  /// @param velodyne_scan Message
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> parse_cloud_packet(
    const std::vector<uint8_t> & packet, double packet_seconds);
};

}  // namespace nebula::drivers

#endif  // NEBULA_VELODYNE_DRIVER_H
