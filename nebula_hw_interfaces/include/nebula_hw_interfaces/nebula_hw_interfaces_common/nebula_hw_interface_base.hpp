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

#ifndef NEBULA_HW_INTERFACE_BASE_H
#define NEBULA_HW_INTERFACE_BASE_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <memory>
#include <vector>

namespace nebula::drivers
{
/// @brief Base class for hardware interface of each LiDAR
class NebulaHwInterfaceBase
{
protected:
  /**
   * Callback function to receive the Cloud Packet data from the UDP Driver
   * @param buffer Buffer containing the data received from the UDP socket
   * @return Status::OK if no error occurred.
   */
  virtual void receive_sensor_packet_callback([[maybe_unused]] std::vector<uint8_t> & buffer) {}
  //  virtual Status RegisterScanCallback(
  //    std::function<void(std::unique_ptr<std::vector<std::vector<uint8_t>>>)> scan_callback) = 0;

public:
  NebulaHwInterfaceBase(NebulaHwInterfaceBase && c) = delete;
  NebulaHwInterfaceBase & operator=(NebulaHwInterfaceBase && c) = delete;
  NebulaHwInterfaceBase(const NebulaHwInterfaceBase & c) = delete;
  NebulaHwInterfaceBase & operator=(const NebulaHwInterfaceBase & c) = delete;

  NebulaHwInterfaceBase() = default;

  /// @brief Virtual function for starting the interface that handles UDP streams
  /// @return Resulting status
  virtual Status sensor_interface_start() = 0;

  /// @brief Virtual function for stopping the interface that handles UDP streams
  /// @return Resulting status
  virtual Status sensor_interface_stop() = 0;
  // You may want to also implement GpsInterfaceStart() and ReceiveGpsCallback, but that is sensor
  // specific.

  /// @brief Virtual function for setting sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  virtual Status set_sensor_configuration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration) = 0;

  /// @brief Virtual function for printing sensor configuration
  /// @param sensor_configuration SensorConfiguration for the checking
  /// @return Resulting status
  virtual Status get_sensor_configuration(SensorConfigurationBase & sensor_configuration) = 0;

  /// @brief Virtual function for printing calibration configuration
  /// @param calibration_configuration CalibrationConfiguration for the checking
  /// @return Resulting status
  virtual Status get_calibration_configuration(
    [[maybe_unused]] CalibrationConfigurationBase & calibration_configuration)
  {
    return Status::NOT_IMPLEMENTED;
  }
};

}  // namespace nebula::drivers

#endif  // NEBULA_HW_INTERFACE_BASE_H
