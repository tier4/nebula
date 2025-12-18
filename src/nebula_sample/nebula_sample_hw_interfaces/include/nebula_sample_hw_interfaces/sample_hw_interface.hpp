// Copyright 2025 TIER IV, Inc.
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

#ifndef NEBULA_SAMPLE_HW_INTERFACE_HPP
#define NEBULA_SAMPLE_HW_INTERFACE_HPP

#include <nebula_core_common/nebula_status.hpp>
#include <nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp>
#include <nebula_sample_common/sample_common.hpp>

#include <memory>
#include <vector>

namespace nebula::drivers
{

/// @brief Hardware interface for Sample LiDAR communication
/// @details This class manages network communication with the sensor.
/// Responsibilities:
/// - Setting up UDP sockets for receiving data packets
/// - Starting/stopping packet reception
/// - Calling registered callbacks when packets arrive
/// - Optionally: sending commands to the sensor (start/stop scan, change settings, etc.)
///
/// @note Advanced implementations may also include:
/// - TCP connections for sensor configuration commands
/// - HTTP API support for newer sensor models
/// - These are optional and can be added as needed for your specific sensor
class SampleHwInterface
{
public:
  SampleHwInterface();

  /// @brief Start receiving packets from the sensor
  /// @return Status::OK on success, error status otherwise
  /// @details Implement the following:
  /// 1. Create UDP socket(s) for data reception
  /// 2. Bind to the configured port(s)
  /// 3. Start async receive loop
  /// 4. Optionally: send start command to sensor
  Status sensor_interface_start();

  /// @brief Stop receiving packets from the sensor
  /// @return Status::OK on success, error status otherwise
  /// @details Implement the following:
  /// 1. Stop the receive loop
  /// 2. Close UDP socket(s)
  /// 3. Optionally: send stop command to sensor
  Status sensor_interface_stop();

  /// @brief Set the sensor configuration
  /// @param sensor_configuration Configuration containing IP addresses, ports, etc.
  /// @return Status::OK on success, error status otherwise
  /// @details Store the configuration for use when starting the interface
  Status set_sensor_configuration(
    std::shared_ptr<const SampleSensorConfiguration> sensor_configuration);

  /// @brief Register callback for incoming packets
  /// @param scan_callback Function to call when a packet is received
  /// @return Status::OK on success, error status otherwise
  /// @details The callback receives raw packet data and metadata (timestamp, source IP, etc.)
  Status register_scan_callback(connections::UdpSocket::callback_t scan_callback);

private:
  std::shared_ptr<const SampleSensorConfiguration> sensor_configuration_;  ///< Sensor config
  connections::UdpSocket::callback_t cloud_packet_callback_;               ///< Packet callback
  // Implementation Items: Add member variables for:
  // - UDP socket instance(s) for data reception
  // - IO context for async operations
  // - Optionally: TCP driver for sensor configuration commands
  // - Optionally: HTTP client for newer sensor models
  // - Any sensor-specific state
};

}  // namespace nebula::drivers

#endif  // NEBULA_SAMPLE_HW_INTERFACE_HPP
