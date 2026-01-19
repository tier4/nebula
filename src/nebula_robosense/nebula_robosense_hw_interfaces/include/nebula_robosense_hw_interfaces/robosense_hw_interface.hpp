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

#include "nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp"

#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_robosense_common/robosense_common.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers
{
constexpr uint16_t mtu_size = 1248;
constexpr uint16_t helios_packet_size = 1248;
constexpr uint16_t helios_info_packet_size = 1248;
constexpr uint16_t bpearl_packet_size = 1248;
constexpr uint16_t bpearl_info_packet_size = 1248;

/// @brief Hardware interface of Robosense driver
class RobosenseHwInterface
{
private:
  std::unique_ptr<connections::UdpSocket> cloud_udp_socket_;
  std::unique_ptr<connections::UdpSocket> info_udp_socket_;
  std::shared_ptr<const RobosenseSensorConfiguration> sensor_configuration_;
  std::function<void(std::vector<uint8_t> & buffer)>
    scan_reception_callback_; /**This function pointer is called when the scan is complete*/
  std::function<void(std::vector<uint8_t> & buffer)>
    info_reception_callback_; /**This function pointer is called when DIFOP packet is received*/
  std::shared_ptr<loggers::Logger> logger_;

public:
  /// @brief Constructor
  explicit RobosenseHwInterface(const std::shared_ptr<loggers::Logger> & logger);

  /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void receive_sensor_packet_callback(std::vector<uint8_t> & buffer);

  /// @brief Callback function to receive the Info Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void receive_info_packet_callback(std::vector<uint8_t> & buffer);

  /// @brief Starting the interface that handles UDP streams for MSOP packets
  /// @return Resulting status
  Status sensor_interface_start();

  /// @brief Starting the interface that handles UDP streams for DIFOP packets
  /// @return Resulting status
  Status info_interface_start();

  /// @brief Setting sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status set_sensor_configuration(
    std::shared_ptr<const RobosenseSensorConfiguration> sensor_configuration);

  /// @brief Registering callback for RobosenseScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status register_scan_callback(std::function<void(std::vector<uint8_t> &)> scan_callback);

  /// @brief Registering callback for RobosensePacket
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status register_info_callback(std::function<void(std::vector<uint8_t> &)> info_callback);
};

}  // namespace nebula::drivers
