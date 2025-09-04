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

// Have to define macros to silence warnings about deprecated headers being used by
// boost/property_tree/ in some versions of boost.
// See: https://github.com/boostorg/property_tree/issues/51
#include <boost/version.hpp>
#if (BOOST_VERSION / 100 >= 1073 && BOOST_VERSION / 100 <= 1076)  // Boost 1.73 - 1.76
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#endif
#if (BOOST_VERSION / 100 == 1074)  // Boost 1.74
#define BOOST_ALLOW_DEPRECATED_HEADERS
#endif

#include <boost_udp_driver/udp_driver.hpp>
#include <nebula_common/loggers/logger.hpp>
#include <nebula_common/robosense/robosense_common.hpp>

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
  std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
  std::unique_ptr<::drivers::common::IoContext> info_io_context_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> info_udp_driver_;
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
