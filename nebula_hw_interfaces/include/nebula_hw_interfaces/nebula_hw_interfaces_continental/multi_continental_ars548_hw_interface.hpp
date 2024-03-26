// Copyright 2024 Tier IV, Inc.
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

#ifndef NEBULA_MULTI_CONTINENTAL_ARS548_HW_INTERFACE_H
#define NEBULA_MULTI_CONTINENTAL_ARS548_HW_INTERFACE_H
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
#include <boost_tcp_driver/http_client_driver.hpp>
#include <boost_tcp_driver/tcp_driver.hpp>
#include <boost_udp_driver/udp_driver.hpp>
#include <nebula_common/continental/continental_ars548.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace nebula
{
namespace drivers
{
namespace continental_ars548
{
/// @brief Hardware interface of the Continental ARS548 radar
class MultiContinentalARS548HwInterface : NebulaHwInterfaceBase
{
private:
  std::unique_ptr<::drivers::common::IoContext> sensor_io_context_;
  std::vector<std::unique_ptr<::drivers::udp_driver::UdpDriver>> sensor_udp_drivers_;
  std::shared_ptr<MultiContinentalARS548SensorConfiguration> sensor_configuration_;
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> nebula_packets_ptr_;
  std::function<void(
    std::unique_ptr<nebula_msgs::msg::NebulaPackets> buffer, const std::string & sensor_ip)>
    nebula_packets_reception_callback_;

  std::mutex sensor_status_mutex_;

  SensorStatusPacket sensor_status_packet_{};
  FilterStatusPacket filter_status_{};
  std::unordered_map<std::string, std::string> sensor_ip_to_frame_;

  std::shared_ptr<rclcpp::Logger> parent_node_logger;

  /// @brief Printing the string to RCLCPP_INFO_STREAM
  /// @param info Target string
  void PrintInfo(std::string info);

  /// @brief Printing the string to RCLCPP_ERROR_STREAM
  /// @param error Target string
  void PrintError(std::string error);

  /// @brief Printing the string to RCLCPP_DEBUG_STREAM
  /// @param debug Target string
  void PrintDebug(std::string debug);

  /// @brief Printing the bytes to RCLCPP_DEBUG_STREAM
  /// @param bytes Target byte vector
  void PrintDebug(const std::vector<uint8_t> & bytes);

public:
  /// @brief Constructor
  MultiContinentalARS548HwInterface();

  /// @brief Process a new filter status packet
  /// @param buffer The buffer containing the status packet
  void ProcessFilterStatusPacket(const std::vector<uint8_t> & buffer);

  /// @brief Process a new data packet
  /// @param buffer The buffer containing the data packet
  void ProcessDataPacket(const std::vector<uint8_t> & buffer, const std::string & sensor_ip);

  /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void ReceiveSensorPacketCallback(
    const std::vector<uint8_t> & buffer, const std::string & sensor_ip);

  /// @brief Starting the interface that handles UDP streams
  /// @return Resulting status
  Status SensorInterfaceStart() final;

  /// @brief Function for stopping the interface that handles UDP streams
  /// @return Resulting status
  Status SensorInterfaceStop() final;

  /// @brief Printing sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration) final;

  /// @brief Setting sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status SetSensorConfiguration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration) final;

  /// @brief Registering callback for PandarScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterScanCallback(
    std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets>, const std::string &)>
      scan_callback);

  /// @brief Set the current lateral acceleration
  /// @param lateral_acceleration Current lateral acceleration
  /// @return Resulting status
  Status SetAccelerationLateralCog(float lateral_acceleration);

  /// @brief Set the current longitudinal acceleration
  /// @param longitudinal_acceleration Current longitudinal acceleration
  /// @return Resulting status
  Status SetAccelerationLongitudinalCog(float longitudinal_acceleration);

  /// @brief Set the characteristic speed
  /// @param characteristic_speed Characteristic speed
  /// @return Resulting status
  Status SetCharacteristicSpeed(float characteristic_speed);

  /// @brief Set the current direction
  /// @param direction Current driving direction
  /// @return Resulting status
  Status SetDrivingDirection(int direction);

  /// @brief Set the current steering angle
  /// @param angle_rad Current steering angle in radians
  /// @return Resulting status
  Status SetSteeringAngleFrontAxle(float angle_rad);

  /// @brief Set the current vehicle velocity
  /// @param velocity Current vehicle velocity
  /// @return Resulting status
  Status SetVelocityVehicle(float velocity);

  /// @brief Set the current yaw rate
  /// @param yaw_rate Current yaw rate
  /// @return Resulting status
  Status SetYawRate(float yaw_rate);

  /// @brief Checking the current settings and changing the difference point
  /// @return Resulting status
  Status CheckAndSetConfig();

  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> node);
};
}  // namespace continental_ars548
}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_MULTI_CONTINENTAL_ARS548_HW_INTERFACE_H
