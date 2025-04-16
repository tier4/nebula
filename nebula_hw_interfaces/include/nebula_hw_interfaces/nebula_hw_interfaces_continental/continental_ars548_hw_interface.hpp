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

#ifndef NEBULA_CONTINENTAL_ARS548_HW_INTERFACE_H
#define NEBULA_CONTINENTAL_ARS548_HW_INTERFACE_H

#include "nebula_common/nebula_status.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp"

#include <boost_udp_driver/udp_driver.hpp>
#include <nebula_common/continental/continental_ars548.hpp>
#include <nebula_common/loggers/logger.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>

#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers::continental_ars548
{
/// @brief Hardware interface of the Continental ARS548 radar
class ContinentalARS548HwInterface
{
public:
  /// @brief Constructor
  explicit ContinentalARS548HwInterface(const std::shared_ptr<loggers::Logger> & logger);

  /// @brief Starting the interface that handles UDP streams
  /// @return Resulting status
  Status sensor_interface_start();

  /// @brief Function for stopping the interface that handles UDP streams
  /// @return Resulting status
  Status sensor_interface_stop();

  /// @brief Setting sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status set_sensor_configuration(
    std::shared_ptr<const ContinentalARS548SensorConfiguration> sensor_configuration);

  /// @brief Registering callback
  /// @param callback Callback function
  /// @return Resulting status
  Status register_packet_callback(
    std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPacket>)> packet_callback);

  /// @brief Set the sensor mounting parameters
  /// @param longitudinal_autosar Desired longitudinal value in autosar coordinates
  /// @param lateral_autosar Desired lateral value in autosar coordinates
  /// @param vertical_autosar Desired vertical value in autosar coordinates
  /// @param yaw_autosar Desired yaw value in autosar coordinates
  /// @param pitch_autosar Desired pitch value in autosar coordinates
  /// @param plug_orientation Desired plug orientation (0 = PLUG_RIGHT, 1 = PLUG_LEFT)
  /// @return Resulting status
  Status set_sensor_mounting(
    float longitudinal_autosar, float lateral_autosar, float vertical_autosar, float yaw_autosar,
    float pitch_autosar, uint8_t plug_orientation);

  /// @brief Set the vehicle parameters
  /// @param length_autosar Desired vehicle length value
  /// @param width_autosar Desired vehicle width value
  /// @param height_autosar Desired height value
  /// @param wheel_base_autosar Desired wheel base value
  /// @return Resulting status
  Status set_vehicle_parameters(
    float length_autosar, float width_autosar, float height_autosar, float wheel_base_autosar);

  /// @brief Set the radar parameters
  /// @param maximum_distance Desired maximum detection distance (93m <= v <= 1514m)
  /// @param frequency_slot Desired frequency slot (0 = Low (76.23 GHz), 1 = Mid (76.48 GHz), 2 =
  /// High (76.73 GHz))
  /// @param cycle_time Desired cycle time value (50ms <= v <= 100ms)
  /// @param time_slot Desired time slot value (10ms <= v <= 90ms)
  /// @param hcc Desired hcc value (1 = Worldwide, 2 = Japan)
  /// @param power_save_standstill Desired power_save_standstill value (0 = Off, 1 = On)
  /// @return Resulting status
  Status set_radar_parameters(
    uint16_t maximum_distance, uint8_t frequency_slot, uint8_t cycle_time, uint8_t time_slot,
    uint8_t hcc, uint8_t power_save_standstill);

  /// @brief Set the sensor ip address
  /// @param sensor_ip_address Desired sensor ip address
  /// @return Resulting status
  Status set_sensor_ip_address(const std::string & sensor_ip_address);

  /// @brief Set the current lateral acceleration
  /// @param lateral_acceleration Current lateral acceleration
  /// @return Resulting status
  Status set_acceleration_lateral_cog(float lateral_acceleration);

  /// @brief Set the current longitudinal acceleration
  /// @param longitudinal_acceleration Current longitudinal acceleration
  /// @return Resulting status
  Status set_acceleration_longitudinal_cog(float longitudinal_acceleration);

  /// @brief Set the characteristic speed
  /// @param characteristic_speed Characteristic speed
  /// @return Resulting status
  Status set_characteristic_speed(float characteristic_speed);

  /// @brief Set the current direction
  /// @param direction Current driving direction
  /// @return Resulting status
  Status set_driving_direction(int direction);

  /// @brief Set the current steering angle
  /// @param angle_rad Current steering angle in radians
  /// @return Resulting status
  Status set_steering_angle_front_axle(float angle_rad);

  /// @brief Set the current vehicle velocity
  /// @param velocity Current vehicle velocity
  /// @return Resulting status
  Status set_velocity_vehicle(float velocity);

  /// @brief Set the current yaw rate
  /// @param yaw_rate Current yaw rate
  /// @return Resulting status
  Status set_yaw_rate(float yaw_rate);

private:
  /// @brief Printing the string to RCLCPP_INFO_STREAM
  /// @param info Target string
  void print_info(std::string info);

  /// @brief Printing the string to RCLCPP_ERROR_STREAM
  /// @param error Target string
  void print_error(std::string error);

  /// @brief Printing the string to RCLCPP_DEBUG_STREAM
  /// @param debug Target string
  void print_debug(std::string debug);

  /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  /// @param metadata Metadata of the received packet
  void receive_sensor_packet_callback(
    const std::vector<uint8_t> & buffer, const connections::UdpSocket::RxMetadata & metadata);

  /// @brief Try to send a buffer via UDP and return an error on failure. Never throws.
  /// @param buffer Buffer to send
  /// @return Resulting status
  [[nodiscard]] Status safe_send(const std::vector<uint8_t> & buffer);

  std::optional<connections::UdpSocket> udp_socket_;
  std::shared_ptr<const ContinentalARS548SensorConfiguration> config_ptr_;
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPacket>)> packet_callback_;
  std::shared_ptr<loggers::Logger> logger_;
};
}  // namespace nebula::drivers::continental_ars548

#endif  // NEBULA_CONTINENTAL_ARS548_HW_INTERFACE_H
