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

#ifndef NEBULA_CONTINENTAL_SRR520_HW_INTERFACE_H
#define NEBULA_CONTINENTAL_SRR520_HW_INTERFACE_H

#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"

#include <nebula_common/continental/continental_srr520.hpp>
#include <nebula_common/loggers/logger.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace nebula::drivers::continental_srr520
{
/// @brief Hardware interface of the Continental SRR520 radar
class ContinentalSRR520HwInterface
{
public:
  /// @brief Constructor
  explicit ContinentalSRR520HwInterface(const std::shared_ptr<loggers::Logger> & logger);

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
    const std::shared_ptr<
      const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration>
      new_config_ptr);

  /// @brief Registering callback
  /// @param callback Callback function
  /// @return Resulting status
  Status register_packet_callback(
    std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPacket>)> packet_callback);

  /// @brief Sensor synchronization routine
  void sensor_sync();

  /// @brief Process a new Sync Follow-up request
  void sensor_sync_follow_up(builtin_interfaces::msg::Time stamp);

  /// @brief Configure the sensor
  /// @param sensor_id Desired sensor id
  /// @param longitudinal_autosar Desired longitudinal value in autosar coordinates
  /// @param lateral_autosar Desired lateral value in AUTOSAR coordinates
  /// @param vertical_autosar Desired vertical value in autosar coordinates
  /// @param yaw_autosar Desired yaw value in autosar coordinates
  /// @param longitudinal_cog Desired longitudinal cog
  /// @param wheelbase Desired wheelbase
  /// @param cover_damping Desired cover damping
  /// @param plug_bottom Desired plug bottom
  /// @param reset Rest the sensor to its default values
  /// @return Resulting status
  Status configure_sensor(
    uint8_t sensor_id, float longitudinal_autosar, float lateral_autosar, float vertical_autosar,
    float yaw_autosar, float longitudinal_cog, float wheelbase, float cover_damping,
    bool plug_bottom, bool reset);

  /// @brief Set the current vehicle dynamics
  /// @param longitudinal_acceleration Longitudinal acceleration
  /// @param lateral_acceleration Lateral acceleration
  /// @param yaw_rate Yaw rate
  /// @param longitudinal_velocity Longitudinal velocity
  /// @param driving_direction Driving direction
  /// @return Resulting status
  Status set_vehicle_dynamics(
    float longitudinal_acceleration, float lateral_acceleration, float yaw_rate,
    float longitudinal_velocity, bool standstill);

private:
  /// @brief Send a Fd frame
  /// @param data a buffer containing the data to send
  template <std::size_t N>
  bool send_frame(const std::array<uint8_t, N> & data, int can_frame_id);

  /// @brief Printing the string to RCLCPP_INFO_STREAM
  /// @param info Target string
  void print_info(std::string info);

  /// @brief Printing the string to RCLCPP_ERROR_STREAM
  /// @param error Target string
  void print_error(std::string error);

  /// @brief Printing the string to RCLCPP_DEBUG_STREAM
  /// @param debug Target string
  void print_debug(std::string debug);

  /// @brief Main loop of the CAN receiver thread
  void receive_loop();

  std::unique_ptr<::drivers::socketcan::SocketCanReceiver> can_receiver_ptr_;
  std::unique_ptr<::drivers::socketcan::SocketCanSender> can_sender_ptr_;
  std::unique_ptr<std::thread> receiver_thread_ptr_;

  std::shared_ptr<const ContinentalSRR520SensorConfiguration> config_ptr_;
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPacket> buffer)>
    nebula_packet_callback_;

  std::mutex receiver_mutex_;
  bool sensor_interface_active_{};

  uint8_t sync_counter_{0};
  bool sync_follow_up_sent_{true};
  builtin_interfaces::msg::Time last_sync_stamp_;

  std::shared_ptr<loggers::Logger> logger_;
};
}  // namespace nebula::drivers::continental_srr520

#endif  // NEBULA_CONTINENTAL_SRR520_HW_INTERFACE_H
