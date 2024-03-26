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

#ifndef NEBULA_CONTINENTAL_SRR520_HW_INTERFACE_H
#define NEBULA_CONTINENTAL_SRR520_HW_INTERFACE_H
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
#include <nebula_common/continental/continental_srr520.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{
namespace continental_srr520
{
/// @brief Hardware interface of the Continental SRR520 radar
class ContinentalSRR520HwInterface : NebulaHwInterfaceBase
{
private:
  // std::unique_ptr<::drivers::common::IoContext> sensor_io_context_;
  // std::unique_ptr<::drivers::udp_driver::UdpDriver> sensor_udp_driver_;

  std::unique_ptr<::drivers::socketcan::SocketCanReceiver> can_receiver_;
  std::unique_ptr<::drivers::socketcan::SocketCanSender> can_sender_;
  std::unique_ptr<std::thread> receiver_thread_;

  std::shared_ptr<ContinentalSRR520SensorConfiguration> sensor_configuration_;
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> nebula_packets_ptr_;
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets> buffer)>
    nebula_packets_reception_callback_;

  std::mutex sensor_status_mutex_;
  std::mutex receiver_mutex_;
  bool sensor_interface_active_{};

  std::unique_ptr<nebula_msgs::msg::NebulaPackets> rdi_near_packets_ptr_{};
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> rdi_hrr_packets_ptr_{};
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> object_packets_ptr_{};

  bool first_rdi_near_packet_{true};
  bool first_rdi_hrr_packet_{true};
  bool first_object_packet_{true};

  uint8_t sync_counter_{0};
  bool sync_fup_sent_{true};
  builtin_interfaces::msg::Time last_sync_stamp_;

  std::shared_ptr<rclcpp::Logger> parent_node_logger;

  /// @brief Send a Fd frame
  /// @param data a buffer containing the data to send
  template <std::size_t N>
  bool SendFrame(const std::array<uint8_t, N> & data, int can_frame_id);

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

  /// @brief Main loop of the CAN receiver thread
  void ReceiveLoop();

  /// @brief Process a new near detection header packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessNearHeaderPacket(const std::vector<uint8_t> & buffer, const uint64_t stamp);

  /// @brief Process a new near element packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessNearElementPacket(const std::vector<uint8_t> & buffer, const uint64_t stamp);

  /// @brief Process a new hrr header packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessHRRHeaderPacket(const std::vector<uint8_t> & buffer, const uint64_t stamp);

  /// @brief Process a new hrr element packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessHRRElementPacket(const std::vector<uint8_t> & buffer, const uint64_t stamp);

  /// @brief Process a new object header packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessObjectHeaderPacket(const std::vector<uint8_t> & buffer, const uint64_t stamp);

  /// @brief Process a new object element packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessObjectElementPacket(const std::vector<uint8_t> & buffer, const uint64_t stamp);

  /// @brief Process a new crc list packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessCRCListPacket(const std::vector<uint8_t> & buffer, const uint64_t stamp);

  /// @brief Process a new Near detections crc list packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessNearCRCListPacket(const std::vector<uint8_t> & buffer, const uint64_t stamp);

  /// @brief Process a new HRR crc list packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessHRRCRCListPacket(
    const std::vector<uint8_t> & buffer, const uint64_t stamp);  // cspell:ignore HRRCRC

  /// @brief Process a new objects crc list packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessObjectCRCListPacket(const std::vector<uint8_t> & buffer, const uint64_t stamp);

  /// @brief Process a new sensor status packet
  /// @param buffer The buffer containing the status packet
  /// @param stamp The stamp in nanoseconds
  void ProcessSensorStatusPacket(const std::vector<uint8_t> & buffer, const uint64_t stamp);

  /// @brief Process a new Sync Fup packet
  /// @param buffer The buffer containing the packet
  /// @param stamp The stamp in nanoseconds
  void ProcessSyncFupPacket(const std::vector<uint8_t> & buffer, const uint64_t stamp);

  /// @brief Process a new filter status packet
  /// @param buffer The buffer containing the status packet
  void ProcessFilterStatusPacket(const std::vector<uint8_t> & buffer);

  /// @brief Process a new data packet
  /// @param buffer The buffer containing the data packet
  void ProcessDataPacket(const std::vector<uint8_t> & buffer);

  /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void ReceiveSensorPacketCallback(const std::vector<uint8_t> & buffer, int id, uint64_t stamp);

public:
  /// @brief Constructor
  ContinentalSRR520HwInterface();

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
    std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets>)> scan_callback);

  /// @brief Sensor synchronization routine
  void SensorSync();

  /// @brief Configure the sensor
  /// @param sensor_id Desired sensor id
  /// @param longitudinal_autosar Desired longitudinal value in autosar coordinates
  /// @param lateral_autosar Desired lateral value in autosar coordinates
  /// @param vertical_autosar Desired vertical value in autosar coordinates
  /// @param yaw_autosar Desired yaw value in autosar coordinates
  /// @param longitudinal_cog Desired longitudinal cog
  /// @param wheelbase Desired wheelbase
  /// @param cover_damping Desired cover damping
  /// @param plug_bottom Desired plug bottom
  /// @param reset Rest the sensor to its default values
  /// @return Resulting status
  Status ConfigureSensor(
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
  Status SetVehicleDynamics(
    float longitudinal_acceleration, float lateral_acceleration, float yaw_rate,
    float longitudinal_velocity, bool standstill);

  /// @brief Checking the current settings and changing the difference point
  /// @return Resulting status
  Status CheckAndSetConfig();

  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> node);
};
}  // namespace continental_srr520
}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_CONTINENTAL_SRR520_HW_INTERFACE_H
