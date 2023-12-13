// Copyright 2023 Tier IV, Inc.
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

#ifndef NEBULA_CONTINENTAL_RADAR_ETHERNET_HW_INTERFACE_H
#define NEBULA_CONTINENTAL_RADAR_ETHERNET_HW_INTERFACE_H
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
#include "boost_tcp_driver/http_client_driver.hpp"
#include "boost_tcp_driver/tcp_driver.hpp"
#include "boost_udp_driver/udp_driver.hpp"
#include "nebula_common/continental/continental_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_types.hpp"

#include <rclcpp/rclcpp.hpp>

#include "nebula_msgs/msg/nebula_packet.hpp"
#include "nebula_msgs/msg/nebula_packets.hpp"

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
constexpr int SERVICE_ID_BYTE = 0;
constexpr int METHOD_ID_BYTE = 2;
constexpr int LENGTH_BYTE = 4;

constexpr int CONFIGURATION_METHOD_ID = 390;
constexpr int CONFIGURATION_PAYLOAD_LENGTH = 56;

constexpr int STATUS_TIMESTAMP_NANOSECONDS_BYTE = 8;
constexpr int STATUS_TIMESTAMP_SECONDS_BYTE = 12;
constexpr int STATUS_SYNC_STATUS_BYTE = 16;
constexpr int STATUS_SW_VERSION_MAJOR_BYTE = 17;
constexpr int STATUS_SW_VERSION_MINOR_BYTE = 18;
constexpr int STATUS_SW_VERSION_PATCH_BYTE = 19;

constexpr int STATUS_LONGITUDINAL_BYTE = 20;
constexpr int STATUS_LATERAL_BYTE = 24;
constexpr int STATUS_VERTICAL_BYTE = 28;
constexpr int STATUS_YAW_BYTE = 32;
constexpr int STATUS_PITCH_BYTE = 36;

constexpr int STATUS_PLUG_ORIENTATION_BYTE = 40;
constexpr int STATUS_LENGTH_BYTE = 41;
constexpr int STATUS_WIDTH_BYTE = 45;
constexpr int STATUS_HEIGHT_BYTE = 49;
constexpr int STATUS_WHEEL_BASE_BYTE = 53;
constexpr int STATUS_MAXIMUM_DISTANCE_BYTE = 57;
constexpr int STATUS_FREQUENCY_SLOT_BYTE = 59;
constexpr int STATUS_CYCLE_TIME_BYTE = 60;
constexpr int STATUS_TIME_SLOT_BYTE = 61;
constexpr int STATUS_HCC_BYTE = 62;
constexpr int STATUS_POWERSAVING_STANDSTILL_BYTE = 63;
constexpr int STATUS_SENSOR_IP_ADDRESS0_BYTE = 64;
constexpr int STATUS_SENSOR_IP_ADDRESS1_BYTE = 68;
constexpr int STATUS_CONFIGURATION_COUNTER_BYTE = 72;
constexpr int STATUS_LONGITUDINAL_VELOCITY_BYTE = 73;
constexpr int STATUS_LONGITUDINAL_ACCELERATION_BYTE = 74;
constexpr int STATUS_LATERAL_ACCELERATION_BYTE = 75;
constexpr int STATUS_YAW_RATE_BYTE = 76;
constexpr int STATUS_STEERING_ANGLE_BYTE = 77;
constexpr int STATUS_DRIVING_DIRECTION_BYTE = 78;
constexpr int STATUS_CHARASTERISTIC_SPEED_BYTE = 79;
constexpr int STATUS_RADAR_STATUS_BYTE = 80;
constexpr int STATUS_VOLTAGE_STATUS_BYTE = 81;
constexpr int STATUS_TEMPERATURE_STATUS_BYTE = 82;
constexpr int STATUS_BLOCKAGE_STATUS_BYTE = 83;

/// @brief Hardware interface of hesai driver
class ContinentalRadarEthernetHwInterface : NebulaHwInterfaceBase
{
private:
  std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> sensor_udp_driver_;
  std::shared_ptr<ContinentalRadarEthernetSensorConfiguration> sensor_configuration_;
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> nebula_packets_ptr_;
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets> buffer)>
    nebula_packets_reception_callback_;

  std::mutex sensor_status_mutex_;

  // KL move these to the continental header file
  static constexpr int DETECTION_FILTER_PROPERTIES_NUM = 7;
  static constexpr int OBJECT_FILTER_PROPERTIES_NUM = 24;

  struct FilterStatus
  {
    uint8_t active;
    uint8_t filter_id;
    uint8_t min_value;
    uint8_t max_value;
  };

  FilterStatus detection_filters_status_[DETECTION_FILTER_PROPERTIES_NUM];
  FilterStatus object_filters_status_[OBJECT_FILTER_PROPERTIES_NUM];

  ContinentalRadarStatus radar_status_;

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
  ContinentalRadarEthernetHwInterface();
  /// @brief Initializing tcp_driver for TCP communication
  /// @param setup_sensor Whether to also initialize tcp_driver for sensor configuration
  /// @return Resulting status

  void ProcessSensorStatusPacket(const std::vector<uint8_t> & buffer);
  void ProcessFilterStatusPacket(const std::vector<uint8_t> & buffer);
  void ProcessDataPacket(const std::vector<uint8_t> & buffer);

  /// @brief Parsing json string to property_tree
  /// @param str JSON string
  /// @return Parsed property_tree
  boost::property_tree::ptree ParseJson(const std::string & str);

  /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void ReceiveCloudPacketCallbackWithSender(
    const std::vector<uint8_t> & buffer, const std::string & sender_ip);
  /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) final;
  /// @brief Starting the interface that handles UDP streams
  /// @return Resulting status
  Status CloudInterfaceStart() final;
  /// @brief Function for stopping the interface that handles UDP streams
  /// @return Resulting status
  Status CloudInterfaceStop() final;
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

  /// @brief @brief Setting SensorMounting
  /// @param longitudinal_autosar Desired longitudinal value in autosar coordinates
  /// @param lateral_autosar Desired lateral value in autosar coordinates
  /// @param vertical_autosar Desired vertical value in autosar coordinates
  /// @param yaw_autosar Desired yaw value in autosar coordinates
  /// @param pitch_autosar Desired pitch value in autosar coordinates
  /// @param plug_orientation Desired plug orientation (0 = PLUG_RIGHT, 1 = PLUG_LEFT)
  /// @return Resulting status
  Status SetSensorMounting(
    float longitudinal_autosar, float lateral_autosar, float vertical_autosar, float yaw_autosar,
    float pitch_autosar, uint8_t plug_orientation);

  /// @brief @brief Setting SensorMounting
  /// @param length_autosar Desired vehicle length value
  /// @param width_autosar Desired vehicle width value
  /// @param height_autosar Desired height value
  /// @param wheel_base_autosar Desired wheel base value
  /// @return Resulting status
  Status SetVehicleParameters(
    float length_autosar, float width_autosar, float height_autosar, float wheel_base_autosar);

  /// @brief @brief Setting RadarParameters
  /// @param maximum_distance Desired maximum detection distance (93m <= v <= 1514m)
  /// @param frequency_slot Desired frequency slot (0 = Low (76.23 GHz), 1 = Mid (76.48 GHz), 2 =
  /// High (76.73 GHz))
  /// @param cycle_time Desired cycle time value (50ms <= v <= 100ms)
  /// @param time_slot Desired time slot value (10ms <= v <= 90ms)
  /// @param hcc Desired hcc value (1 = Worldwide, 2 = Japan)
  /// @param powersave_standstill Desired powersave_standstill value (0 = Off, 1 = On)
  /// @return Resulting status
  Status SetRadarParameters(
    uint16_t maximum_distance, uint8_t frequency_slot, uint8_t cycle_time, uint8_t time_slot,
    uint8_t hcc, uint8_t powersave_standstill);

  /// @brief @brief Setting RadarParameters
  /// @param sensor_ip_address Desired sensor ip address
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetSensorIPAddress(const std::string & sensor_ip_address);

  Status SetAccelerationLateralCog(float lateral_acceleration);
  Status SetAccelerationLongitudinalCog(float longitudinal_acceleration);
  Status SetCharasteristicSpeed(float charasteristic_speed);
  Status SetDrivingDirection(int direction);
  Status SetSteeringAngleFrontAxle(float angle_rad);
  Status SetVelocityVehicle(float velocity);
  Status SetYawRate(float yaw_rate);

  Status GetLidarMonitor(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  Status GetLidarMonitor(bool with_run = true);

  /// @brief Checking the current settings and changing the difference point
  /// @return Resulting status
  Status CheckAndSetConfig();

  /// @brief Returns the last semantic sensor status
  /// @return Last semantic sensor status message
  ContinentalRadarStatus GetRadarStatus();

  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> node);
};
}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_CONTINENTAL_RADAR_ETHERNET_HW_INTERFACE_H
