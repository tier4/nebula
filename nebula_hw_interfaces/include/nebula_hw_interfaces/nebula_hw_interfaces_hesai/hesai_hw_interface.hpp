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

#ifndef NEBULA_HESAI_HW_INTERFACE_H
#define NEBULA_HESAI_HW_INTERFACE_H
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
#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/hesai/hesai_status.hpp"
#include "nebula_common/util/expected.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp"

#include <rclcpp/rclcpp.hpp>

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
const int PandarTcpCommandPort = 9347;
const uint8_t PTC_COMMAND_DUMMY_BYTE = 0x00;
const uint8_t PTC_COMMAND_HEADER_HIGH = 0x47;
const uint8_t PTC_COMMAND_HEADER_LOW = 0x74;
const uint8_t PTC_COMMAND_GET_LIDAR_CALIBRATION = 0x05;
const uint8_t PTC_COMMAND_PTP_DIAGNOSTICS = 0x06;
const uint8_t PTC_COMMAND_PTP_STATUS = 0x01;
const uint8_t PTC_COMMAND_PTP_PORT_DATA_SET = 0x02;
const uint8_t PTC_COMMAND_PTP_TIME_STATUS_NP = 0x03;
const uint8_t PTC_COMMAND_PTP_GRANDMASTER_SETTINGS_NP = 0x04;
const uint8_t PTC_COMMAND_GET_INVENTORY_INFO = 0x07;
const uint8_t PTC_COMMAND_GET_CONFIG_INFO = 0x08;
const uint8_t PTC_COMMAND_GET_LIDAR_STATUS = 0x09;
const uint8_t PTC_COMMAND_SET_SPIN_RATE = 0x17;
const uint8_t PTC_COMMAND_SET_SYNC_ANGLE = 0x18;
const uint8_t PTC_COMMAND_SET_TRIGGER_METHOD = 0x1b;
const uint8_t PTC_COMMAND_SET_STANDBY_MODE = 0x1c;
const uint8_t PTC_COMMAND_SET_RETURN_MODE = 0x1e;
const uint8_t PTC_COMMAND_SET_CLOCK_SOURCE = 0x1f;
const uint8_t PTC_COMMAND_SET_DESTINATION_IP = 0x20;
const uint8_t PTC_COMMAND_SET_CONTROL_PORT = 0x21;
const uint8_t PTC_COMMAND_SET_LIDAR_RANGE = 0x22;
const uint8_t PTC_COMMAND_GET_LIDAR_RANGE = 0x23;
const uint8_t PTC_COMMAND_SET_PTP_CONFIG = 0x24;
const uint8_t PTC_COMMAND_GET_PTP_CONFIG = 0x26;
const uint8_t PTC_COMMAND_RESET = 0x25;
const uint8_t PTC_COMMAND_SET_ROTATE_DIRECTION = 0x2a;
const uint8_t PTC_COMMAND_LIDAR_MONITOR = 0x27;

const uint8_t PTC_ERROR_CODE_NO_ERROR = 0x00;
const uint8_t PTC_ERROR_CODE_INVALID_INPUT_PARAM = 0x01;
const uint8_t PTC_ERROR_CODE_SERVER_CONN_FAILED = 0x02;
const uint8_t PTC_ERROR_CODE_INVALID_DATA = 0x03;
const uint8_t PTC_ERROR_CODE_OUT_OF_MEMORY = 0x04;
const uint8_t PTC_ERROR_CODE_UNSUPPORTED_CMD = 0x05;
const uint8_t PTC_ERROR_CODE_FPGA_COMM_FAILED = 0x06;
const uint8_t PTC_ERROR_CODE_OTHER = 0x07;

const uint8_t TCP_ERROR_UNRELATED_RESPONSE = 1;
const uint8_t TCP_ERROR_UNEXPECTED_PAYLOAD = 2;
const uint8_t TCP_ERROR_TIMEOUT = 4;
const uint8_t TCP_ERROR_INCOMPLETE_RESPONSE = 8;

const uint16_t PANDARQT64_PACKET_SIZE = 1072;
const uint16_t PANDARQT128_PACKET_SIZE = 1127;
const uint16_t PANDARXT32_PACKET_SIZE = 1080;
const uint16_t PANDARXT32M_PACKET_SIZE = 820;
const uint16_t PANDARAT128_PACKET_SIZE = 1118;
const uint16_t PANDAR64_PACKET_SIZE = 1194;
const uint16_t PANDAR64_EXTENDED_PACKET_SIZE = 1198;
const uint16_t PANDAR40_PACKET_SIZE = 1262;
const uint16_t PANDAR40P_EXTENDED_PACKET_SIZE = 1266;
const uint16_t PANDAR128_E4X_PACKET_SIZE = 861;
const uint16_t PANDAR128_E4X_EXTENDED_PACKET_SIZE = 1117;
const uint16_t MTU_SIZE = 1500;

// Time interval between Announce messages, in units of log seconds (default: 1)
const int PTP_LOG_ANNOUNCE_INTERVAL = 1;
// Time interval between Sync messages, in units of log seconds (default: 1)
const int PTP_SYNC_INTERVAL = 1;
// Minimum permitted mean time between Delay_Req messages, in units of log seconds (default: 0)
const int PTP_LOG_MIN_DELAY_INTERVAL = 0;

const int HESAI_LIDAR_GPS_CLOCK_SOURCE = 0;
const int HESAI_LIDAR_PTP_CLOCK_SOURCE = 1;

/// @brief Hardware interface of hesai driver
class HesaiHwInterface
{
private:
  struct ptc_error_t
  {
    uint8_t error_flags = 0;
    uint8_t ptc_error_code = 0;

    [[nodiscard]] bool ok() const { return !error_flags && !ptc_error_code; }
  };

  using ptc_cmd_result_t = nebula::util::expected<std::vector<uint8_t>, ptc_error_t>;

  std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
  std::shared_ptr<boost::asio::io_context> m_owned_ctx;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> tcp_driver_;
  std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration_;
  std::function<void(std::vector<uint8_t> & buffer)>
    cloud_packet_callback_; /**This function pointer is called when the scan is complete*/

  std::mutex mtx_inflight_tcp_request_;

  int target_model_no;

  /// @brief Get a one-off HTTP client to communicate with the hardware
  /// @param ctx IO Context
  /// @param hcd Got http client driver
  /// @return Resulting status
  HesaiStatus GetHttpClientDriverOnce(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd);
  /// @brief Get a one-off HTTP client to communicate with the hardware (without specifying
  /// io_context)
  /// @param hcd Got http client driver
  /// @return Resulting status
  HesaiStatus GetHttpClientDriverOnce(
    std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd);
  /// @brief A callback that receives a string (just prints)
  /// @param str Received string
  void str_cb(const std::string & str);

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

  /// @brief Convert an error code to a human-readable string
  /// @param error_code The error code, containing the sensor's error code (if any), along with
  /// flags such as TCP_ERROR_UNRELATED_RESPONSE etc.
  /// @return A string description of all errors in this code
  std::string PrettyPrintPTCError(ptc_error_t error_code);

  /// @brief Checks if the data size matches that of the struct to be parsed, and parses the struct.
  /// If data is too small, a std::runtime_error is thrown. If data is too large, a warning is
  /// printed and the struct is parsed with the first sizeof(T) bytes.
  template <typename T>
  T CheckSizeAndParse(const std::vector<uint8_t> & data);

  /// @brief Send a PTC request with an optional payload, and return the full response payload.
  /// Blocking.
  /// @param command_id PTC command number.
  /// @param payload Payload bytes of the PTC command. Not including the 8-byte PTC header.
  /// @return The returned payload, if successful, or nullptr.
  ptc_cmd_result_t SendReceive(const uint8_t command_id, const std::vector<uint8_t> & payload = {});

public:
  /// @brief Constructor
  HesaiHwInterface();
  /// @brief Destructor
  ~HesaiHwInterface();
  /// @brief Initializing tcp_driver for TCP communication
  /// @param setup_sensor Whether to also initialize tcp_driver for sensor configuration
  /// @return Resulting status
  Status InitializeTcpDriver();
  /// @brief Closes the TcpDriver and related resources
  /// @return Status result
  Status FinalizeTcpDriver();
  /// @brief Parsing json string to property_tree
  /// @param str JSON string
  /// @return Parsed property_tree
  boost::property_tree::ptree ParseJson(const std::string & str);

  /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void ReceiveSensorPacketCallback(std::vector<uint8_t> & buffer);
  /// @brief Starting the interface that handles UDP streams
  /// @return Resulting status
  Status SensorInterfaceStart();
  /// @brief Function for stopping the interface that handles UDP streams
  /// @return Resulting status
  Status SensorInterfaceStop();
  /// @brief Printing sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status GetSensorConfiguration(const SensorConfigurationBase & sensor_configuration);
  /// @brief Printing calibration configuration
  /// @param calibration_configuration CalibrationConfiguration for the checking
  /// @return Resulting status
  Status GetCalibrationConfiguration(CalibrationConfigurationBase & calibration_configuration);
  /// @brief Setting sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status SetSensorConfiguration(
    std::shared_ptr<const SensorConfigurationBase> sensor_configuration);
  /// @brief Registering callback for PandarScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterScanCallback(std::function<void(std::vector<uint8_t> &)> scan_callback);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @return Resulting status
  std::string GetLidarCalibrationString();
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @return Resulting status
  std::vector<uint8_t> GetLidarCalibrationBytes();
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP STATUS)
  /// @return Resulting status
  HesaiPtpDiagStatus GetPtpDiagStatus();
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV PORT_DATA_SET)
  /// @return Resulting status
  HesaiPtpDiagPort GetPtpDiagPort();
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV TIME_STATUS_NP)
  /// @return Resulting status
  HesaiPtpDiagTime GetPtpDiagTime();
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV GRANDMASTER_SETTINGS_NP)
  /// @return Resulting status
  HesaiPtpDiagGrandmaster GetPtpDiagGrandmaster();
  /// @brief Getting data with PTC_COMMAND_GET_INVENTORY_INFO
  /// @return Resulting status
  HesaiInventory GetInventory();
  /// @brief Getting data with PTC_COMMAND_GET_CONFIG_INFO
  /// @return Resulting status
  HesaiConfig GetConfig();
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_STATUS
  /// @return Resulting status
  HesaiLidarStatus GetLidarStatus();
  /// @brief Setting value with PTC_COMMAND_SET_SPIN_RATE
  /// @param rpm Spin rate
  /// @return Resulting status
  Status SetSpinRate(uint16_t rpm);
  /// @brief Setting value with PTC_COMMAND_SET_SYNC_ANGLE
  /// @param sync_angle Sync angle enable flag
  /// @param angle Angle value
  /// @return Resulting status
  Status SetSyncAngle(int sync_angle, int angle);
  /// @brief Setting mode with PTC_COMMAND_SET_TRIGGER_METHOD
  /// @param trigger_method Trigger method
  /// @return Resulting status
  Status SetTriggerMethod(int trigger_method);
  /// @brief Setting mode with PTC_COMMAND_SET_STANDBY_MODE
  /// @param standby_mode Standby mode
  /// @return Resulting status
  Status SetStandbyMode(int standby_mode);
  /// @brief Setting mode with PTC_COMMAND_SET_RETURN_MODE
  /// @param return_mode Return mode
  /// @return Resulting status
  Status SetReturnMode(int return_mode);
  /// @brief Setting IP with PTC_COMMAND_SET_DESTINATION_IP
  /// @param dest_ip_1 The 1st byte represents the 1st section
  /// @param dest_ip_2 The 2nd byte represents the 2nd section
  /// @param dest_ip_3 The 3rd byte represents the 3rd section
  /// @param dest_ip_4 The 4th byte represents the 4th section
  /// @param port LiDAR Destination Port
  /// @param gps_port GPS Destination Port
  /// @return Resulting status
  Status SetDestinationIp(
    int dest_ip_1, int dest_ip_2, int dest_ip_3, int dest_ip_4, int port, int gps_port);
  /// @brief Setting IP with PTC_COMMAND_SET_CONTROL_PORT
  /// @param ip_1 Device IP of the 1st byte represents the 1st section
  /// @param ip_2 Device IP of the 2nd byte represents the 2nd section
  /// @param ip_3 Device IP of the 3rd byte represents the 3rd section
  /// @param ip_4 Device IP of the 4th byte represents the 4th section
  /// @param mask_1 Device subnet mask of the 1st byte represents the 1st section
  /// @param mask_2 Device subnet mask of the 2nd byte represents the 2nd section
  /// @param mask_3 Device subnet mask of the 3rd byte represents the 3rd section
  /// @param mask_4 Device subnet mask of the 4th byte represents the 4th section
  /// @param gateway_1 Device gateway of the 1st byte represents the 1st section
  /// @param gateway_2 Device gateway of the 2nd byte represents the 2nd section
  /// @param gateway_3 Device gateway of the 3rd byte represents the 3rd section
  /// @param gateway_4 Device gateway of the 4th byte represents the 4th section
  /// @param vlan_flg VLAN Status
  /// @param vlan_id VLAN ID
  /// @return Resulting status
  Status SetControlPort(
    int ip_1, int ip_2, int ip_3, int ip_4, int mask_1, int mask_2, int mask_3, int mask_4,
    int gateway_1, int gateway_2, int gateway_3, int gateway_4, int vlan_flg, int vlan_id);
  /// @brief Setting values with PTC_COMMAND_SET_LIDAR_RANGE
  /// @param method Method
  /// @param data Set data
  /// @return Resulting status
  Status SetLidarRange(int method, std::vector<unsigned char> data);
  /// @brief Setting values with PTC_COMMAND_SET_LIDAR_RANGE
  /// @param start Start angle
  /// @param end End angle
  /// @return Resulting status
  Status SetLidarRange(int start, int end);
  /// @brief Getting values with PTC_COMMAND_GET_LIDAR_RANGE
  /// @return Resulting status
  HesaiLidarRangeAll GetLidarRange();

  /**
   * @brief Given the HW interface's sensor configuration and a given calibration, set the sensor
   * FoV (min and max angles) with appropriate padding around the FoV set in the configuration. This
   * compensates for the points lost due to the sensor filtering FoV by raw encoder angle.
   *
   * @param calibration The calibration file of the sensor
   * @return Status If the FoV was updated correctly
   */
  [[nodiscard]] Status checkAndSetLidarRange(const HesaiCalibrationConfigurationBase & calibration);

  Status SetClockSource(int clock_source);

  /// @brief Setting values with PTC_COMMAND_SET_PTP_CONFIG
  /// @param profile IEEE timing and synchronization standard
  /// @param domain Domain attribute of the local clock
  /// @param network Network transport type of 1588v2
  /// @param switch_type Switch type of 802.1AS Automotive
  /// @param logAnnounceInterval Time interval between Announce messages, in units of log seconds
  /// (default: 1)
  /// @param logSyncInterval Time interval between Sync messages, in units of log seconds (default:
  /// 1)
  /// @param logMinDelayReqInterval Minimum permitted mean time between Delay_Req messages, in units
  /// of log seconds (default: 0)
  /// @return Resulting status
  Status SetPtpConfig(
    int profile, int domain, int network, int switch_type, int logAnnounceInterval = 1,
    int logSyncInterval = 1, int logMinDelayReqInterval = 0);
  /// @brief Getting data with PTC_COMMAND_GET_PTP_CONFIG
  /// @return Resulting status
  HesaiPtpConfig GetPtpConfig();
  /// @brief Sending command with PTC_COMMAND_RESET
  /// @return Resulting status
  Status SendReset();
  /// @brief Setting values with PTC_COMMAND_SET_ROTATE_DIRECTION
  /// @param mode Rotation of the motor
  /// @return Resulting status
  Status SetRotDir(int mode);
  /// @brief Getting data with PTC_COMMAND_LIDAR_MONITOR
  /// @return Resulting status
  HesaiLidarMonitor GetLidarMonitor();

  /// @brief Call run() of IO Context
  void IOContextRun();
  /// @brief GetIO Context
  /// @return IO Context
  std::shared_ptr<boost::asio::io_context> GetIOContext();

  /// @brief Setting spin_speed via HTTP API
  /// @param ctx IO Context used
  /// @param rpm spin_speed (300, 600, 1200)
  /// @return Resulting status
  HesaiStatus SetSpinSpeedAsyncHttp(std::shared_ptr<boost::asio::io_context> ctx, uint16_t rpm);
  /// @brief Setting spin_speed via HTTP API
  /// @param rpm spin_speed (300, 600, 1200)
  /// @return Resulting status
  HesaiStatus SetSpinSpeedAsyncHttp(uint16_t rpm);

  HesaiStatus SetPtpConfigSyncHttp(
    std::shared_ptr<boost::asio::io_context> ctx, int profile, int domain, int network,
    int logAnnounceInterval, int logSyncInterval, int logMinDelayReqInterval);
  HesaiStatus SetPtpConfigSyncHttp(
    int profile, int domain, int network, int logAnnounceInterval, int logSyncInterval,
    int logMinDelayReqInterval);
  HesaiStatus SetSyncAngleSyncHttp(
    std::shared_ptr<boost::asio::io_context> ctx, int enable, int angle);
  HesaiStatus SetSyncAngleSyncHttp(int enable, int angle);

  /// @brief Getting lidar_monitor via HTTP API
  /// @param ctx IO Context
  /// @param str_callback Callback function for received string
  /// @return Resulting status
  HesaiStatus GetLidarMonitorAsyncHttp(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::function<void(const std::string & str)> str_callback);
  /// @brief Getting lidar_monitor via HTTP API
  /// @param str_callback Callback function for received string
  /// @return Resulting status
  HesaiStatus GetLidarMonitorAsyncHttp(std::function<void(const std::string & str)> str_callback);

  /// @brief Checking the current settings and changing the difference point
  /// @param sensor_configuration Current SensorConfiguration
  /// @param hesai_config Current HesaiConfig
  /// @return Resulting status
  HesaiStatus CheckAndSetConfig(
    std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration, HesaiConfig hesai_config);
  /// @brief Checking the current settings and changing the difference point
  /// @param sensor_configuration Current SensorConfiguration
  /// @param hesai_lidar_range_all Current HesaiLidarRangeAll
  /// @return Resulting status
  HesaiStatus CheckAndSetConfig(
    std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration,
    HesaiLidarRangeAll hesai_lidar_range_all);
  /// @brief Checking the current settings and changing the difference point
  /// @return Resulting status
  HesaiStatus CheckAndSetConfig();

  /// @brief Convert to model in Hesai protocol from nebula::drivers::SensorModel
  /// @param model
  /// @return
  int NebulaModelToHesaiModelNo(nebula::drivers::SensorModel model);

  /// @brief Set target model number (for proper use of HTTP and TCP according to the support of the
  /// target model)
  /// @param model Model number
  void SetTargetModel(int model);

  /// @brief Set target model number (for proper use of HTTP and TCP according to the support of the
  /// target model)
  /// @param model Model
  void SetTargetModel(nebula::drivers::SensorModel model);

  /// @brief Whether to use HTTP for setting SpinRate
  /// @param model Model number
  /// @return Use HTTP
  bool UseHttpSetSpinRate(int model);
  /// @brief Whether to use HTTP for setting SpinRate
  /// @return Use HTTP
  bool UseHttpSetSpinRate();
  /// @brief Whether to use HTTP for getting LidarMonitor
  /// @param model Model number
  /// @return Use HTTP
  bool UseHttpGetLidarMonitor(int model);
  /// @brief Whether to use HTTP for getting LidarMonitor
  /// @return Use HTTP
  bool UseHttpGetLidarMonitor();

  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> node);
};
}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_HW_INTERFACE_H
