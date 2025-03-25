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
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp"

#include <nebula_common/nebula_status.hpp>

#include <boost/version.hpp>

#include <cstddef>
#if (BOOST_VERSION / 100 >= 1073 && BOOST_VERSION / 100 <= 1076)  // Boost 1.73 - 1.76
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#endif
#if (BOOST_VERSION / 100 == 1074)  // Boost 1.74
#define BOOST_ALLOW_DEPRECATED_HEADERS
#endif
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp"

#include <boost_tcp_driver/http_client_driver.hpp>
#include <boost_tcp_driver/tcp_driver.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/hesai/hesai_status.hpp>
#include <nebula_common/loggers/logger.hpp>
#include <nebula_common/util/expected.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{
const int g_pandar_tcp_command_port = 9347;
const uint8_t g_ptc_command_dummy_byte = 0x00;
const uint8_t g_ptc_command_header_high = 0x47;
const uint8_t g_ptc_command_header_low = 0x74;
const uint8_t g_ptc_command_get_lidar_calibration = 0x05;
const uint8_t g_ptc_command_ptp_diagnostics = 0x06;
const uint8_t g_ptc_command_ptp_status = 0x01;
const uint8_t g_ptc_command_ptp_port_data_set = 0x02;
const uint8_t g_ptc_command_ptp_time_status_np = 0x03;
const uint8_t g_ptc_command_ptp_grandmaster_settings_np = 0x04;
const uint8_t g_ptc_command_get_inventory_info = 0x07;
const uint8_t g_ptc_command_get_config_info = 0x08;
const uint8_t g_ptc_command_get_lidar_status = 0x09;
const uint8_t g_ptc_command_set_spin_rate = 0x17;
const uint8_t g_ptc_command_set_sync_angle = 0x18;
const uint8_t g_ptc_command_set_trigger_method = 0x1b;
const uint8_t g_ptc_command_set_standby_mode = 0x1c;
const uint8_t g_ptc_command_set_return_mode = 0x1e;
const uint8_t g_ptc_command_set_clock_source = 0x1f;
const uint8_t g_ptc_command_set_destination_ip = 0x20;
const uint8_t g_ptc_command_set_control_port = 0x21;
const uint8_t g_ptc_command_set_lidar_range = 0x22;
const uint8_t g_ptc_command_get_lidar_range = 0x23;
const uint8_t g_ptc_command_set_ptp_config = 0x24;
const uint8_t g_ptc_command_get_ptp_config = 0x26;
const uint8_t g_ptc_command_set_high_resolution_mode = 0x29;
const uint8_t g_ptc_command_get_high_resolution_mode = 0x28;
const uint8_t g_ptp_command_set_ptp_lock_offset = 0x39;
const uint8_t g_ptp_command_get_ptp_lock_offset = 0x3a;
const uint8_t g_ptc_command_reset = 0x25;
const uint8_t g_ptc_command_set_rotate_direction = 0x2a;
const uint8_t g_ptc_command_lidar_monitor = 0x27;

const uint8_t g_ptc_error_code_no_error = 0x00;
const uint8_t g_ptc_error_code_invalid_input_param = 0x01;
const uint8_t g_ptc_error_code_server_conn_failed = 0x02;
const uint8_t g_ptc_error_code_invalid_data = 0x03;
const uint8_t g_ptc_error_code_out_of_memory = 0x04;
const uint8_t g_ptc_error_code_unsupported_cmd = 0x05;
const uint8_t g_ptc_error_code_fpga_comm_failed = 0x06;
const uint8_t g_ptc_error_code_other = 0x07;

const uint8_t g_tcp_error_unrelated_response = 1;
const uint8_t g_tcp_error_unexpected_payload = 2;
const uint8_t g_tcp_error_timeout = 4;
const uint8_t g_tcp_error_incomplete_response = 8;

const uint16_t g_mtu_size = 1500;

/// @brief The kernel buffer size in bytes to use for receiving UDP packets. If the buffer is too
/// small to bridge scheduling and processing delays, packets will be dropped. This corresponds to
/// the net.core.rmem_default setting in Linux. The current value is hardcoded to accommodate one
/// pointcloud worth of OT128 packets (currently the highest data rate sensor supported).
const size_t g_udp_socket_buffer_size = g_mtu_size * 3600;

// Time interval between Announce messages, in units of log seconds (default: 1)
const int g_ptp_log_announce_interval = 1;
// Time interval between Sync messages, in units of log seconds (default: 1)
const int g_ptp_sync_interval = 1;
// Minimum permitted mean time between Delay_Req messages, in units of log seconds (default: 0)
const int g_ptp_log_min_delay_interval = 0;

const int g_hesai_lidar_gps_clock_source = 0;
const int g_hesai_lidar_ptp_clock_source = 1;

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

  std::shared_ptr<loggers::Logger> logger_;
  std::optional<connections::UdpSocket> udp_socket_;
  std::shared_ptr<boost::asio::io_context> m_owned_ctx_;
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> tcp_driver_;
  std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration_;
  std::function<void(const std::vector<uint8_t> & buffer)>
    cloud_packet_callback_; /**This function pointer is called when the scan is complete*/

  std::mutex mtx_inflight_tcp_request_;

  int target_model_no_;

  /// @brief Get a one-off HTTP client to communicate with the hardware
  /// @param ctx IO Context
  /// @param hcd Got http client driver
  /// @return Resulting status
  HesaiStatus get_http_client_driver_once(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd);
  /// @brief Get a one-off HTTP client to communicate with the hardware (without specifying
  /// io_context)
  /// @param hcd Got http client driver
  /// @return Resulting status
  HesaiStatus get_http_client_driver_once(
    std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd);
  /// @brief A callback that receives a string (just prints)
  /// @param str Received string
  void str_cb(const std::string & str);

  /// @brief Convert an error code to a human-readable string
  /// @param error_code The error code, containing the sensor's error code (if any), along with
  /// flags such as TCP_ERROR_UNRELATED_RESPONSE etc.
  /// @return A string description of all errors in this code
  std::string pretty_print_ptc_error(ptc_error_t error_code);

  /// @brief Checks if the data size matches that of the struct to be parsed, and parses the struct.
  /// If data is too small, a std::runtime_error is thrown. If data is too large, a warning is
  /// printed and the struct is parsed with the first sizeof(T) bytes.
  template <typename T>
  T check_size_and_parse(const std::vector<uint8_t> & data);

  /// @brief Send a PTC request with an optional payload, and return the full response payload.
  /// Blocking.
  /// @param command_id PTC command number.
  /// @param payload Payload bytes of the PTC command. Not including the 8-byte PTC header.
  /// @return The returned payload, if successful, or nullptr.
  ptc_cmd_result_t send_receive(
    const uint8_t command_id, const std::vector<uint8_t> & payload = {});

  static std::pair<HesaiStatus, std::string> unwrap_http_response(const std::string & response);

public:
  /// @brief Constructor
  explicit HesaiHwInterface(const std::shared_ptr<loggers::Logger> & logger);
  /// @brief Destructor
  ~HesaiHwInterface();
  /// @brief Initializing tcp_driver for TCP communication
  /// @param setup_sensor Whether to also initialize tcp_driver for sensor configuration
  /// @return Resulting status
  Status initialize_tcp_driver();
  /// @brief Closes the TcpDriver and related resources
  /// @return Status result
  Status finalize_tcp_driver();
  /// @brief Parsing json string to property_tree
  /// @param str JSON string
  /// @return Parsed property_tree
  boost::property_tree::ptree parse_json(const std::string & str);

  /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void receive_sensor_packet_callback(const std::vector<uint8_t> & buffer);
  /// @brief Starting the interface that handles UDP streams
  /// @return Resulting status
  Status sensor_interface_start();
  /// @brief Function for stopping the interface that handles UDP streams
  /// @return Resulting status
  Status sensor_interface_stop();
  /// @brief Printing sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status get_sensor_configuration(const SensorConfigurationBase & sensor_configuration);
  /// @brief Printing calibration configuration
  /// @param calibration_configuration CalibrationConfiguration for the checking
  /// @return Resulting status
  Status get_calibration_configuration(CalibrationConfigurationBase & calibration_configuration);
  /// @brief Setting sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status set_sensor_configuration(
    std::shared_ptr<const SensorConfigurationBase> sensor_configuration);
  /// @brief Registering callback for PandarScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status register_scan_callback(std::function<void(const std::vector<uint8_t> &)> scan_callback);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @return Resulting status
  std::string get_lidar_calibration_string();
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @return Resulting status
  std::vector<uint8_t> get_lidar_calibration_bytes();
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP STATUS)
  /// @return Resulting status
  HesaiPtpDiagStatus get_ptp_diag_status();
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV PORT_DATA_SET)
  /// @return Resulting status
  HesaiPtpDiagPort get_ptp_diag_port();
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV TIME_STATUS_NP)
  /// @return Resulting status
  HesaiPtpDiagTime get_ptp_diag_time();
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV GRANDMASTER_SETTINGS_NP)
  /// @return Resulting status
  HesaiPtpDiagGrandmaster get_ptp_diag_grandmaster();
  /// @brief Getting data with PTC_COMMAND_GET_INVENTORY_INFO
  /// @return Resulting status
  std::shared_ptr<HesaiInventoryBase> get_inventory();
  /// @brief Getting data with PTC_COMMAND_GET_CONFIG_INFO
  /// @return Resulting status
  std::shared_ptr<HesaiConfigBase> get_config();
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_STATUS
  /// @return Resulting status
  std::shared_ptr<HesaiLidarStatusBase> get_lidar_status();
  /// @brief Setting value with PTC_COMMAND_SET_SPIN_RATE
  /// @param rpm Spin rate
  /// @return Resulting status
  Status set_spin_rate(uint16_t rpm);
  /// @brief Setting value with PTC_COMMAND_SET_SYNC_ANGLE
  /// @param sync_angle Sync angle enable flag
  /// @param angle Angle value
  /// @return Resulting status
  Status set_sync_angle(int sync_angle, int angle);
  /// @brief Setting mode with PTC_COMMAND_SET_TRIGGER_METHOD
  /// @param trigger_method Trigger method
  /// @return Resulting status
  Status set_trigger_method(int trigger_method);
  /// @brief Setting mode with PTC_COMMAND_SET_STANDBY_MODE
  /// @param standby_mode Standby mode
  /// @return Resulting status
  Status set_standby_mode(int standby_mode);
  /// @brief Setting mode with PTC_COMMAND_SET_RETURN_MODE
  /// @param return_mode Return mode
  /// @return Resulting status
  Status set_return_mode(int return_mode);
  /// @brief Setting IP with PTC_COMMAND_SET_DESTINATION_IP
  /// @param dest_ip_1 The 1st byte represents the 1st section
  /// @param dest_ip_2 The 2nd byte represents the 2nd section
  /// @param dest_ip_3 The 3rd byte represents the 3rd section
  /// @param dest_ip_4 The 4th byte represents the 4th section
  /// @param port LiDAR Destination Port
  /// @param gps_port GPS Destination Port
  /// @return Resulting status
  Status set_destination_ip(
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
  Status set_control_port(
    int ip_1, int ip_2, int ip_3, int ip_4, int mask_1, int mask_2, int mask_3, int mask_4,
    int gateway_1, int gateway_2, int gateway_3, int gateway_4, int vlan_flg, int vlan_id);
  /// @brief Setting values with PTC_COMMAND_SET_LIDAR_RANGE
  /// @param method Method
  /// @param data Set data
  /// @return Resulting status
  Status set_lidar_range(int method, std::vector<unsigned char> data);
  /// @brief Setting values with PTC_COMMAND_SET_LIDAR_RANGE
  /// @param start Start angle
  /// @param end End angle
  /// @return Resulting status
  Status set_lidar_range(int start, int end);
  /// @brief Getting values with PTC_COMMAND_GET_LIDAR_RANGE
  /// @return Resulting status
  HesaiLidarRangeAll get_lidar_range();
  /// @brief Setting values with PTC_COMMAND_SET_HIGH_RESOLUTION_MODE
  /// @return Resulting status
  Status set_high_resolution_mode(bool enable);
  /// @brief Getting values with PTC_COMMAND_GET_HIGH_RESOLUTION_MODE
  /// @return Resulting status
  bool get_high_resolution_mode();

  /**
   * @brief Given the HW interface's sensor configuration and a given calibration, set the sensor
   * FoV (min and max angles) with appropriate padding around the FoV set in the configuration. This
   * compensates for the points lost due to the sensor filtering FoV by raw encoder angle.
   *
   * @param calibration The calibration file of the sensor
   * @return Status Resulting status of setting the FoV
   */
  [[nodiscard]] Status check_and_set_lidar_range(
    const HesaiCalibrationConfigurationBase & calibration);

  Status set_clock_source(int clock_source);

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
  Status set_ptp_config(
    int profile, int domain, int network, int switch_type, int logAnnounceInterval = 1,
    int logSyncInterval = 1, int logMinDelayReqInterval = 0);
  /// @brief Getting data with PTC_COMMAND_GET_PTP_CONFIG
  /// @return Resulting status
  HesaiPtpConfig get_ptp_config();

  Status set_ptp_lock_offset(uint8_t lock_offset);

  uint8_t get_ptp_lock_offset();

  /// @brief Sending command with PTC_COMMAND_RESET
  /// @return Resulting status
  Status send_reset();
  /// @brief Setting values with PTC_COMMAND_SET_ROTATE_DIRECTION
  /// @param mode Rotation of the motor
  /// @return Resulting status
  Status set_rot_dir(int mode);
  /// @brief Getting data with PTC_COMMAND_LIDAR_MONITOR
  /// @return Resulting status
  HesaiLidarMonitor get_lidar_monitor();

  /// @brief Call run() of IO Context
  void io_context_run();
  /// @brief GetIO Context
  /// @return IO Context
  std::shared_ptr<boost::asio::io_context> get_io_context();

  /// @brief Setting spin_speed via HTTP API
  /// @param ctx IO Context used
  /// @param rpm spin_speed (300, 600, 1200)
  /// @return Resulting status
  HesaiStatus set_spin_speed_async_http(std::shared_ptr<boost::asio::io_context> ctx, uint16_t rpm);
  /// @brief Setting spin_speed via HTTP API
  /// @param rpm spin_speed (300, 600, 1200)
  /// @return Resulting status
  HesaiStatus set_spin_speed_async_http(uint16_t rpm);

  HesaiStatus set_ptp_config_sync_http(
    std::shared_ptr<boost::asio::io_context> ctx, int profile, int domain, int network,
    int logAnnounceInterval, int logSyncInterval, int logMinDelayReqInterval);
  HesaiStatus set_ptp_config_sync_http(
    int profile, int domain, int network, int logAnnounceInterval, int logSyncInterval,
    int logMinDelayReqInterval);
  HesaiStatus set_sync_angle_sync_http(
    std::shared_ptr<boost::asio::io_context> ctx, int enable, int angle);
  HesaiStatus set_sync_angle_sync_http(int enable, int angle);

  /// @brief Getting lidar_monitor via HTTP API
  /// @param ctx IO Context
  /// @param str_callback Callback function for received string
  /// @return Resulting status
  HesaiStatus get_lidar_monitor_async_http(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::function<void(const std::string & str)> str_callback);
  /// @brief Getting lidar_monitor via HTTP API
  /// @param str_callback Callback function for received string
  /// @return Resulting status
  HesaiStatus get_lidar_monitor_async_http(
    std::function<void(const std::string & str)> str_callback);

  /// @brief Checking the current settings and changing the difference point
  /// @param sensor_configuration Current SensorConfiguration
  /// @param hesai_config Current HesaiConfig
  /// @return Resulting status
  HesaiStatus check_and_set_config(
    std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration,
    std::shared_ptr<HesaiConfigBase> hesai_config);
  /// @brief Checking the current settings and changing the difference point
  /// @param sensor_configuration Current SensorConfiguration
  /// @param hesai_lidar_range_all Current HesaiLidarRangeAll
  /// @return Resulting status
  HesaiStatus check_and_set_config(
    std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration,
    HesaiLidarRangeAll hesai_lidar_range_all);
  /// @brief Checking the current settings and changing the difference point
  /// @return Resulting status
  HesaiStatus check_and_set_config();

  /// @brief Convert to model in Hesai protocol from nebula::drivers::SensorModel
  /// @param model
  /// @return
  int nebula_model_to_hesai_model_no(nebula::drivers::SensorModel model);

  /// @brief Set target model number (for proper use of HTTP and TCP according to the support of the
  /// target model)
  /// @param model Model number
  void set_target_model(int model);

  /// @brief Set target model number (for proper use of HTTP and TCP according to the support of the
  /// target model)
  /// @param model Model
  void set_target_model(nebula::drivers::SensorModel model);

  /// @brief Whether to use HTTP for setting SpinRate
  /// @param model Model number
  /// @return Use HTTP
  bool use_http_set_spin_rate(int model);
  /// @brief Whether to use HTTP for setting SpinRate
  /// @return Use HTTP
  bool use_http_set_spin_rate();
  /// @brief Whether to use HTTP for getting LidarMonitor
  /// @param model Model number
  /// @return Use HTTP
  bool use_http_get_lidar_monitor(int model);
  /// @brief Whether to use HTTP for getting LidarMonitor
  /// @return Use HTTP
  bool use_http_get_lidar_monitor();
};
}  // namespace nebula::drivers

#endif  // NEBULA_HESAI_HW_INTERFACE_H
