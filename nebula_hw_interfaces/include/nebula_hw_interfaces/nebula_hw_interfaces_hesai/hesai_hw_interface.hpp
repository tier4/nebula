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
#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/hesai/hesai_status.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp"
#include "boost_tcp_driver/http_client_driver.hpp"
#include "boost_tcp_driver/tcp_driver.hpp"
#include "boost_udp_driver/udp_driver.hpp"

#include <rclcpp/rclcpp.hpp>

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <mutex>

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

const int PTP_PROFILE = 0; // Fixed: IEEE 1588v2
const int PTP_DOMAIN_ID = 0; // 0-127, Default: 0
const int PTP_NETWORK_TRANSPORT = 0; // 0: UDP/IP, 1: L2
const int PTP_LOG_ANNOUNCE_INTERVAL = 1; // Time interval between Announce messages, in units of log seconds (default: 1)
const int PTP_SYNC_INTERVAL = 1; //Time interval between Sync messages, in units of log seconds (default: 1)
const int PTP_LOG_MIN_DELAY_INTERVAL = 0; //Minimum permitted mean time between Delay_Req messages, in units of log seconds (default: 0)

const int HESAI_LIDAR_GPS_CLOCK_SOURCE = 0;
const int HESAI_LIDAR_PTP_CLOCK_SOURCE = 1;

/// @brief Hardware interface of hesai driver
class HesaiHwInterface : NebulaHwInterfaceBase
{
private:
  std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
  std::shared_ptr<boost::asio::io_context> m_owned_ctx;
  std::shared_ptr<boost::asio::io_context> m_owned_ctx_s;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> tcp_driver_;
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> tcp_driver_s_;
  std::shared_ptr<HesaiSensorConfiguration> sensor_configuration_;
  std::shared_ptr<HesaiCalibrationConfiguration> calibration_configuration_;
  size_t azimuth_index_{};
  size_t mtu_size_{};
  std::unique_ptr<pandar_msgs::msg::PandarScan> scan_cloud_ptr_;
  std::function<bool(size_t)>
    is_valid_packet_; /*Lambda Function Array to verify proper packet size*/
  std::function<void(std::unique_ptr<pandar_msgs::msg::PandarScan> buffer)>
    scan_reception_callback_; /**This function pointer is called when the scan is complete*/

  int prev_phase_{};

  int timeout_ = 2000;
  std::timed_mutex tm_;
  int tm_fail_cnt = 0;
  int tm_fail_cnt_max = 0;
  std::timed_mutex tms_;
  int tms_fail_cnt = 0;
  int tms_fail_cnt_max = 3;
  bool wl = true;
  bool is_solid_state = false;
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

  /// @brief Lock function during TCP communication
  /// @param tm Mutex
  /// @param fail_cnt # of failures
  /// @param fail_cnt_max # of times to accept failure
  /// @param name Confirmation name used in PrintDebug
  /// @return Locked
  bool CheckLock(std::timed_mutex & tm, int & fail_cnt, const int & fail_cnt_max, std::string name);
  /// @brief Unlock function during TCP communication
  /// @param tm Mutex
  /// @param name Confirmation name used in PrintDebug
  void CheckUnlock(std::timed_mutex & tm, std::string name);

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
  HesaiHwInterface();
  /// @brief Initializing tcp_driver for TCP communication
  /// @param setup_sensor Whether to also initialize tcp_driver for sensor configuration
  /// @return Resulting status
  Status InitializeTcpDriver(bool setup_sensor = true);
  /// @brief Closes the TcpDriver and related resources
  /// @return Status result
  Status FinalizeTcpDriver();
  /// @brief Parsing json string to property_tree
  /// @param str JSON string
  /// @return Parsed property_tree
  boost::property_tree::ptree ParseJson(const std::string & str);

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
  /// @brief Printing calibration configuration
  /// @param calibration_configuration CalibrationConfiguration for the checking
  /// @return Resulting status
  Status GetCalibrationConfiguration(CalibrationConfigurationBase & calibration_configuration);
  /// @brief Setting sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status SetSensorConfiguration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration) final;
  /// @brief Registering callback for PandarScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterScanCallback(
    std::function<void(std::unique_ptr<pandar_msgs::msg::PandarScan>)> scan_callback);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION (sync)
  /// @param target_tcp_driver TcpDriver used
  /// @return Resulting status
  Status syncGetLidarCalibration(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
    std::function<void(const std::vector<uint8_t> & received_bytes)> bytes_callback);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION (sync)
  /// @param bytes_callback callback
  /// @param ctx IO Context used
  /// @return Resulting status
  Status syncGetLidarCalibration(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
    std::function<void(const std::string & str)> str_callback);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION (sync)
  /// @param str_callback callback
  /// @param ctx IO Context used
  /// @return Resulting status
  Status syncGetLidarCalibration(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION (sync)
  /// @param ctx IO Context used
  /// @return Resulting status
  Status syncGetLidarCalibration(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::function<void(const std::string & str)> str_callback);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION (sync)
  /// @param str_callback callback
  /// @return Resulting status
  Status syncGetLidarCalibration(std::shared_ptr<boost::asio::io_context> ctx);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION (sync)
  /// @return Resulting status
  Status syncGetLidarCalibrationFromSensor(
    std::function<void(const std::vector<uint8_t> & received_bytes)> bytes_callback);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION (sync)
  /// @param target_tcp_driver TcpDriver used
  /// @param bytes_callback callback
  /// @return Resulting status
  Status syncGetLidarCalibrationFromSensor(
    std::function<void(const std::string & str)> str_callback);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION (sync)
  /// @param target_tcp_driver TcpDriver used
  /// @param str_callback callback
  /// @return Resulting status
  Status syncGetLidarCalibrationFromSensor();
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @param target_tcp_driver TcpDriver used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarCalibration(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
    std::function<void(const std::vector<uint8_t> & received_bytes)> bytes_callback,
    bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @param bytes_callback callback
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarCalibration(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
    std::function<void(const std::string & str)> str_callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @param str_callback callback
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarCalibration(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarCalibration(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::function<void(const std::string & str)> str_callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @param str_callback callback
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarCalibration(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarCalibrationFromSensor(
    std::function<void(const std::vector<uint8_t> & received_bytes)> bytes_callback,
    bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @param target_tcp_driver TcpDriver used
  /// @param bytes_callback callback
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarCalibrationFromSensor(
    std::function<void(const std::string & str)> str_callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_CALIBRATION
  /// @param target_tcp_driver TcpDriver used
  /// @param str_callback callback
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarCalibrationFromSensor(bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP STATUS)
  /// @param target_tcp_driver TcpDriver used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagStatus(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP STATUS)
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagStatus(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP STATUS)
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagStatus(bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV PORT_DATA_SET)
  /// @param target_tcp_driver TcpDriver used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagPort(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV PORT_DATA_SET)
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagPort(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV PORT_DATA_SET)
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagPort(bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV TIME_STATUS_NP)
  /// @param target_tcp_driver TcpDriver used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagTime(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV TIME_STATUS_NP)
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagTime(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV TIME_STATUS_NP)
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagTime(bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV GRANDMASTER_SETTINGS_NP)
  /// @param target_tcp_driver TcpDriver used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagGrandmaster(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV GRANDMASTER_SETTINGS_NP)
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagGrandmaster(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_PTP_DIAGNOSTICS (PTP TLV GRANDMASTER_SETTINGS_NP)
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpDiagGrandmaster(bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_INVENTORY_INFO
  /// @param target_tcp_driver TcpDriver used
  /// @param callback Callback function for received HesaiInventory
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetInventory(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
    std::function<void(HesaiInventory & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_INVENTORY_INFO
  /// @param ctx IO Context used
  /// @param callback Callback function for received HesaiInventory
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetInventory(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::function<void(HesaiInventory & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_INVENTORY_INFO
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetInventory(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_INVENTORY_INFO
  /// @param callback Callback function for received HesaiInventory
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetInventory(std::function<void(HesaiInventory & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_INVENTORY_INFO
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetInventory(bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_CONFIG_INFO
  /// @param target_tcp_driver TcpDriver used
  /// @param callback Callback function for received HesaiConfig
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetConfig(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
    std::function<void(HesaiConfig & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_CONFIG_INFO
  /// @param ctx IO Context used
  /// @param callback Callback function for received HesaiConfig
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetConfig(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::function<void(HesaiConfig & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_CONFIG_INFO
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetConfig(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_CONFIG_INFO
  /// @param callback Callback function for received HesaiConfig
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetConfig(std::function<void(HesaiConfig & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_CONFIG_INFO
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetConfig(bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_STATUS
  /// @param target_tcp_driver TcpDriver used
  /// @param callback Callback function for received HesaiLidarStatus
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarStatus(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
    std::function<void(HesaiLidarStatus & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_STATUS
  /// @param ctx IO Context used
  /// @param callback Callback function for received HesaiLidarStatus
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarStatus(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::function<void(HesaiLidarStatus & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_STATUS
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarStatus(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_STATUS
  /// @param callback Callback function for received HesaiLidarStatus
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarStatus(
    std::function<void(HesaiLidarStatus & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_LIDAR_STATUS
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarStatus(bool with_run = true);
  /// @brief Setting value with PTC_COMMAND_SET_SPIN_RATE
  /// @param target_tcp_driver TcpDriver used
  /// @param rpm Spin rate
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetSpinRate(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, uint16_t rpm,
    bool with_run = true);
  /// @brief Setting value with PTC_COMMAND_SET_SPIN_RATE
  /// @param ctx IO Context used
  /// @param rpm Spin rate
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetSpinRate(
    std::shared_ptr<boost::asio::io_context> ctx, uint16_t rpm, bool with_run = true);
  /// @brief Setting value with PTC_COMMAND_SET_SPIN_RATE
  /// @param rpm Spin rate
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetSpinRate(uint16_t rpm, bool with_run = true);
  /// @brief Setting value with PTC_COMMAND_SET_SYNC_ANGLE
  /// @param target_tcp_driver TcpDriver used
  /// @param sync_angle Sync angle enable flag
  /// @param angle Angle value
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetSyncAngle(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int sync_angle, int angle,
    bool with_run = true);
  /// @brief Setting value with PTC_COMMAND_SET_SYNC_ANGLE
  /// @param ctx IO Context used
  /// @param sync_angle Sync angle enable flag
  /// @param angle Angle value
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetSyncAngle(
    std::shared_ptr<boost::asio::io_context> ctx, int sync_angle, int angle, bool with_run = true);
  /// @brief Setting value with PTC_COMMAND_SET_SYNC_ANGLE
  /// @param sync_angle Sync angle enable flag
  /// @param angle Angle value
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetSyncAngle(int sync_angle, int angle, bool with_run = true);
  /// @brief Setting mode with PTC_COMMAND_SET_TRIGGER_METHOD
  /// @param target_tcp_driver TcpDriver used
  /// @param trigger_method Trigger method
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetTriggerMethod(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int trigger_method,
    bool with_run = true);
  /// @brief Setting mode with PTC_COMMAND_SET_TRIGGER_METHOD
  /// @param ctx IO Context used
  /// @param trigger_method Trigger method
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetTriggerMethod(
    std::shared_ptr<boost::asio::io_context> ctx, int trigger_method, bool with_run = true);
  /// @brief Setting mode with PTC_COMMAND_SET_TRIGGER_METHOD
  /// @param trigger_method Trigger method
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetTriggerMethod(int trigger_method, bool with_run = true);
  /// @brief Setting mode with PTC_COMMAND_SET_STANDBY_MODE
  /// @param target_tcp_driver TcpDriver used
  /// @param standby_mode Standby mode
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetStandbyMode(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int standby_mode,
    bool with_run = true);
  /// @brief Setting mode with PTC_COMMAND_SET_STANDBY_MODE
  /// @param ctx IO Context used
  /// @param standby_mode Standby mode
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetStandbyMode(
    std::shared_ptr<boost::asio::io_context> ctx, int standby_mode, bool with_run = true);
  /// @brief Setting mode with PTC_COMMAND_SET_STANDBY_MODE
  /// @param standby_mode Standby mode
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetStandbyMode(int standby_mode, bool with_run = true);
  /// @brief Setting mode with PTC_COMMAND_SET_RETURN_MODE
  /// @param target_tcp_driver TcpDriver used
  /// @param return_mode Return mode
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetReturnMode(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int return_mode,
    bool with_run = true);
  /// @brief Setting mode with PTC_COMMAND_SET_RETURN_MODE
  /// @param ctx IO Context used
  /// @param return_mode Return mode
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetReturnMode(
    std::shared_ptr<boost::asio::io_context> ctx, int return_mode, bool with_run = true);
  /// @brief Setting mode with PTC_COMMAND_SET_RETURN_MODE
  /// @param return_mode Return mode
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetReturnMode(int return_mode, bool with_run = true);
  /// @brief Setting IP with PTC_COMMAND_SET_DESTINATION_IP
  /// @param target_tcp_driver TcpDriver used
  /// @param dest_ip_1 The 1st byte represents the 1st section
  /// @param dest_ip_2 The 2nd byte represents the 2nd section
  /// @param dest_ip_3 The 3rd byte represents the 3rd section
  /// @param dest_ip_4 The 4th byte represents the 4th section
  /// @param port LiDAR Destination Port
  /// @param gps_port GPS Destination Port
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetDestinationIp(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int dest_ip_1,
    int dest_ip_2, int dest_ip_3, int dest_ip_4, int port, int gps_port, bool with_run = true);
  /// @brief Setting IP with PTC_COMMAND_SET_DESTINATION_IP
  /// @param ctx IO Context used
  /// @param dest_ip_1 The 1st byte represents the 1st section
  /// @param dest_ip_2 The 2nd byte represents the 2nd section
  /// @param dest_ip_3 The 3rd byte represents the 3rd section
  /// @param dest_ip_4 The 4th byte represents the 4th section
  /// @param port LiDAR Destination Port
  /// @param gps_port GPS Destination Port
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetDestinationIp(
    std::shared_ptr<boost::asio::io_context> ctx, int dest_ip_1, int dest_ip_2, int dest_ip_3,
    int dest_ip_4, int port, int gps_port, bool with_run = true);
  /// @brief Setting IP with PTC_COMMAND_SET_DESTINATION_IP
  /// @param dest_ip_1 The 1st byte represents the 1st section
  /// @param dest_ip_2 The 2nd byte represents the 2nd section
  /// @param dest_ip_3 The 3rd byte represents the 3rd section
  /// @param dest_ip_4 The 4th byte represents the 4th section
  /// @param port LiDAR Destination Port
  /// @param gps_port GPS Destination Port
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetDestinationIp(
    int dest_ip_1, int dest_ip_2, int dest_ip_3, int dest_ip_4, int port, int gps_port,
    bool with_run = true);
  /// @brief Setting IP with PTC_COMMAND_SET_CONTROL_PORT
  /// @param target_tcp_driver TcpDriver used
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
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetControlPort(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int ip_1, int ip_2,
    int ip_3, int ip_4, int mask_1, int mask_2, int mask_3, int mask_4, int gateway_1,
    int gateway_2, int gateway_3, int gateway_4, int vlan_flg, int vlan_id, bool with_run = true);
  /// @brief Setting IP with PTC_COMMAND_SET_CONTROL_PORT
  /// @param ctx IO Context used
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
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetControlPort(
    std::shared_ptr<boost::asio::io_context> ctx, int ip_1, int ip_2, int ip_3, int ip_4,
    int mask_1, int mask_2, int mask_3, int mask_4, int gateway_1, int gateway_2, int gateway_3,
    int gateway_4, int vlan_flg, int vlan_id, bool with_run = true);
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
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetControlPort(
    int ip_1, int ip_2, int ip_3, int ip_4, int mask_1, int mask_2, int mask_3, int mask_4,
    int gateway_1, int gateway_2, int gateway_3, int gateway_4, int vlan_flg, int vlan_id,
    bool with_run = true);
  /// @brief Setting values with PTC_COMMAND_SET_LIDAR_RANGE
  /// @param target_tcp_driver TcpDriver used
  /// @param method Method
  /// @param data Set data
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetLidarRange(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int method,
    std::vector<unsigned char> data, bool with_run = true);
  /// @brief Setting values with PTC_COMMAND_SET_LIDAR_RANGE
  /// @param ctx IO Context used
  /// @param method Method
  /// @param data Set data
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetLidarRange(
    std::shared_ptr<boost::asio::io_context> ctx, int method, std::vector<unsigned char> data,
    bool with_run = true);
  /// @brief Setting values with PTC_COMMAND_SET_LIDAR_RANGE
  /// @param method Method
  /// @param data Set data
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetLidarRange(int method, std::vector<unsigned char> data, bool with_run = true);
  /// @brief Setting values with PTC_COMMAND_SET_LIDAR_RANGE
  /// @param target_tcp_driver TcpDriver used
  /// @param start Start angle
  /// @param end End angle
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetLidarRange(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int start, int end,
    bool with_run = true);
  /// @brief Setting values with PTC_COMMAND_SET_LIDAR_RANGE
  /// @param ctx IO Context used
  /// @param start Start angle
  /// @param end End angle
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetLidarRange(
    std::shared_ptr<boost::asio::io_context> ctx, int start, int end, bool with_run = true);
  /// @brief Setting values with PTC_COMMAND_SET_LIDAR_RANGE
  /// @param start Start angle
  /// @param end End angle
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetLidarRange(int start, int end, bool with_run = true);
  /// @brief Getting values with PTC_COMMAND_GET_LIDAR_RANGE
  /// @param target_tcp_driver TcpDriver used
  /// @param callback Callback function for received HesaiLidarRangeAll
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarRange(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
    std::function<void(HesaiLidarRangeAll & result)> callback, bool with_run = true);
  /// @brief Getting values with PTC_COMMAND_GET_LIDAR_RANGE
  /// @param ctx IO Context used
  /// @param callback Callback function for received HesaiLidarRangeAll
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarRange(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::function<void(HesaiLidarRangeAll & result)> callback, bool with_run = true);
  /// @brief Getting values with PTC_COMMAND_GET_LIDAR_RANGE
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarRange(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  /// @brief Getting values with PTC_COMMAND_GET_LIDAR_RANGE
  /// @param callback Callback function for received HesaiLidarRangeAll
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarRange(
    std::function<void(HesaiLidarRangeAll & result)> callback, bool with_run = true);
  /// @brief Getting values with PTC_COMMAND_GET_LIDAR_RANGE
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarRange(bool with_run = true);

  Status SetClockSource(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int clock_source, bool with_run);
  Status SetClockSource(std::shared_ptr<boost::asio::io_context> ctx, int clock_source, bool with_run);
  Status SetClockSource(int clock_source, bool with_run = true);

  /// @brief Setting values with PTC_COMMAND_SET_PTP_CONFIG
  /// @param target_tcp_driver TcpDriver used
  /// @param profile IEEE timing and synchronization standard
  /// @param domain Domain attribute of the local clock
  /// @param network Network transport type of 1588v2
  /// @param logAnnounceInterval Time interval between Announce messages, in units of log seconds
  /// (default: 1)
  /// @param logSyncInterval Time interval between Sync messages, in units of log seconds (default:
  /// 1)
  /// @param logMinDelayReqInterval Minimum permitted mean time between Delay_Req messages, in units
  /// of log seconds (default: 0)
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetPtpConfig(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int profile, int domain,
    int network, int logAnnounceInterval, int logSyncInterval, int logMinDelayReqInterval,
    bool with_run = true);
  /// @brief Setting values with PTC_COMMAND_SET_PTP_CONFIG
  /// @param ctx IO Context used
  /// @param profile IEEE timing and synchronization standard
  /// @param domain Domain attribute of the local clock
  /// @param network Network transport type of 1588v2
  /// @param logAnnounceInterval Time interval between Announce messages, in units of log seconds
  /// (default: 1)
  /// @param logSyncInterval Time interval between Sync messages, in units of log seconds (default:
  /// 1)
  /// @param logMinDelayReqInterval Minimum permitted mean time between Delay_Req messages, in units
  /// of log seconds (default: 0)
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetPtpConfig(
    std::shared_ptr<boost::asio::io_context> ctx, int profile, int domain, int network,
    int logAnnounceInterval, int logSyncInterval, int logMinDelayReqInterval, bool with_run = true);
  /// @brief Setting values with PTC_COMMAND_SET_PTP_CONFIG
  /// @param profile IEEE timing and synchronization standard
  /// @param domain Domain attribute of the local clock
  /// @param network Network transport type of 1588v2
  /// @param logAnnounceInterval Time interval between Announce messages, in units of log seconds
  /// (default: 1)
  /// @param logSyncInterval Time interval between Sync messages, in units of log seconds (default:
  /// 1)
  /// @param logMinDelayReqInterval Minimum permitted mean time between Delay_Req messages, in units
  /// of log seconds (default: 0)
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetPtpConfig(
    int profile, int domain, int network, int logAnnounceInterval, int logSyncInterval,
    int logMinDelayReqInterval, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_PTP_CONFIG
  /// @param target_tcp_driver TcpDriver used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpConfig(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_PTP_CONFIG
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpConfig(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_GET_PTP_CONFIG
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetPtpConfig(bool with_run = true);
  /// @brief Sending command with PTC_COMMAND_RESET
  /// @param target_tcp_driver TcpDriver used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SendReset(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run = true);
  /// @brief Sending command with PTC_COMMAND_RESET
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SendReset(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  /// @brief Sending command with PTC_COMMAND_RESET
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SendReset(bool with_run = true);
  /// @brief Setting values with PTC_COMMAND_SET_ROTATE_DIRECTION
  /// @param target_tcp_driver TcpDriver used
  /// @param mode Rotation of the motor
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetRotDir(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int mode,
    bool with_run = true);
  /// @brief Setting values with PTC_COMMAND_SET_ROTATE_DIRECTION
  /// @param ctx IO Context used
  /// @param mode Rotation of the motor
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetRotDir(std::shared_ptr<boost::asio::io_context> ctx, int mode, bool with_run = true);
  /// @brief Setting values with PTC_COMMAND_SET_ROTATE_DIRECTION
  /// @param mode Rotation of the motor
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SetRotDir(int mode, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_LIDAR_MONITOR
  /// @param target_tcp_driver TcpDriver used
  /// @param callback Callback function for received HesaiLidarMonitor
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarMonitor(
    std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
    std::function<void(HesaiLidarMonitor & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_LIDAR_MONITOR
  /// @param ctx IO Context used
  /// @param callback Callback function for received HesaiLidarMonitor
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarMonitor(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::function<void(HesaiLidarMonitor & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_LIDAR_MONITOR
  /// @param ctx IO Context used
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarMonitor(std::shared_ptr<boost::asio::io_context> ctx, bool with_run = true);
  Status GetLidarMonitor(
    std::function<void(HesaiLidarMonitor & result)> callback, bool with_run = true);
  /// @brief Getting data with PTC_COMMAND_LIDAR_MONITOR
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status GetLidarMonitor(bool with_run = true);

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
    std::shared_ptr<boost::asio::io_context> ctx,
    int profile,
    int domain,
    int network,
    int logAnnounceInterval,
    int logSyncInterval,
    int logMinDelayReqInterval);
  HesaiStatus SetPtpConfigSyncHttp(int profile,
                                   int domain,
                                   int network,
                                   int logAnnounceInterval,
                                   int logSyncInterval,
                                   int logMinDelayReqInterval);
  HesaiStatus SetSyncAngleSyncHttp(
    std::shared_ptr<boost::asio::io_context> ctx,
    int enable,
    int angle);
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
    std::shared_ptr<HesaiSensorConfiguration> sensor_configuration, HesaiConfig hesai_config);
  /// @brief Checking the current settings and changing the difference point
  /// @param sensor_configuration Current SensorConfiguration
  /// @param hesai_lidar_range_all Current HesaiLidarRangeAll
  /// @return Resulting status
  HesaiStatus CheckAndSetConfig(
    std::shared_ptr<HesaiSensorConfiguration> sensor_configuration,
    HesaiLidarRangeAll hesai_lidar_range_all);
  /// @brief Checking the current settings and changing the difference point
  /// @return Resulting status
  HesaiStatus CheckAndSetConfig();

  /// @brief Set target model number (for proper use of HTTP and TCP according to the support of the
  /// target model)
  /// @param model Model number
  void SetTargetModel(int model);

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
