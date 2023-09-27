#ifndef NEBULA_ROBOSENSE_HW_INTERFACE_H
#define NEBULA_ROBOSENSE_HW_INTERFACE_H
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
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_common/robosense/robosense_status.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_cmd_response.hpp"

#include <rclcpp/rclcpp.hpp>

#include "robosense_msgs/msg/robosense_packet.hpp"
#include "robosense_msgs/msg/robosense_scan.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <mutex>

namespace nebula
{
namespace drivers
{
const int RobosenseTcpHostPort = 24568;
const uint16_t HELIOS_PACKET_SIZE = 1248;
const uint16_t MEMS_MSOP_PACKET_SIZE = 1210;
const uint16_t MEMS_DIFOP_PACKET_SIZE = 256;
const uint16_t MTU_SIZE = 1500;

/// @brief Hardware interface of robosense driver
class RobosenseHwInterface : NebulaHwInterfaceBase
{
private:
  std::unique_ptr<::drivers::common::IoContext> msop_io_context_;
  std::unique_ptr<::drivers::common::IoContext> difop_io_context_;
  std::shared_ptr<boost::asio::io_context> m_owned_ctx;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> msop_udp_driver_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> difop_udp_driver_;
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> tcp_driver_;
  std::shared_ptr<RobosenseSensorConfiguration> sensor_configuration_;
  size_t azimuth_index_{0};
  size_t pkt_seq_index_{0};
  std::unique_ptr<robosense_msgs::msg::RobosenseScan> scan_cloud_ptr_;
  std::function<bool(size_t)>
    is_valid_packet_; /*Lambda Function Array to verify proper packet size*/
  std::function<bool(size_t)>
    is_valid_difop_packet_; /*Lambda Function Array to verify proper packet size*/
  std::function<void(std::unique_ptr<robosense_msgs::msg::RobosenseScan> buffer)>
    scan_reception_callback_; /**This function pointer is called when the scan is complete*/
  int prev_phase_{};
  int pre_pkt_seq{0};
  std::mutex scan_cloud_ptr_mtx_;
  RobosenseHeliosNetWorkParamkParam heliosConfigParameter_;
  RobosenseBpRubyNetWorkParamkParam bpRubyConfigParameter_;
  bool is_tcp_driver_init_{false};
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
  RobosenseHwInterface();
  /// @brief A callback that receives a string (just prints)
  /// @param str Received string
  void str_cb(const std::string & str);

  /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) final;
  /// @brief Callback function to receive the Difop Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void ReceiveDifopPacketCallback(const std::vector<uint8_t> & buffer);
  /// @brief Callback function to receive the Lidar config Packet data from the TCP Driver
  /// @param buffer Buffer containing the data received from the TCP socket
  void ReceiveTcpPacketCallback(const std::vector<uint8_t> & buffer);
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
  /// @brief Registering callback for RobosenseScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterScanCallback(
    std::function<void(std::unique_ptr<robosense_msgs::msg::RobosenseScan>)> scan_callback);

  /// @brief Sending command and Getting data with PTC_COMMAND_GET_CONFIG_INFO
  /// @param Send buffer
  /// @param with_run Automatically executes run() of TcpDriver
  /// @return Resulting status
  Status SendAndGetConfig(std::vector<unsigned char> buf_tcp_send_vec_, bool with_run = true);
  /// @brief Call run() of IO Context
  void IOContextRun();
  /// @brief GetIO Context
  /// @return IO Context
  std::shared_ptr<boost::asio::io_context> GetIOContext();

  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> node);

  /// @brief Getting Lidar Info
  Status GetLidarInfo();
  /// @brief Setting Lidar IP
  /// @param Lidar IP
  /// @return Resulting status
  Status SetLidarIp(const uint8_t lidar_ip[4]);
  /// @brief Setting Remote IP
  /// @param Remote IP
  /// @return Resulting status
  Status SetRemoteIp(const uint8_t remote_ip[4]);
  /// @brief Setting Netmask
  /// @param Netmask
  /// @return Resulting status
  Status SetNetMask(const uint8_t mask[4]);
  /// @brief Setting Gateway
  /// @param Gateway
  /// @return Resulting status
  Status SetNetGateWay(const uint8_t gateway[4]);
  /// @brief Setting Lidar Msop Port
  /// @param Msop Port
  /// @return Resulting status
  Status SetLidarMsopPort(const uint16_t & msop_port);
  /// @brief Setting Lidar Difop Port
  /// @param Difop Port
  /// @return Resulting status
  Status SetLidarDifopPort(const uint16_t & difop_port);
  /// @brief Setting Lidar Difop Port
  /// @param wave_mode{0:dual 1:strongest 2:last 3:first}
  /// @return Resulting status
  Status SetReturnMode(const uint16_t & wave_mode);
  /// @brief Setting Lidar Horizontal FOV Range
  /// @param fov_start(accurate to one hundredth)
  /// @param fov_end(accurate to one hundredth)
  /// @return Resulting status
  Status SetFovRange(const float & fov_start, const float & fov_end);
  /// @brief Setting Motor RPM
  /// @param rpm the RPM of the motor
  /// @return Resulting status
  Status SetRpmAsync(uint16_t rpm);
  /// @brief Setting Time Sync Mode
  /// @param time_sync_mode (0:SYNC_MODE_GPS 1:SYNC_MODE_PTP_E2E 2:SYNC_MODE_PTP_P2P
  /// 3:SYNC_MODE_PTP_GPTP  4:SYNC_MODE_PTP_E2E_L2)
  /// @return Resulting status
  Status SetTimeSyncMode(const uint16_t & time_sync_mode);
};
}  // namespace drivers
}  // namespace nebula

#endif  //  NEBULA_ROBOSENSE_HW_INTERFACE_H
