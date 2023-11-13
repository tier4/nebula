#ifndef NEBULA_Innovusion_HW_INTERFACE_H
#define NEBULA_Innovusion_HW_INTERFACE_H
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
#include "nebula_common/innovusion/innovusion_common.hpp"
#include "nebula_common/innovusion/innovusion_status.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"
#include "boost_tcp_driver/http_client_driver.hpp"
#include "boost_tcp_driver/tcp_driver.hpp"
#include "boost_udp_driver/udp_driver.hpp"

#include <rclcpp/rclcpp.hpp>

#include "innovusion_msgs/msg/innovusion_packet.hpp"
#include "innovusion_msgs/msg/innovusion_scan.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <mutex>

namespace nebula
{
namespace drivers
{

const uint16_t kInnoMagicNumberDataPacket = 0x176A;
const uint16_t kInnoProtocolOldHeaderLen = 54;
const uint16_t kInnoProtocolNewHeaderLen = 70;
const uint16_t kInnoCommonHeaderLength = 26;
const uint16_t kInnoPktIdSection = 26;
const uint16_t kInnoPktIdLength = 8;
const uint16_t kInnoPktTypeIndex = 38;
const uint16_t kInnoPktSizeSectionIndex = 10;
const uint16_t kInnoPktSizeSectionLength = 4;
const uint16_t kInnoPktMajorVersionSection = 2;
const uint16_t kInnoPktMinorVersionSection = 3;
const uint16_t kInnoPktVersionSectionLen = 1;
const uint32_t kInnoProtocolMajorV1 = 1;
const uint32_t kInnoPktMax = 10000;

/// @brief Hardware interface of Innovusion driver
class InnovusionHwInterface : NebulaHwInterfaceBase
{
private:
  std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
  std::shared_ptr<boost::asio::io_context> m_owned_ctx;
  std::shared_ptr<boost::asio::io_context> m_owned_ctx_s;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> http_client_driver_;
  std::shared_ptr<InnovusionSensorConfiguration> sensor_configuration_;
  std::shared_ptr<InnovusionCalibrationConfiguration> calibration_configuration_;
  std::unique_ptr<innovusion_msgs::msg::InnovusionScan> scan_cloud_ptr_;
  std::function<void(std::unique_ptr<innovusion_msgs::msg::InnovusionScan> buffer)>
  scan_reception_callback_; /**This function pointer is called when the scan is complete*/
  std::uint64_t current_packet_id = 0;

  std::shared_ptr<rclcpp::Logger> parent_node_logger;

  /// @brief check packet vaild
  /// @param buffer Point Udp Data Buffer
  bool IsPacketValid(const std::vector<uint8_t> & buffer);
  /// @brief Printing the string to RCLCPP_INFO_STREAM
  /// @param buffer Point Udp Data Buffer
  void ProtocolCompatibility(std::vector<uint8_t> & buffer);
  /// @brief Printing the string to RCLCPP_INFO_STREAM
  /// @param info Target string
  void PrintInfo(std::string info);
  /// @brief Printing the string to RCLCPP_ERROR_STREAM
  /// @param error Target string
  void PrintError(std::string error);
  /// @brief Printing the string to RCLCPP_DEBUG_STREAM
  /// @param debug Target string
  void PrintDebug(std::string debug);

  /// @brief Get sensor configuration and status data
  /// @param key information key
  /// @return value of key
  std::string GetSensorParameter(const std::string & key);

  /// @brief Set sensor configuration and status data
  /// @param key information key
  std::string SetSensorParameter(const std::string & key, const std::string &value);

  /// @brief Init HttpClientDriver
  void InitHttpClientDriver();

  /// @brief Print sensor common information
  void DisplayCommonVersion();

  /// @brief check ip is broadcast
  bool IsBroadcast(std::string strIp);

  /// @brief check ip is multicast
  bool IsMulticast(std::string strIp);

  /// @brief add to multicast group
  /// @param strMuiticastIp remote Lidar send udp data ip
  /// @param uMulticastPort remote send udp data port
  /// @param strLocalIp local ip
  /// @param uLocalPort local port
  void AddToMuiticastGroup(const std::string &strMuiticastIp, const uint16_t &uMulticastPort,
     const std::string &strLocalIp, const uint16_t &uLocalPort);

  /// @brief update unicast ip port
  /// @param strUnicastIp remote Lidar send udp data ip
  /// @param uUnicastPort remote send udp data port
  /// @param strLocalIp local ip
  /// @param uLocalPort local port
  void UpdateUnicastIpPort(const std::string &strUnicastIp, const uint16_t &uUnicastPort,
     const std::string &strLocalIp, const uint16_t &uLocalPort);

public:
  /// @brief Constructor
  InnovusionHwInterface();
  /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) final;
  /// @brief Starting the interface that handles UDP streams
  /// @return Resulting status
  Status CloudInterfaceStart() final;
  /// @brief Function for stopping the interface that handles UDP streams
  /// @return Resulting status
  Status CloudInterfaceStop() final;
  /// @brief Setting sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status SetSensorConfiguration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration) final;
  /// @brief Printing sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration) final;
  /// @brief Printing calibration configuration
  /// @param calibration_configuration CalibrationConfiguration for the checking
  /// @return Resulting status
  Status GetCalibrationConfiguration(CalibrationConfigurationBase & calibration_configuration) final;
  /// @brief Registering callback for InnovusionScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterScanCallback(
    std::function<void(std::unique_ptr<innovusion_msgs::msg::InnovusionScan>)> scan_callback);
  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> node);
  /// @brief Get a one-off HTTP client to communicate with the hardware
  /// @param ctx IO Context
  /// @param hcd Got http client driver
  /// @return Resulting status
  Status GetHttpClientDriverOnce(
  std::shared_ptr<boost::asio::io_context> ctx,
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd);
  /// @brief Getting current sensor configuration and status data (async)
  /// @param str_callback Callback function for received JSON string
  /// @param snapshot snapshot information
  /// @return Resulting status
  Status GetSnapshotAsync(
  std::function<void(const std::string & str)> str_callback, const std::string & snapshot);
  /// @brief Parsing JSON string to property_tree
  /// @param str JSON string
  /// @return property_tree
  boost::property_tree::ptree ParseJson(const std::string & str);
};
}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_Innovusion_HW_INTERFACE_H
