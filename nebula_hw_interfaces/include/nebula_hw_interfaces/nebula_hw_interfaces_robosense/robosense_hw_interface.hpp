#pragma once

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

#include <boost_udp_driver/udp_driver.hpp>
#include <nebula_common/robosense/robosense_common.hpp>

#include <rclcpp/rclcpp.hpp>

namespace nebula
{
namespace drivers
{
constexpr uint16_t MTU_SIZE = 1248;
constexpr uint16_t HELIOS_PACKET_SIZE = 1248;
constexpr uint16_t HELIOS_INFO_PACKET_SIZE = 1248;
constexpr uint16_t BPEARL_PACKET_SIZE = 1248;
constexpr uint16_t BPEARL_INFO_PACKET_SIZE = 1248;

/// @brief Hardware interface of Robosense driver
class RobosenseHwInterface
{
private:
  std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
  std::unique_ptr<::drivers::common::IoContext> info_io_context_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> info_udp_driver_;
  std::shared_ptr<const RobosenseSensorConfiguration> sensor_configuration_;
  std::function<void(std::vector<uint8_t> & buffer)>
    scan_reception_callback_; /**This function pointer is called when the scan is complete*/
  std::function<void(std::vector<uint8_t> & buffer)>
    info_reception_callback_; /**This function pointer is called when DIFOP packet is received*/
  std::shared_ptr<rclcpp::Logger> parent_node_logger_;

  /// @brief Printing the string to RCLCPP_INFO_STREAM
  /// @param info Target string
  void PrintInfo(std::string info);

  /// @brief Printing the string to RCLCPP_DEBUG_STREAM
  /// @param debug Target string
  void PrintDebug(std::string debug);

public:
  /// @brief Constructor
  RobosenseHwInterface();

  /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void ReceiveSensorPacketCallback(std::vector<uint8_t> & buffer);

  /// @brief Callback function to receive the Info Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void ReceiveInfoPacketCallback(std::vector<uint8_t> & buffer);

  /// @brief Starting the interface that handles UDP streams for MSOP packets
  /// @return Resulting status
  Status SensorInterfaceStart();

  /// @brief Starting the interface that handles UDP streams for DIFOP packets
  /// @return Resulting status
  Status InfoInterfaceStart();

  /// @brief Setting sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status SetSensorConfiguration(
    std::shared_ptr<const RobosenseSensorConfiguration> sensor_configuration);

  /// @brief Registering callback for RobosenseScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterScanCallback(
    std::function<void(std::vector<uint8_t> &)> scan_callback);

  /// @brief Registering callback for RobosensePacket
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterInfoCallback(
    std::function<void(std::vector<uint8_t> &)> info_callback);

  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> logger);
};

}  // namespace drivers
}  // namespace nebula
