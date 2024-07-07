#ifndef NEBULA_SEYOND_HW_INTERFACE_H
#define NEBULA_SEYOND_HW_INTERFACE_H
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
#include "nebula_common/seyond/seyond_common.hpp"
#include "nebula_common/util/expected.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include "nebula_msgs/msg/nebula_packet.hpp"
#include "nebula_msgs/msg/nebula_packets.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <vector>

namespace nebula
{
namespace drivers
{
constexpr uint16_t SeyondTcpCommandPort = 8001;
constexpr uint16_t SeyondHttpCommandPort = 8010;
constexpr uint32_t kSeyondPktMax = 10000;
/// @brief Hardware interface of seyond driver
class SeyondHwInterface
{
private:
  std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::shared_ptr<const SeyondSensorConfiguration> sensor_configuration_;
  std::function<void(std::vector<uint8_t> & buffer)>
    cloud_packet_callback_; /**This function pointer is called when the scan is complete*/

  std::shared_ptr<boost::asio::io_context> m_owned_ctx_;
  std::unique_ptr<::drivers::tcp_driver::TcpDriver> command_tcp_driver_;

  std::shared_ptr<rclcpp::Logger> parent_node_logger_;

  /// @brief Printing the string to RCLCPP_INFO_STREAM
  /// @param info Target string
  void PrintInfo(std::string info);

  /// @brief Printing the string to RCLCPP_ERROR_STREAM
  /// @param error Target string
  void PrintError(std::string error);

  /// @brief Printing the string to RCLCPP_DEBUG_STREAM
  /// @param debug Target string
  void PrintDebug(std::string debug);

  /// @brief Send TCP command to the device and return its response as string
  /// @param commad payload of the command to be sent
  /// @return string response from the device
  std::string SendCommand(std::string commad);

  /// @brief Start UDP stream from the device
  Status StartUdpStreaming();

public:
  /// @brief Constructor
  SeyondHwInterface();

  /// @brief Destructor
  ~SeyondHwInterface();

  /// @brief Initializing tcp_driver for TCP communication
  /// @return Resulting status
  Status InitializeTcpDriver();

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
  Status GetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration);

  /// @brief Setting sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status SetSensorConfiguration(
    const std::shared_ptr<const SensorConfigurationBase>& sensor_configuration);

  // /// @brief Set target model number
  // /// @param model Model number
  // void SetTargetModel(nebula::drivers::SensorModel model);

  /// @brief Registering callback for NebulaPackets
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterScanCallback(std::function<void(std::vector<uint8_t> &)> scan_callback);

  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> logger);

  /// @brief Display common information acquired from sensor
  void DisplayCommonVersion();

  /// @brief Setting device return mode
  /// @param return_mode The mode of return
  /// @return Resulting status
  Status SetReturnMode(int return_mode);


  /// @brief Setting PTP profile
  /// @param profile profile to be set
  /// @return Resulting status
  Status SetPtpMode(PtpProfile profile);

  /// @brief validate the current settings then set them
  /// @return Resulting status
  Status CheckAndSetConfig();
};
}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_SEYOND_HW_INTERFACE_H
