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

#include "boost_udp_driver/udp_driver.hpp"
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include "robosense_msgs/msg/robosense_info_packet.hpp"
#include "robosense_msgs/msg/robosense_packet.hpp"
#include "robosense_msgs/msg/robosense_scan.hpp"

namespace nebula
{
namespace drivers
{

const uint16_t MTU_SIZE = 1248;
const uint16_t HELIOS5515_PACKET_SIZE = 1248;
const uint16_t HELIOS5515_INFO_PACKET_SIZE = 1248;
const uint16_t HELIOS5515_CORRECTED_VERTICAL_ANGLE_OFFSET = 468;
const uint16_t HELIOS5515_CORRECTED_HORIZONTAL_ANGLE_OFFSET = 564;
const uint16_t HELIOS5515_RETURN_MODE_OFFSET = 300;
const uint16_t BPEARL_PACKET_SIZE = 1248;
const uint16_t BPEARL_INFO_PACKET_SIZE = 1248;
const uint16_t BPEARL_CORRECTED_VERTICAL_ANGLE_OFFSET = 468;
const uint16_t BPEARL_CORRECTED_HORIZONTAL_ANGLE_OFFSET = 564;
const uint16_t BPEARL_RETURN_MODE_OFFSET = 300;

/// @brief Hardware interface of Robosense driver
class RobosenseHwInterface : NebulaHwInterfaceBase
{
private:
  std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
  std::unique_ptr<::drivers::common::IoContext> info_io_context_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> info_udp_driver_;
  std::shared_ptr<RobosenseSensorConfiguration> sensor_configuration_;
  std::unique_ptr<robosense_msgs::msg::RobosenseScan> scan_cloud_ptr_;
  size_t azimuth_index_{44};  // For Helios and Bpearl 42 byte header + 2 byte flag
  int prev_phase_{};
  std::atomic<bool> is_info_received{false};         // To check if DIFOP is received
  std::optional<std::vector<uint8_t>> info_buffer_;  // To hold DIFOP data
  std::optional<SensorModel> sensor_model_;          // To hold sensor model
  std::function<bool(size_t)>
    is_valid_packet_; /*Lambda Function Array to verify proper packet size for data*/
  std::function<bool(size_t)>
    is_valid_info_packet_; /*Lambda Function Array to verify proper packet size for info*/
  std::function<void(std::unique_ptr<robosense_msgs::msg::RobosenseScan> buffer)>
    scan_reception_callback_; /**This function pointer is called when the scan is complete*/
  std::function<void(std::unique_ptr<robosense_msgs::msg::RobosenseInfoPacket> buffer)>
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
  void ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) final;

  /// @brief Callback function to receive the Info Packet data from the UDP Driver
  /// @param buffer Buffer containing the data received from the UDP socket
  void ReceiveInfoPacketCallback(const std::vector<uint8_t> & buffer);

  /// @brief Starting the interface that handles UDP streams for MSOP packets
  /// @return Resulting status
  Status CloudInterfaceStart() final;

  /// @brief Starting the interface that handles UDP streams for DIFOP packets
  /// @return Resulting status
  Status InfoInterfaceStart();

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
  Status GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration) override;

  /// @brief Registering callback for RobosenseScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterScanCallback(
    std::function<void(std::unique_ptr<robosense_msgs::msg::RobosenseScan>)> scan_callback);

  /// @brief Registering callback for RobosensePacket
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterInfoCallback(
    std::function<void(std::unique_ptr<robosense_msgs::msg::RobosenseInfoPacket>)> info_callback);

  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> logger);
};

}  // namespace drivers
}  // namespace nebula
