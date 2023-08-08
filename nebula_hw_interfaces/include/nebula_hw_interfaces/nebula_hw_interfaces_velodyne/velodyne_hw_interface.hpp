#ifndef NEBULA_VELODYNE_HW_INTERFACE_H
#define NEBULA_VELODYNE_HW_INTERFACE_H

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
#include "nebula_common/velodyne/velodyne_common.hpp"
#include "nebula_common/velodyne/velodyne_status.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"
#include "boost_tcp_driver/http_client_driver.hpp"
#include "boost_udp_driver/udp_driver.hpp"

#include <rclcpp/rclcpp.hpp>

#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>

namespace nebula
{
namespace drivers
{
/// @brief Hardware interface of velodyne driver
class VelodyneHwInterface : NebulaHwInterfaceBase
{
private:
  std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::shared_ptr<VelodyneSensorConfiguration> sensor_configuration_;
  std::shared_ptr<VelodyneCalibrationConfiguration> calibration_configuration_;
  std::unique_ptr<velodyne_msgs::msg::VelodyneScan> scan_cloud_ptr_;
  std::function<bool(size_t)>
    is_valid_packet_; /*Lambda Function Array to verify proper packet size*/
  std::function<void(std::unique_ptr<velodyne_msgs::msg::VelodyneScan> buffer)>
    scan_reception_callback_; /**This function pointer is called when the scan is complete*/

  uint16_t packet_first_azm_ = 0;
  uint16_t packet_first_azm_phased_ = 0;
  uint16_t packet_last_azm_ = 0;
  uint16_t packet_last_azm_phased_ = 0;
  uint16_t prev_packet_first_azm_phased_ = 0;
  uint16_t phase_ = 0;
  uint processed_packets_ = 0;

  std::shared_ptr<boost::asio::io_context> boost_ctx_;
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> http_client_driver_;

  std::string TARGET_STATUS{"/cgi/status.json"};
  std::string TARGET_DIAG{"/cgi/diag.json"};
  std::string TARGET_SNAPSHOT{"/cgi/snapshot.hdl"};
  std::string TARGET_SETTING{"/cgi/setting"};
  std::string TARGET_FOV{"/cgi/setting/fov"};
  std::string TARGET_HOST{"/cgi/setting/host"};
  std::string TARGET_NET{"/cgi/setting/net"};
  std::string TARGET_SAVE{"/cgi/save"};
  std::string TARGET_RESET{"/cgi/reset"};
  void StringCallback(const std::string & str);

  /// @brief Get a one-off HTTP client to communicate with the hardware
  /// @param ctx IO Context
  /// @param hcd Got http client driver
  /// @return Resulting status
  VelodyneStatus GetHttpClientDriverOnce(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd);
  /// @brief Get a one-off HTTP client to communicate with the hardware (without specifying
  /// io_context)
  /// @param hcd Got http client driver
  /// @return Resulting status
  VelodyneStatus GetHttpClientDriverOnce(
    std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd);

  /// @brief Checking the current settings and changing the difference point
  /// @param sensor_configuration Current SensorConfiguration
  /// @param tree Current settings (property_tree)
  /// @return Resulting status
  VelodyneStatus CheckAndSetConfig(
    std::shared_ptr<VelodyneSensorConfiguration> sensor_configuration,
    boost::property_tree::ptree tree);

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

public:
  /// @brief Constructor
  VelodyneHwInterface();

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
  Status GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration) final;
  /// @brief Initializing sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status InitializeSensorConfiguration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration);
  /// @brief Setting sensor configuration with InitializeSensorConfiguration &
  /// CheckAndSetConfigBySnapshotAsync
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status SetSensorConfiguration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration) final;
  /// @brief Registering callback for PandarScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterScanCallback(
    std::function<void(std::unique_ptr<velodyne_msgs::msg::VelodyneScan>)> scan_callback);

  /// @brief Parsing JSON string to property_tree
  /// @param str JSON string
  /// @return property_tree
  boost::property_tree::ptree ParseJson(const std::string & str);

  /// @brief Initializing HTTP client (sync)
  /// @return Resulting status
  VelodyneStatus InitHttpClient();
  /// @brief Getting the current operational state and parameters of the sensor (sync)
  /// @return Resulting JSON string
  std::string GetStatus();
  /// @brief Getting diagnostic information from the sensor (sync)
  /// @return Resulting JSON string
  std::string GetDiag();
  /// @brief Getting current sensor configuration and status data (sync)
  /// @return Resulting JSON string
  std::string GetSnapshot();
  /// @brief Setting Motor RPM (sync)
  /// @param rpm the RPM of the motor
  /// @return Resulting status
  VelodyneStatus SetRpm(uint16_t rpm);
  /// @brief Setting Field of View Start (sync)
  /// @param fov_start FOV start
  /// @return Resulting status
  VelodyneStatus SetFovStart(uint16_t fov_start);
  /// @brief Setting Field of View End (sync)
  /// @param fov_end FOV end
  /// @return Resulting status
  VelodyneStatus SetFovEnd(uint16_t fov_end);
  /// @brief Setting Return Type (sync)
  /// @param return_mode ReturnMode
  /// @return Resulting status
  VelodyneStatus SetReturnType(ReturnMode return_mode);
  /// @brief Save Configuration to the LiDAR memory (sync)
  /// @return Resulting status
  VelodyneStatus SaveConfig();
  /// @brief Resets the sensor (sync)
  /// @return Resulting status
  VelodyneStatus ResetSystem();
  /// @brief Turn laser state on (sync)
  /// @return Resulting status
  VelodyneStatus LaserOn();
  /// @brief Turn laser state off (sync)
  /// @return Resulting status
  VelodyneStatus LaserOff();
  /// @brief Turn laser state on/off (sync)
  /// @param on is ON
  /// @return Resulting status
  VelodyneStatus LaserOnOff(bool on);
  /// @brief Setting host (destination) IP address (sync)
  /// @param addr destination IP address
  /// @return Resulting status
  VelodyneStatus SetHostAddr(std::string addr);
  /// @brief Setting host (destination) data port (sync)
  /// @param dport destination data port
  /// @return Resulting status
  VelodyneStatus SetHostDport(uint16_t dport);
  /// @brief Setting host (destination) telemetry port (sync)
  /// @param tport destination telemetry port
  /// @return Resulting status
  VelodyneStatus SetHostTport(uint16_t tport);
  /// @brief Setting network (sensor) IP address (sync)
  /// @param addr sensor IP address
  /// @return Resulting status
  VelodyneStatus SetNetAddr(std::string addr);
  /// @brief Setting the network mask of the sensor (sync)
  /// @param mask Network mask
  /// @return Resulting status
  VelodyneStatus SetNetMask(std::string mask);
  /// @brief Setting the gateway address of the sensor (sync)
  /// @param gateway Gateway address
  /// @return Resulting status
  VelodyneStatus SetNetGateway(std::string gateway);
  /// @brief This determines if the sensor is to rely on a DHCP server for its IP address (sync)
  /// @param use_dhcp DHCP on
  /// @return Resulting status
  VelodyneStatus SetNetDhcp(bool use_dhcp);

  /// @brief Initializing HTTP client (async)
  /// @return Resulting status
  VelodyneStatus InitHttpClientAsync();
  /// @brief Getting the current operational state and parameters of the sensor (async)
  /// @param str_callback Callback function for received JSON string
  /// @return Resulting status
  VelodyneStatus GetStatusAsync(std::function<void(const std::string & str)> str_callback);
  /// @brief Getting the current operational state and parameters of the sensor (async)
  /// @return Resulting status
  VelodyneStatus GetStatusAsync();
  /// @brief Getting diagnostic information from the sensor (async)
  /// @param str_callback Callback function for received JSON string
  /// @return Resulting status
  VelodyneStatus GetDiagAsync(std::function<void(const std::string & str)> str_callback);
  /// @brief Getting diagnostic information from the sensor (async)
  /// @return Resulting status
  VelodyneStatus GetDiagAsync();
  /// @brief Getting current sensor configuration and status data (async)
  /// @param str_callback Callback function for received JSON string
  /// @return Resulting status
  VelodyneStatus GetSnapshotAsync(std::function<void(const std::string & str)> str_callback);
  /// @brief Getting current sensor configuration and status data (async)
  /// @return Resulting status
  VelodyneStatus GetSnapshotAsync();
  /// @brief Checking the current settings and changing the difference point
  /// @return Resulting status
  VelodyneStatus CheckAndSetConfigBySnapshotAsync();
  /// @brief Setting Motor RPM (async)
  /// @param rpm the RPM of the motor
  /// @return Resulting status
  VelodyneStatus SetRpmAsync(uint16_t rpm);
  /// @brief Setting Field of View Start (async)
  /// @param fov_start FOV start
  /// @return Resulting status
  VelodyneStatus SetFovStartAsync(uint16_t fov_start);
  /// @brief Setting Field of View End (async)
  /// @param fov_end FOV end
  /// @return Resulting status
  VelodyneStatus SetFovEndAsync(uint16_t fov_end);
  /// @brief Setting Return Type (async)
  /// @param return_mode ReturnMode
  /// @return Resulting status
  VelodyneStatus SetReturnTypeAsync(ReturnMode return_mode);
  /// @brief Save Configuration to the LiDAR memory (async)
  /// @return Resulting status
  VelodyneStatus SaveConfigAsync();
  /// @brief Resets the sensor (async)
  /// @return Resulting status
  VelodyneStatus ResetSystemAsync();
  /// @brief Turn laser state on (async)
  /// @return Resulting status
  VelodyneStatus LaserOnAsync();
  /// @brief Turn laser state off (async)
  /// @return Resulting status
  VelodyneStatus LaserOffAsync();
  /// @brief Turn laser state on/off (async)
  /// @param on is ON
  /// @return Resulting status
  VelodyneStatus LaserOnOffAsync(bool on);
  /// @brief Setting host (destination) IP address (async)
  /// @param addr destination IP address
  /// @return Resulting status
  VelodyneStatus SetHostAddrAsync(std::string addr);
  /// @brief Setting host (destination) data port (async)
  /// @param dport destination data port
  /// @return Resulting status
  VelodyneStatus SetHostDportAsync(uint16_t dport);
  /// @brief Setting host (destination) telemetry port (async)
  /// @param tport destination telemetry port
  /// @return Resulting status
  VelodyneStatus SetHostTportAsync(uint16_t tport);
  /// @brief Setting network (sensor) IP address (async)
  /// @param addr sensor IP address
  /// @return Resulting status
  VelodyneStatus SetNetAddrAsync(std::string addr);
  /// @brief Setting the network mask of the sensor (async)
  /// @param mask Network mask
  /// @return Resulting status
  VelodyneStatus SetNetMaskAsync(std::string mask);
  /// @brief Setting the gateway address of the sensor (async)
  /// @param gateway Gateway address
  /// @return Resulting status
  VelodyneStatus SetNetGatewayAsync(std::string gateway);
  /// @brief This determines if the sensor is to rely on a DHCP server for its IP address (async)
  /// @param use_dhcp DHCP on
  /// @return Resulting status
  VelodyneStatus SetNetDhcpAsync(bool use_dhcp);

  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> node);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_VELODYNE_HW_INTERFACE_H
