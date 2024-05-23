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

#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"

#include <boost_tcp_driver/http_client_driver.hpp>
#include <boost_udp_driver/udp_driver.hpp>
#include <nebula_common/velodyne/velodyne_common.hpp>
#include <nebula_common/velodyne/velodyne_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{
/// @brief Hardware interface of velodyne driver
class VelodyneHwInterface
{
private:
  std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration_;
  std::function<void(std::vector<uint8_t> &)>
    cloud_packet_callback_; /**This function pointer is called when the scan is complete*/

  std::shared_ptr<boost::asio::io_context> boost_ctx_;
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> http_client_driver_;

  std::mutex mtx_inflight_request_;

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

  std::string HttpGetRequest(const std::string & endpoint);
  std::string HttpPostRequest(const std::string & endpoint, const std::string & body);

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
    std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration,
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
  Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration);
  /// @brief Printing calibration configuration
  /// @param calibration_configuration CalibrationConfiguration for the checking
  /// @return Resulting status
  Status GetCalibrationConfiguration(CalibrationConfigurationBase & calibration_configuration);
  /// @brief Initializing sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status InitializeSensorConfiguration(
    std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration);
  /// @brief Setting sensor configuration with InitializeSensorConfiguration &
  /// CheckAndSetConfigBySnapshotAsync
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status SetSensorConfiguration(
    std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration);
  /// @brief Registering callback for PandarScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status RegisterScanCallback(std::function<void(std::vector<uint8_t> & packet)> scan_callback);

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

  /// @brief Setting rclcpp::Logger
  /// @param node Logger
  void SetLogger(std::shared_ptr<rclcpp::Logger> node);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_VELODYNE_HW_INTERFACE_H
