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

#ifndef NEBULA_VELODYNE_HW_INTERFACE_H
#define NEBULA_VELODYNE_HW_INTERFACE_H

// Have to define macros to silence warnings about deprecated headers being used by
// boost/property_tree/ in some versions of boost.
// See: https://github.com/boostorg/property_tree/issues/51
#include <boost/version.hpp>

#include <optional>
#if (BOOST_VERSION / 100 >= 1073 && BOOST_VERSION / 100 <= 1076)  // Boost 1.73 - 1.76
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#endif
#if (BOOST_VERSION / 100 == 1074)  // Boost 1.74
#define BOOST_ALLOW_DEPRECATED_HEADERS
#endif

#include "nebula_common/util/expected.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"

#include <boost_tcp_driver/http_client_driver.hpp>
#include <boost_udp_driver/udp_driver.hpp>
#include <nebula_common/loggers/logger.hpp>
#include <nebula_common/velodyne/velodyne_common.hpp>
#include <nebula_common/velodyne/velodyne_status.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace nebula::drivers
{
/// @brief Hardware interface of velodyne driver
class VelodyneHwInterface
{
private:
  std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration_;
  /**This function pointer is called when the scan is complete*/
  std::function<void(const std::vector<uint8_t> &)> cloud_packet_callback_;
  // Calls the above `cloud_packet_callback_` and thus has to be destroyed before it.
  std::optional<connections::UdpSocket> udp_socket_;

  std::shared_ptr<boost::asio::io_context> boost_ctx_;
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> http_client_driver_;

  std::mutex mtx_inflight_request_;

  std::string target_status_{"/cgi/status.json"};
  std::string target_diag_{"/cgi/diag.json"};
  std::string target_snapshot_{"/cgi/snapshot.hdl"};
  std::string target_setting_{"/cgi/setting"};
  std::string target_fov_{"/cgi/setting/fov"};
  std::string target_host_{"/cgi/setting/host"};
  std::string target_net_{"/cgi/setting/net"};
  std::string target_save_{"/cgi/save"};
  std::string target_reset_{"/cgi/reset"};
  void string_callback(const std::string & str);

  template <typename CallbackType>
  nebula::util::expected<std::string, VelodyneStatus> do_http_request_with_retries(
    CallbackType do_request, std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & client)
  {
    constexpr int max_retries = 3;
    constexpr int retry_delay_ms = 100;

    for (int retry = 0; retry < max_retries; ++retry) {
      try {
        if (!client->client()->isOpen()) {
          client->client()->open();
        }

        std::string response = do_request();
        client->client()->close();
        return nebula::util::expected<std::string, VelodyneStatus>(response);
      } catch (const std::exception & ex) {
        if (retry == max_retries - 1) {
          return nebula::util::expected<std::string, VelodyneStatus>(
            VelodyneStatus::HTTP_CONNECTION_ERROR);
        }

        if (client->client()->isOpen()) {
          try {
            client->client()->close();
          } catch (const std::exception & ex) {
            return nebula::util::expected<std::string, VelodyneStatus>(
              VelodyneStatus::HTTP_CONNECTION_ERROR);
          }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
      }
    }

    return nebula::util::expected<std::string, VelodyneStatus>(
      VelodyneStatus::HTTP_CONNECTION_ERROR);
  }

  nebula::util::expected<std::string, VelodyneStatus> http_get_request(
    const std::string & endpoint);
  nebula::util::expected<std::string, VelodyneStatus> http_post_request(
    const std::string & endpoint, const std::string & body);

  /// @brief Get a one-off HTTP client to communicate with the hardware
  /// @param ctx IO Context
  /// @param hcd Got http client driver
  /// @return Resulting status
  VelodyneStatus get_http_client_driver_once(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd);
  /// @brief Get a one-off HTTP client to communicate with the hardware (without specifying
  /// io_context)
  /// @param hcd Got http client driver
  /// @return Resulting status
  VelodyneStatus get_http_client_driver_once(
    std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd);

  /// @brief Checking the current settings and changing the difference point
  /// @param sensor_configuration Current SensorConfiguration
  /// @param tree Current settings (property_tree)
  /// @return Resulting status
  VelodyneStatus check_and_set_config(
    std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration,
    boost::property_tree::ptree tree);

  std::shared_ptr<loggers::Logger> logger_;

public:
  /// @brief Constructor
  explicit VelodyneHwInterface(const std::shared_ptr<loggers::Logger> & logger);

  virtual ~VelodyneHwInterface() = default;

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
  Status get_sensor_configuration(SensorConfigurationBase & sensor_configuration);
  /// @brief Printing calibration configuration
  /// @param calibration_configuration CalibrationConfiguration for the checking
  /// @return Resulting status
  Status get_calibration_configuration(CalibrationConfigurationBase & calibration_configuration);
  /// @brief Initializing sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status initialize_sensor_configuration(
    std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration);
  /// @brief Setting sensor configuration with InitializeSensorConfiguration &
  /// CheckAndSetConfigBySnapshotAsync
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  Status set_sensor_configuration(
    std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration);
  /// @brief Registering callback for PandarScan
  /// @param scan_callback Callback function
  /// @return Resulting status
  Status register_scan_callback(
    std::function<void(const std::vector<uint8_t> & packet)> scan_callback);

  /// @brief Parsing JSON string to property_tree
  /// @param str JSON string
  /// @return property_tree
  boost::property_tree::ptree parse_json(const std::string & str);

  /// @brief Initializing HTTP client (sync)
  /// @return Resulting status
  VelodyneStatus init_http_client();
  /// @brief Getting the current operational state and parameters of the sensor (sync)
  /// @return Resulting JSON string
  nebula::util::expected<std::string, VelodyneStatus> get_status();
  /// @brief Getting diagnostic information from the sensor (sync)
  /// @return Resulting JSON string
  nebula::util::expected<std::string, VelodyneStatus> get_diag();
  /// @brief Getting current sensor configuration and status data (sync)
  /// @return Resulting JSON string
  nebula::util::expected<std::string, VelodyneStatus> get_snapshot();
  /// @brief Setting Motor RPM (sync)
  /// @param rpm the RPM of the motor
  /// @return Resulting status
  VelodyneStatus set_rpm(uint16_t rpm);
  /// @brief Setting Field of View Start (sync)
  /// @param fov_start FOV start
  /// @return Resulting status
  VelodyneStatus set_fov_start(uint16_t fov_start);
  /// @brief Setting Field of View End (sync)
  /// @param fov_end FOV end
  /// @return Resulting status
  VelodyneStatus set_fov_end(uint16_t fov_end);
  /// @brief Setting Return Type (sync)
  /// @param return_mode ReturnMode
  /// @return Resulting status
  VelodyneStatus set_return_type(ReturnMode return_mode);
  /// @brief Save Configuration to the LiDAR memory (sync)
  /// @return Resulting status
  VelodyneStatus save_config();
  /// @brief Resets the sensor (sync)
  /// @return Resulting status
  VelodyneStatus reset_system();
  /// @brief Turn laser state on (sync)
  /// @return Resulting status
  VelodyneStatus laser_on();
  /// @brief Turn laser state off (sync)
  /// @return Resulting status
  VelodyneStatus laser_off();
  /// @brief Turn laser state on/off (sync)
  /// @param on is ON
  /// @return Resulting status
  VelodyneStatus laser_on_off(bool on);
  /// @brief Setting host (destination) IP address (sync)
  /// @param addr destination IP address
  /// @return Resulting status
  VelodyneStatus set_host_addr(std::string addr);
  /// @brief Setting host (destination) data port (sync)
  /// @param dport destination data port
  /// @return Resulting status
  VelodyneStatus set_host_dport(uint16_t dport);
  /// @brief Setting host (destination) telemetry port (sync)
  /// @param tport destination telemetry port
  /// @return Resulting status
  VelodyneStatus set_host_tport(uint16_t tport);
  /// @brief Setting network (sensor) IP address (sync)
  /// @param addr sensor IP address
  /// @return Resulting status
  VelodyneStatus set_net_addr(std::string addr);
  /// @brief Setting the network mask of the sensor (sync)
  /// @param mask Network mask
  /// @return Resulting status
  VelodyneStatus set_net_mask(std::string mask);
  /// @brief Setting the gateway address of the sensor (sync)
  /// @param gateway Gateway address
  /// @return Resulting status
  VelodyneStatus set_net_gateway(std::string gateway);
  /// @brief This determines if the sensor is to rely on a DHCP server for its IP address (sync)
  /// @param use_dhcp DHCP on
  /// @return Resulting status
  VelodyneStatus set_net_dhcp(bool use_dhcp);
};
}  // namespace nebula::drivers

#endif  // NEBULA_VELODYNE_HW_INTERFACE_H
