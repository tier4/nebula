// Copyright 2026 TIER IV, Inc.
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

#ifndef NEBULA_SEYOND_HW_INTERFACE_HPP
#define NEBULA_SEYOND_HW_INTERFACE_HPP

#include <nebula_core_common/nebula_status.hpp>
#include <nebula_core_hw_interfaces/connections/http_client.hpp>
#include <nebula_core_hw_interfaces/connections/tcp.hpp>
#include <nebula_core_hw_interfaces/connections/udp.hpp>
#include <nebula_seyond_common/seyond_calibration_data.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace nebula::drivers
{

/// @brief Receives raw Seyond packets and forwards them to a registered callback.
class SeyondHwInterface
{
public:
  explicit SeyondHwInterface(const SeyondSensorConfiguration & config);

  /// @brief Initialize the hardware interface and start receiving packets.
  Status sensor_interface_start();

  /// @brief Stop receiving packets and close the interface.
  Status sensor_interface_stop();

  /// @brief Register a callback for incoming UDP packets.
  Status register_scan_callback(connections::UdpSocket::callback_t scan_callback);

  /// @brief Configure a sensor attribute via HTTP.
  Status set_attribute(const std::string & name, const std::string & value);

  /// @brief Get a sensor attribute via HTTP.
  Status get_attribute(const std::string & name, std::string & response);

  /// @brief Fetch calibration data from the sensor.
  util::expected<SeyondCalibrationData, SeyondCalibrationData::Error> get_calibration();

  /// @brief Configure the sensor's destination IP/port and other settings.
  Status setup_sensor(const SeyondSensorConfiguration & config);

private:
  Status send_raw_command(
    const std::string & command, std::string * response = nullptr, int timeout_ms = 2000);
  Status set_network(
    const std::string & sensor_ip, const std::string & netmask, const std::string & gateway);
  Status save_configuration();
  Status set_reflectance_mode(SeyondReflectanceMode reflectance_mode);
  Status set_return_mode(ReturnMode return_mode);
  Status set_time_sync(SeyondSyncMode sync_mode);
  Status set_frame_rate(double frame_rate);
  bool requires_direct_start_command() const;
  bool uses_four_field_udp_ports_ip() const;
  bool uses_six_field_udp_ports_ip() const;
  std::string build_udp_ports_ip_value(const SeyondConnectionConfiguration & config) const;
  Status download_binary_file(const std::string & command, std::vector<uint8_t> & output);

  SeyondSensorConfiguration sensor_config_;
  connections::UdpSocket::callback_t scan_callback_;
  std::optional<connections::UdpSocket> udp_socket_;
  std::unique_ptr<connections::HttpClient> http_client_;
  std::unique_ptr<connections::TcpSocket> streaming_control_socket_;
  std::mutex interface_mutex_;

  static constexpr uint16_t http_port = 8010;
  static constexpr uint16_t control_port = 8002;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SEYOND_HW_INTERFACE_HPP
