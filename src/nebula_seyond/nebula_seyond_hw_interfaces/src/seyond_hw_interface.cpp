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

#include <nebula_seyond_hw_interfaces/seyond_hw_interface.hpp>

#include <arpa/inet.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{
namespace
{
constexpr double no_roi_value = 10000.0;
constexpr size_t k_seyond_udp_mtu = 10000;
constexpr size_t k_seyond_udp_socket_buffer_size = 4 * 1024 * 1024;

std::optional<std::array<uint8_t, 4>> parse_ipv4_octets(const std::string & ip)
{
  in_addr addr{};
  if (inet_pton(AF_INET, ip.c_str(), &addr) != 1) {
    return std::nullopt;
  }

  const auto * bytes = reinterpret_cast<const uint8_t *>(&addr.s_addr);
  return std::array<uint8_t, 4>{bytes[0], bytes[1], bytes[2], bytes[3]};
}

std::string format_double(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << value;
  return stream.str();
}
}  // namespace

SeyondHwInterface::SeyondHwInterface(const SeyondSensorConfiguration & config)
: sensor_config_(config)
{
  http_client_ =
    std::make_unique<connections::HttpClient>(sensor_config_.connection.sensor_ip, http_port);
}

bool SeyondHwInterface::requires_direct_start_command() const
{
  return sensor_config_.sensor_model == SeyondSensorModel::HUMMINGBIRD_D1 ||
         sensor_config_.sensor_model == SeyondSensorModel::ROBIN_W ||
         sensor_config_.sensor_model == SeyondSensorModel::ROBIN_E1X ||
         sensor_config_.sensor_model == SeyondSensorModel::FALCON_K;
}

bool SeyondHwInterface::uses_four_field_udp_ports_ip() const
{
  return sensor_config_.sensor_model == SeyondSensorModel::HUMMINGBIRD_D1 ||
         sensor_config_.sensor_model == SeyondSensorModel::ROBIN_W ||
         sensor_config_.sensor_model == SeyondSensorModel::FALCON_K;
}

bool SeyondHwInterface::uses_six_field_udp_ports_ip() const
{
  return !uses_four_field_udp_ports_ip();
}

std::string SeyondHwInterface::build_udp_ports_ip_value(
  const SeyondConnectionConfiguration & config) const
{
  std::ostringstream network_val;
  network_val << config.udp_port << ',' << config.udp_message_port << ',' << config.udp_status_port
              << ',' << config.host_ip;

  if (uses_six_field_udp_ports_ip()) {
    network_val << ',' << config.host_ip << ',' << config.sensor_ip;
  }

  return network_val.str();
}

Status SeyondHwInterface::download_binary_file(
  const std::string & command, std::vector<uint8_t> & output)
{
  output.clear();

  try {
    auto socket = connections::TcpSocket::Builder(sensor_config_.connection.sensor_ip, control_port)
                    .set_connect_timeout(2000)
                    .connect();

    std::string wire_command = command + "\n";
    socket.send(std::vector<uint8_t>(wire_command.begin(), wire_command.end()));

    // Read 4-byte big-endian length prefix
    auto header = socket.receive(std::chrono::milliseconds(2000));
    if (header.size() < 4) {
      return Status::ERROR_1;
    }

    uint32_t length = (static_cast<uint32_t>(header[0]) << 24) |
                      (static_cast<uint32_t>(header[1]) << 16) |
                      (static_cast<uint32_t>(header[2]) << 8) | static_cast<uint32_t>(header[3]);
    if (length == 0) {
      return Status::ERROR_1;
    }

    output.reserve(length);

    // Remaining data in first chunk
    if (header.size() > 4) {
      const auto payload_end =
        header.begin() + 4 + std::min<size_t>(length, static_cast<size_t>(header.size() - 4));
      output.insert(output.end(), header.begin() + 4, payload_end);
    }

    while (output.size() < length) {
      auto chunk = socket.receive(std::chrono::milliseconds(2000));
      if (chunk.empty()) {
        output.clear();
        return Status::ERROR_1;
      }

      const auto remaining = length - output.size();
      output.insert(
        output.end(), chunk.begin(), chunk.begin() + std::min<size_t>(remaining, chunk.size()));
    }

    if (output.size() != length) {
      output.clear();
      return Status::ERROR_1;
    }

    return Status::OK;
  } catch (const std::exception &) {
    output.clear();
    return Status::ERROR_1;
  }
}
Status SeyondHwInterface::sensor_interface_start()
{
  std::lock_guard<std::mutex> lock(interface_mutex_);

  if (!scan_callback_) {
    return Status::NOT_INITIALIZED;
  }

  if (udp_socket_) {
    return Status::OK;
  }

  try {
    std::string host_ip = sensor_config_.connection.host_ip;
    uint16_t port = sensor_config_.connection.udp_port;
    connections::UdpSocket::Builder builder{host_ip, port};
    builder.set_mtu(k_seyond_udp_mtu).set_socket_buffer_size(k_seyond_udp_socket_buffer_size);
    udp_socket_.emplace(std::move(builder).bind());
  } catch (const std::exception &) {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (requires_direct_start_command()) {
    try {
      streaming_control_socket_ = std::make_unique<connections::TcpSocket>(
        connections::TcpSocket::Builder(sensor_config_.connection.sensor_ip, control_port)
          .set_connect_timeout(2000)
          .set_buffer_size(4096)
          .connect());

      std::string start_cmd = "start\n";
      streaming_control_socket_->send(std::vector<uint8_t>(start_cmd.begin(), start_cmd.end()));

      std::string direct_cmd =
        "start direct " + std::to_string(sensor_config_.connection.udp_port) + '\n';
      streaming_control_socket_->send(std::vector<uint8_t>(direct_cmd.begin(), direct_cmd.end()));
    } catch (const std::exception &) {
      streaming_control_socket_.reset();
      udp_socket_.reset();
      return Status::ERROR_1;
    }
  }
  udp_socket_->subscribe(std::move(scan_callback_));
  return Status::OK;
}

Status SeyondHwInterface::sensor_interface_stop()
{
  std::lock_guard<std::mutex> lock(interface_mutex_);

  if (udp_socket_) {
    try {
      udp_socket_->unsubscribe();
      udp_socket_.reset();
    } catch (const std::exception &) {
      return Status::UDP_CONNECTION_ERROR;
    }
  }

  if (streaming_control_socket_) {
    try {
      std::string stop_cmd = "stop\n";
      streaming_control_socket_->send(std::vector<uint8_t>(stop_cmd.begin(), stop_cmd.end()));
      streaming_control_socket_.reset();
    } catch (const std::exception &) {
    }
  }

  return Status::OK;
}

Status SeyondHwInterface::register_scan_callback(connections::UdpSocket::callback_t scan_callback)
{
  if (!scan_callback) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  scan_callback_ = std::move(scan_callback);
  return Status::OK;
}

Status SeyondHwInterface::set_attribute(const std::string & name, const std::string & value)
{
  try {
    std::string endpoint = "/command/?set_" + name + "=" + value;
    http_client_->get(endpoint);
    return Status::OK;
  } catch (const std::exception &) {
    return Status::HTTP_CONNECTION_ERROR;
  }
}

Status SeyondHwInterface::get_attribute(const std::string & name, std::string & response)
{
  try {
    std::string endpoint = "/command/?get_" + name;
    response = http_client_->get(endpoint);
    return Status::OK;
  } catch (const std::exception &) {
    return Status::HTTP_CONNECTION_ERROR;
  }
}

Status SeyondHwInterface::send_raw_command(
  const std::string & command, std::string * response, int timeout_ms)
{
  try {
    auto socket = connections::TcpSocket::Builder(sensor_config_.connection.sensor_ip, control_port)
                    .set_connect_timeout(timeout_ms)
                    .set_buffer_size(4096)
                    .connect();

    std::string wire_command = command + "\n";
    socket.send(std::vector<uint8_t>(wire_command.begin(), wire_command.end()));

    std::string reply;
    const auto timeout = std::chrono::milliseconds(timeout_ms);
    while (true) {
      try {
        auto chunk = socket.receive(timeout);
        reply.append(reinterpret_cast<const char *>(chunk.data()), chunk.size());
        if (reply.find("\n\n") != std::string::npos) {
          break;
        }
      } catch (const std::exception &) {
        if (!reply.empty()) {
          break;
        }
        return Status::ERROR_1;
      }
    }

    while (reply.size() >= 2 && reply.substr(reply.size() - 2) == "\n\n") {
      reply.resize(reply.size() - 2);
    }
    while (!reply.empty() && (reply.back() == '\n' || reply.back() == '\r')) {
      reply.pop_back();
    }

    if (response) {
      *response = reply;
    }
    return Status::OK;
  } catch (const std::exception &) {
    return Status::ERROR_1;
  }
}

Status SeyondHwInterface::set_network(
  const std::string & sensor_ip, const std::string & netmask, const std::string & gateway)
{
  const auto source_ip = parse_ipv4_octets(sensor_ip);
  const auto network_mask = parse_ipv4_octets(netmask);
  const auto gateway_ip = parse_ipv4_octets(gateway.empty() ? std::string{"0.0.0.0"} : gateway);
  if (!source_ip || !network_mask || !gateway_ip) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  std::ostringstream command;
  command << "set_network " << static_cast<int>((*source_ip)[0]) << ' '
          << static_cast<int>((*source_ip)[1]) << ' ' << static_cast<int>((*source_ip)[2]) << ' '
          << static_cast<int>((*source_ip)[3]) << ' ' << static_cast<int>((*network_mask)[0]) << ' '
          << static_cast<int>((*network_mask)[1]) << ' ' << static_cast<int>((*network_mask)[2])
          << ' ' << static_cast<int>((*network_mask)[3]) << ' '
          << static_cast<int>((*gateway_ip)[0]) << ' ' << static_cast<int>((*gateway_ip)[1]) << ' '
          << static_cast<int>((*gateway_ip)[2]) << ' ' << static_cast<int>((*gateway_ip)[3]);
  return send_raw_command(command.str());
}

Status SeyondHwInterface::set_frame_rate(double frame_rate)
{
  if (!std::isfinite(frame_rate) || frame_rate <= 0.0) {
    return Status::SENSOR_CONFIG_ERROR;
  }
  return send_raw_command("set_i_config motor galvo_framerate " + format_double(frame_rate));
}

Status SeyondHwInterface::set_time_sync(SeyondSyncMode sync_mode)
{
  return send_raw_command(
    "set_i_config time time_stamping " + std::to_string(static_cast<int>(sync_mode)));
}

Status SeyondHwInterface::set_reflectance_mode(SeyondReflectanceMode reflectance_mode)
{
  return set_attribute("reflectance_mode", std::to_string(static_cast<int>(reflectance_mode)));
}

Status SeyondHwInterface::save_configuration()
{
  const auto http_status = set_attribute("save_network", "1");
  const auto raw_status = send_raw_command("save_config");

  if (http_status == Status::OK || raw_status == Status::OK) {
    return Status::OK;
  }

  return raw_status != Status::OK ? raw_status : http_status;
}

Status SeyondHwInterface::set_return_mode(ReturnMode return_mode)
{
  int return_val = 0;
  switch (return_mode) {
    case ReturnMode::STRONGEST:
      return_val = 1;
      break;
    case ReturnMode::DUAL:
      return_val = 2;
      break;
    case ReturnMode::DUAL_LAST_STRONGEST:
      return_val = 3;
      break;
    default:
      return Status::SENSOR_CONFIG_ERROR;
  }
  return set_attribute("return_mode", std::to_string(return_val));
}
util::expected<SeyondCalibrationData, SeyondCalibrationData::Error>
SeyondHwInterface::get_calibration()
{
  SeyondCalibrationData calibration;
  std::string response;

  auto status = get_attribute("v_angle_offset", response);
  if (status == Status::OK) {
    try {
      calibration.v_angle_offset = std::stod(response);
    } catch (const std::exception &) {
    }
  }

  if (
    sensor_config_.sensor_model == SeyondSensorModel::ROBIN_W ||
    sensor_config_.sensor_model == SeyondSensorModel::ROBIN_E1X ||
    sensor_config_.sensor_model == SeyondSensorModel::HUMMINGBIRD_D1) {
    try {
      std::string table_response = http_client_->get("/command/?get_anglehv_table", 2000);
      calibration.angle_hv_table.assign(table_response.begin(), table_response.end());
    } catch (const std::exception &) {
    }
    if (
      sensor_config_.sensor_model == SeyondSensorModel::ROBIN_E1X ||
      sensor_config_.sensor_model == SeyondSensorModel::HUMMINGBIRD_D1) {
      if (download_binary_file("download_cal_file 0", calibration.geo_yaml) != Status::OK) {
        download_binary_file("get_geo_yaml", calibration.geo_yaml);
      }
      download_binary_file("download_cal_file 1", calibration.sn_yaml);
    }
  }

  return calibration;
}

Status SeyondHwInterface::setup_sensor(const SeyondSensorConfiguration & config)
{
  sensor_config_ = config;
  http_client_ =
    std::make_unique<connections::HttpClient>(sensor_config_.connection.sensor_ip, http_port);

  auto status = set_network(
    config.connection.sensor_ip, config.connection.netmask,
    config.connection.gateway.empty() ? std::string{"0.0.0.0"} : config.connection.gateway);
  if (status != Status::OK) {
    return status;
  }

  status = set_attribute("udp_ports_ip", build_udp_ports_ip_value(config.connection));
  if (status != Status::OK) {
    return status;
  }

  status = set_reflectance_mode(config.reflectance_mode);
  if (status != Status::OK) {
    return status;
  }

  status = set_return_mode(config.return_mode);
  if (status != Status::OK) {
    return status;
  }

  status = set_time_sync(config.sync_mode);
  if (status != Status::OK) {
    return status;
  }

  if (config.frame_rate > 0.0) {
    status = set_frame_rate(config.frame_rate);
    if (status != Status::OK) {
      return status;
    }
  }

  if (config.horizontal_roi != no_roi_value || config.vertical_roi != no_roi_value) {
    std::ostringstream roi_val;
    roi_val << format_double(config.horizontal_roi) << ',' << format_double(config.vertical_roi);
    status = set_attribute("roi", roi_val.str());
    if (status != Status::OK) {
      return status;
    }
  }

  status = set_attribute("enabled", "1");
  if (status != Status::OK) {
    return status;
  }

  status = set_attribute("mode", "3");
  if (status != Status::OK) {
    return status;
  }

  status = save_configuration();
  if (status != Status::OK) {
    return status;
  }

  return Status::OK;
}

}  // namespace nebula::drivers
