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
constexpr double k_no_roi_value = 10000.0;

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
    std::make_unique<connections::HttpClient>(sensor_config_.connection.sensor_ip, k_http_port);
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
    std::string ip = sensor_config_.connection.host_ip;
    uint16_t port = sensor_config_.connection.udp_port;
    connections::UdpSocket::Builder builder{ip, port};
    udp_socket_.emplace(std::move(builder).bind());
  } catch (const std::exception &) {
    return Status::UDP_CONNECTION_ERROR;
  }

  udp_socket_->subscribe(std::move(scan_callback_));
  return Status::OK;
}

Status SeyondHwInterface::sensor_interface_stop()
{
  std::lock_guard<std::mutex> lock(interface_mutex_);

  if (!udp_socket_) {
    return Status::OK;
  }

  try {
    udp_socket_->unsubscribe();
    udp_socket_.reset();
  } catch (const std::exception &) {
    return Status::UDP_CONNECTION_ERROR;
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
    auto socket =
      connections::TcpSocket::Builder(sensor_config_.connection.sensor_ip, k_control_port)
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
  const auto ip = parse_ipv4_octets(sensor_ip);
  const auto mask = parse_ipv4_octets(netmask);
  const auto gw = parse_ipv4_octets(gateway.empty() ? std::string{"0.0.0.0"} : gateway);
  if (!ip || !mask || !gw) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  std::ostringstream command;
  command << "set_network " << static_cast<int>((*ip)[0]) << ' ' << static_cast<int>((*ip)[1])
          << ' ' << static_cast<int>((*ip)[2]) << ' ' << static_cast<int>((*ip)[3]) << ' '
          << static_cast<int>((*mask)[0]) << ' ' << static_cast<int>((*mask)[1]) << ' '
          << static_cast<int>((*mask)[2]) << ' ' << static_cast<int>((*mask)[3]) << ' '
          << static_cast<int>((*gw)[0]) << ' ' << static_cast<int>((*gw)[1]) << ' '
          << static_cast<int>((*gw)[2]) << ' ' << static_cast<int>((*gw)[3]);
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
  }

  return calibration;
}

Status SeyondHwInterface::setup_sensor(const SeyondSensorConfiguration & config)
{
  sensor_config_ = config;
  http_client_ =
    std::make_unique<connections::HttpClient>(sensor_config_.connection.sensor_ip, k_http_port);

  auto status = set_network(
    config.connection.sensor_ip, config.connection.netmask,
    config.connection.gateway.empty() ? std::string{"0.0.0.0"} : config.connection.gateway);
  if (status != Status::OK) {
    return status;
  }

  std::ostringstream network_val;
  network_val << config.connection.udp_port << ',' << config.connection.udp_message_port << ','
              << config.connection.udp_status_port << ',' << config.connection.host_ip;
  status = set_attribute("udp_ports_ip", network_val.str());
  if (status != Status::OK) {
    return status;
  }

  status =
    set_attribute("reflectance_mode", std::to_string(static_cast<int>(config.reflectance_mode)));
  if (status != Status::OK) {
    return status;
  }

  int return_val = 0;
  switch (config.return_mode) {
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
  status = set_attribute("return_mode", std::to_string(return_val));
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

  if (config.horizontal_roi != k_no_roi_value || config.vertical_roi != k_no_roi_value) {
    std::ostringstream roi_val;
    roi_val << format_double(config.horizontal_roi) << ',' << format_double(config.vertical_roi);
    status = set_attribute("roi", roi_val.str());
    if (status != Status::OK) {
      return status;
    }
  }

  return Status::OK;
}

}  // namespace nebula::drivers
