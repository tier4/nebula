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

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{

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
  } catch (const std::exception & e) {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (scan_callback_) {
    udp_socket_->subscribe(std::move(scan_callback_));
  }

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
  } catch (const std::exception & e) {
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
  } catch (const std::exception & e) {
    return Status::HTTP_CONNECTION_ERROR;
  }
}

Status SeyondHwInterface::get_attribute(const std::string & name, std::string & response)
{
  try {
    std::string endpoint = "/command/?get_" + name;
    response = http_client_->get(endpoint);
    return Status::OK;
  } catch (const std::exception & e) {
    return Status::HTTP_CONNECTION_ERROR;
  }
}

util::expected<SeyondCalibrationData, SeyondCalibrationData::Error>
SeyondHwInterface::get_calibration()
{
  SeyondCalibrationData calibration;
  std::string response;

  // Fetch v_angle_offset for all models if available
  auto status = get_attribute("v_angle_offset", response);
  if (status == Status::OK) {
    try {
      calibration.v_angle_offset = std::stod(response);
    } catch (const std::exception & e) {
      // If fail to parse, keep default 0.0 or handle error
    }
  }

  // Fetch anglehv_table for Robin W, Robin E1X, and Hummingbird D1
  if (
    sensor_config_.sensor_model == SeyondSensorModel::ROBIN_W ||
    sensor_config_.sensor_model == SeyondSensorModel::ROBIN_E1X ||
    sensor_config_.sensor_model == SeyondSensorModel::HUMMINGBIRD_D1) {
    try {
      std::string table_response = http_client_->get("/command/?get_anglehv_table", 2000);
      calibration.angle_hv_table.assign(table_response.begin(), table_response.end());
    } catch (const std::exception & e) {
      // Log or handle error: failed to fetch anglehv_table
    }
  }

  return calibration;
}

Status SeyondHwInterface::setup_sensor(const SeyondSensorConfiguration & config)
{
  // Set destination network settings
  std::string network_val =
    std::to_string(config.connection.udp_port) + "," + std::to_string(config.connection.udp_port) +
    "," + std::to_string(config.connection.udp_port) + "," + config.connection.host_ip;
  auto status = set_attribute("udp_ports_ip", network_val);
  if (status != Status::OK) {
    return status;
  }

  // Set reflectance mode
  status =
    set_attribute("reflectance_mode", std::to_string(static_cast<int>(config.reflectance_mode)));
  if (status != Status::OK) {
    return status;
  }

  // Set return mode
  int return_val = 1;  // Default Single
  if (config.return_mode == ReturnMode::DUAL) {
    return_val = 2;  // 2 Strongest
  } else if (config.return_mode == ReturnMode::DUAL_LAST_STRONGEST) {
    return_val = 3;  // 2 Strongest Furthest
  }
  status = set_attribute("return_mode", std::to_string(return_val));
  if (status != Status::OK) {
    return status;
  }

  // Set time sync
  status = set_attribute("time_sync", std::to_string(static_cast<int>(config.sync_mode)));
  if (status != Status::OK) {
    return status;
  }

  return Status::OK;
}

}  // namespace nebula::drivers
