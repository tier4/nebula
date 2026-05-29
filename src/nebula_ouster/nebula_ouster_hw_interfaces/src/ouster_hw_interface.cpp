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

#include "nebula_ouster_hw_interfaces/ouster_hw_interface.hpp"

#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{

OusterHwInterface::OusterHwInterface(ConnectionConfiguration connection_configuration)
: connection_configuration_(std::move(connection_configuration))
{
  // The ouster driver is UDP-only. Real integrations can initialize control/sync channels here.
}

util::expected<std::monostate, OusterHwInterface::Error> OusterHwInterface::sensor_interface_start()
{
  std::lock_guard<std::mutex> lock(callback_mutex_);

  if (!packet_callback_) {
    return Error{
      Error::Code::CALLBACK_NOT_REGISTERED,
      "Cannot start sensor interface before registering a packet callback"};
  }

  if (udp_socket_ && udp_socket_->is_subscribed()) {
    return std::monostate{};
  }

  try {
    connections::UdpSocket::Builder builder(
      connection_configuration_.host_ip, connection_configuration_.data_port);
    builder.set_mtu(static_cast<size_t>(connection_configuration_.receiver_mtu_bytes));

    if (connection_configuration_.filter_sender_ip) {
      builder.limit_to_sender(
        connection_configuration_.sensor_ip, connection_configuration_.data_port);
    }
    udp_socket_.emplace(std::move(builder).bind());

    // Callback can only be set while udp_socket_ is nullopt, so we don't need locking here
    udp_socket_->subscribe(
      [this](std::vector<uint8_t> & packet, const connections::UdpSocket::RxMetadata & metadata) {
        if (this->packet_callback_ && *this->packet_callback_) {
          (*this->packet_callback_)(packet, metadata);
        }
      });

    // Open an optional second socket for IMU packets when imu_port is configured. Each socket
    // has its own dedicated callback — no shared mutable state between lidar and IMU threads.
    const auto imu_port = connection_configuration_.imu_port;
    if (imu_port != 0 && imu_port != connection_configuration_.data_port && imu_callback_) {
      connections::UdpSocket::Builder imu_builder(
        connection_configuration_.host_ip, imu_port);
      imu_builder.set_mtu(128);  // IMU packets are fixed 48 bytes
      if (connection_configuration_.filter_sender_ip) {
        imu_builder.limit_to_sender(connection_configuration_.sensor_ip, imu_port);
      }
      imu_socket_.emplace(std::move(imu_builder).bind());
      imu_socket_->subscribe(
        [this](
          std::vector<uint8_t> & packet, const connections::UdpSocket::RxMetadata & metadata) {
          if (this->imu_callback_ && *this->imu_callback_) {
            (*this->imu_callback_)(packet, metadata);
          }
        });
    }
  } catch (const connections::SocketError & e) {
    return Error{
      Error::Code::SOCKET_OPEN_FAILED, std::string("Failed to open UDP socket: ") + e.what()};
  } catch (const connections::UsageError & e) {
    return Error{
      Error::Code::SOCKET_OPEN_FAILED,
      std::string("Invalid UDP socket configuration: ") + e.what()};
  } catch (const std::exception & e) {
    return Error{
      Error::Code::SOCKET_OPEN_FAILED,
      std::string("Failed to open UDP socket due to unexpected error: ") + e.what()};
  }

  return std::monostate{};
}

util::expected<std::monostate, OusterHwInterface::Error> OusterHwInterface::sensor_interface_stop()
{
  if (!udp_socket_) {
    return std::monostate{};
  }

  try {
    if (imu_socket_) {
      imu_socket_->unsubscribe();
      imu_socket_.reset();
    }
    udp_socket_->unsubscribe();
    udp_socket_.reset();
  } catch (const connections::SocketError & e) {
    return Error{
      Error::Code::SOCKET_CLOSE_FAILED, std::string("Failed to close UDP socket: ") + e.what()};
  } catch (const std::exception & e) {
    return Error{
      Error::Code::SOCKET_CLOSE_FAILED,
      std::string("Failed to close UDP socket due to unexpected error: ") + e.what()};
  }

  return std::monostate{};
}

util::expected<std::monostate, OusterHwInterface::Error> OusterHwInterface::register_scan_callback(
  connections::UdpSocket::callback_t scan_callback)
{
  if (!scan_callback) {
    return Error{Error::Code::INVALID_CALLBACK, "Cannot register an empty packet callback"};
  }

  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (udp_socket_) {
    return Error{
      Error::Code::INVALID_OPERATION,
      "Cannot replace packet callback while sensor interface is active"};
  }
  packet_callback_ = std::make_shared<connections::UdpSocket::callback_t>(std::move(scan_callback));
  return std::monostate{};
}

util::expected<std::monostate, OusterHwInterface::Error> OusterHwInterface::register_imu_callback(
  connections::UdpSocket::callback_t imu_callback)
{
  if (!imu_callback) {
    return Error{Error::Code::INVALID_CALLBACK, "Cannot register an empty IMU callback"};
  }

  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (imu_socket_) {
    return Error{
      Error::Code::INVALID_OPERATION,
      "Cannot replace IMU callback while sensor interface is active"};
  }
  imu_callback_ = std::make_shared<connections::UdpSocket::callback_t>(std::move(imu_callback));
  return std::monostate{};
}

const char * OusterHwInterface::to_cstr(OusterHwInterface::Error::Code code)
{
  switch (code) {
    case Error::Code::CALLBACK_NOT_REGISTERED:
      return "callback not registered";
    case Error::Code::INVALID_CALLBACK:
      return "invalid callback";
    case Error::Code::INVALID_OPERATION:
      return "invalid operation";
    case Error::Code::SOCKET_OPEN_FAILED:
      return "failed to open UDP socket";
    case Error::Code::SOCKET_CLOSE_FAILED:
      return "failed to close UDP socket";
    default:
      return "unknown hardware error";
  }
}

}  // namespace nebula::drivers
