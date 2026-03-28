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

#include "nebula_sample_hw_interfaces/sample_hw_interface.hpp"

#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{

SampleHwInterface::SampleHwInterface(ConnectionConfiguration connection_configuration)
: connection_configuration_(std::move(connection_configuration))
{
  // The sample driver is UDP-only. Real integrations can initialize control/sync channels here.
}

util::expected<std::monostate, SampleHwInterface::Error> SampleHwInterface::sensor_interface_start()
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
    udp_socket_.emplace(
      connections::UdpSocket::Builder(
        connection_configuration_.host_ip, connection_configuration_.data_port)
        .limit_to_sender(connection_configuration_.sensor_ip, connection_configuration_.data_port)
        .bind());

    // Callback can only be set while udp_socket_ is nullopt, so we don't need locking here
    udp_socket_->subscribe(
      [this](std::vector<uint8_t> & packet, const connections::UdpSocket::RxMetadata & metadata) {
        if (this->packet_callback_ && *this->packet_callback_) {
          (*this->packet_callback_)(packet, metadata);
        }
      });
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

util::expected<std::monostate, SampleHwInterface::Error> SampleHwInterface::sensor_interface_stop()
{
  if (!udp_socket_) {
    return std::monostate{};
  }

  try {
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

util::expected<std::monostate, SampleHwInterface::Error> SampleHwInterface::register_scan_callback(
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

const char * SampleHwInterface::to_cstr(SampleHwInterface::Error::Code code)
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
