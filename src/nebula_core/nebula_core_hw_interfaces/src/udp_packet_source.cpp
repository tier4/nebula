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

#include <nebula_core_hw_interfaces/udp_packet_source.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{

UdpPacketSource::UdpPacketSource()
{
}

UdpPacketSource::~UdpPacketSource()
{
  stop();
}

void UdpPacketSource::configure(const std::string & host_ip, uint16_t port)
{
  host_ip_ = host_ip;
  port_ = port;
}

void UdpPacketSource::set_packet_callback(SensorPacketCallback callback)
{
  callback_ = callback;
}

void UdpPacketSource::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
}

void UdpPacketSource::start()
{
  if (socket_) return;

  try {
    socket_ = std::make_unique<connections::UdpSocket>(
      connections::UdpSocket::Builder(host_ip_, port_).bind());

    socket_->subscribe(
      std::bind(
        &UdpPacketSource::on_udp_packet, this, std::placeholders::_1, std::placeholders::_2));
  } catch (const std::exception & e) {
    // Reset before rethrowing so is_running() reflects the failed start and a
    // subsequent start() does not see a populated socket_ with no receive thread.
    socket_.reset();
    if (error_callback_) {
      SensorError error;
      error.type = SensorErrorType::TransportError;
      error.message = e.what();
      error_callback_(error);
    }
    throw;
  }
}

void UdpPacketSource::stop()
{
  if (socket_) {
    socket_->unsubscribe();
    socket_.reset();
  }
}

bool UdpPacketSource::is_running() const
{
  return static_cast<bool>(socket_);
}

void UdpPacketSource::on_udp_packet(
  std::vector<uint8_t> & data, const connections::UdpSocket::RxMetadata & metadata)
{
  if (callback_) {
    SensorPacket sp;
    sp.transport = SensorTransportKind::UDP;

    if (metadata.timestamp_ns.has_value()) {
      sp.timestamp_ns = *metadata.timestamp_ns;
    } else {
      sp.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
    }

    sp.destination = SensorEndpoint{host_ip_, port_};

    sp.payload = std::move(data);

    callback_(sp);
  }
}

}  // namespace nebula::drivers
