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

#include "packet_source_utils.hpp"

#include <nebula_core_hw_interfaces/can_packet_source.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

namespace nebula::drivers
{

CanPacketSource::CanPacketSource()
{
}

CanPacketSource::~CanPacketSource()
{
  stop();
}

void CanPacketSource::configure(const std::string & interface_name)
{
  interface_name_ = interface_name;
}

void CanPacketSource::set_packet_callback(SensorPacketCallback callback)
{
  callback_ = callback;
}

void CanPacketSource::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
}

void CanPacketSource::start()
{
  if (socket_) return;

  try {
    socket_ = std::make_unique<connections::CanSocket>(
      connections::CanSocket::Builder(interface_name_).bind());

    socket_->subscribe(
      std::bind(
        &CanPacketSource::on_can_frame, this, std::placeholders::_1, std::placeholders::_2));
  } catch (const std::exception & e) {
    // Reset before rethrowing so is_running() reflects the failed start and a
    // subsequent start() does not see a populated socket_ with no receive thread.
    socket_.reset();
    report_transport_error(error_callback_, SensorErrorType::TransportError, e.what());
    throw;
  }
}

void CanPacketSource::stop()
{
  if (socket_) {
    socket_->unsubscribe();
    socket_.reset();
  }
}

bool CanPacketSource::is_running() const
{
  return static_cast<bool>(socket_);
}

void CanPacketSource::on_can_frame(
  const canfd_frame & frame, const connections::CanSocket::RxMetadata & metadata)
{
  if (callback_) {
    SensorPacket sp;
    sp.transport = SensorTransportKind::CAN;

    if (metadata.timestamp_ns.has_value()) {
      sp.timestamp_ns = *metadata.timestamp_ns;
    } else {
      sp.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
    }

    SensorCanMetadata can_md;
    can_md.set_interface_name(interface_name_);
    can_md.can_id = frame.can_id & CAN_EFF_MASK;
    can_md.is_extended_id = (frame.can_id & CAN_EFF_FLAG) != 0;
    const uint8_t len = std::min<uint8_t>(frame.len, CANFD_MAX_DLEN);
    can_md.dlc = len;
    sp.can = can_md;

    sp.payload.assign(frame.data, frame.data + len);

    invoke_packet_callback("CanPacketSource", callback_, sp);
  }
}

}  // namespace nebula::drivers
