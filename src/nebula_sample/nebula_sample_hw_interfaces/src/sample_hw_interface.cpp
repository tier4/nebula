// Copyright 2025 TIER IV, Inc.
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

#include <utility>

namespace nebula::drivers
{

SampleHwInterface::SampleHwInterface(ConnectionConfiguration connection_configuration)
: connection_configuration_(std::move(connection_configuration))
{
  // Implement: Initialize non-pointcloud hardware connections, and sync/check sensor settings
}

util::expected<std::monostate, SampleHwInterface::Error> SampleHwInterface::sensor_interface_start()
{
  if (!packet_callback_) {
    return Error::CALLBACK_NOT_REGISTERED;
  }

  // Implement: Create/bind UDP receiver and forward incoming packets to packet_callback_.
  return std::monostate{};
}

util::expected<std::monostate, SampleHwInterface::Error> SampleHwInterface::sensor_interface_stop()
{
  // Implement: Stop receiver threads and release network resources (RAII preferred).
  return std::monostate{};
}

util::expected<std::monostate, SampleHwInterface::Error> SampleHwInterface::register_scan_callback(
  connections::UdpSocket::callback_t scan_callback)
{
  if (!scan_callback) {
    return Error::INVALID_CALLBACK;
  }

  // Implement: Depending on threading model, may need synchronization around packet_callback_
  // access.
  packet_callback_.emplace(std::move(scan_callback));
  return std::monostate{};
}

const char * SampleHwInterface::to_cstr(SampleHwInterface::Error error)
{
  switch (error) {
    case Error::CALLBACK_NOT_REGISTERED:
      return "callback not registered";
    case Error::INVALID_CALLBACK:
      return "invalid callback";
    default:
      return "unknown hardware error";
  }
}

}  // namespace nebula::drivers
