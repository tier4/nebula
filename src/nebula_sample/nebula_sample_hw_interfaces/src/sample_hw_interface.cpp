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

SampleHwInterface::SampleHwInterface(SampleSensorConfiguration sensor_configuration)
: sensor_configuration_(std::move(sensor_configuration))
{
  // Constructor - initialize any member variables if needed
}

util::expected<std::monostate, SampleHwInterface::Error> SampleHwInterface::sensor_interface_start()
{
  if (!packet_callback_) {
    return Error::CALLBACK_NOT_REGISTERED;
  }

  // Implementation Items: Implement sensor interface startup
  // 1. Create UDP socket using connections::UdpSocket
  // 2. Bind to the port specified in sensor_configuration_
  // 3. Start async receive loop
  // 4. When packets arrive, call packet_callback_ with the packet data
  // 5. Optionally: send HTTP/TCP command to sensor to start scanning

  return std::monostate{};
}

util::expected<std::monostate, SampleHwInterface::Error> SampleHwInterface::sensor_interface_stop()
{
  // Implementation Items: Implement sensor interface shutdown
  // 1. Stop the receive loop
  // 3. Optionally: send command to sensor to stop scanning

  return std::monostate{};
}

util::expected<std::monostate, SampleHwInterface::Error> SampleHwInterface::register_scan_callback(
  connections::UdpSocket::callback_t scan_callback)
{
  if (!scan_callback) {
    return Error::INVALID_CALLBACK;
  }
  packet_callback_.emplace(std::move(scan_callback));
  return std::monostate{};
}

}  // namespace nebula::drivers
