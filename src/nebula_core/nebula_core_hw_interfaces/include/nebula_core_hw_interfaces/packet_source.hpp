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

#ifndef NEBULA_PACKET_SOURCE_HPP
#define NEBULA_PACKET_SOURCE_HPP

#include <nebula_core_common/sensor_packet.hpp>
#include <nebula_core_common/sensor_runtime_common.hpp>

#include <functional>
#include <memory>

namespace nebula::drivers
{
using SensorPacketCallback = std::function<void(const SensorPacket &)>;

class PacketSource
{
public:
  virtual ~PacketSource() = default;

  virtual void set_packet_callback(SensorPacketCallback callback) = 0;
  virtual void set_error_callback(SensorErrorCallback callback) = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual bool is_running() const = 0;
};

}  // namespace nebula::drivers

#endif  // NEBULA_PACKET_SOURCE_HPP
