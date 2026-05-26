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

#ifndef NEBULA_SENSOR_REQUIREMENTS_HPP
#define NEBULA_SENSOR_REQUIREMENTS_HPP

#include <nebula_core_common/sensor_packet.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace nebula::drivers
{
struct PayloadSignature
{
  size_t offset{0};
  std::vector<uint8_t> bytes;
  std::optional<std::vector<uint8_t>> mask;
};

struct PacketChannelRequirement
{
  SensorTransportKind transport{SensorTransportKind::UDP};
  SensorPacketChannel channel{SensorPacketChannel::Unknown};
  bool required{false};
  std::optional<uint16_t> destination_port;
  std::optional<uint32_t> can_id;
  std::optional<PayloadSignature> payload_signature;
};

struct LiveTransportRequirement
{
  SensorTransportKind transport{SensorTransportKind::UDP};
  SensorPacketChannel channel{SensorPacketChannel::Unknown};
  bool required{false};
  std::string name;
  std::optional<uint16_t> port;
  std::optional<std::string> http_path;
  std::optional<uint32_t> can_id;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SENSOR_REQUIREMENTS_HPP
