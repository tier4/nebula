// Copyright 2024 TIER IV, Inc.
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

#ifndef NEBULA_SENSOR_PACKET_HPP
#define NEBULA_SENSOR_PACKET_HPP

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace nebula::drivers
{
enum class SensorTransportKind {
  UDP,
  TCP,
  HTTP,
  CAN,
};

enum class SensorPacketChannel {
  Data,
  Info,
  Status,
  Control,
  Correction,
  Radar,
  Unknown,
};

struct SensorEndpoint
{
  std::string address;
  uint16_t port = 0;
};

struct SensorCanMetadata
{
  std::string interface_name;
  uint32_t can_id = 0;
  bool is_extended_id = false;
  uint8_t dlc = 0;
};

struct SensorPacket
{
  SensorTransportKind transport{SensorTransportKind::UDP};
  SensorPacketChannel channel{SensorPacketChannel::Unknown};
  uint64_t timestamp_ns{0};
  bool from_replay{false};
  std::optional<SensorEndpoint> source;
  std::optional<SensorEndpoint> destination;
  std::optional<SensorCanMetadata> can;
  std::vector<uint8_t> payload;
};

struct NebulaPacket
{
  struct
  {
    uint32_t sec;
    uint32_t nanosec;
  } stamp;
  std::vector<uint8_t> data;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SENSOR_PACKET_HPP
