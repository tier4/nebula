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

#ifndef NEBULA_SENSOR_PACKET_HPP
#define NEBULA_SENSOR_PACKET_HPP

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string_view>
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

// Fixed-size address avoids heap allocation on the packet hot path.
// Holds up to 45 characters (max IPv6 length) plus a null terminator.
// Addresses longer than 45 characters are a programmer error and trigger an assertion.
struct SensorEndpoint
{
  std::array<char, 46> address{};
  uint16_t port{0};

  SensorEndpoint() = default;
  SensorEndpoint(std::string_view addr, uint16_t p) : port(p)
  {
    assert(addr.size() < address.size() && "SensorEndpoint: address exceeds 45-char IPv6 maximum");
    auto n = addr.size() < address.size() - 1 ? addr.size() : address.size() - 1;
    std::memcpy(address.data(), addr.data(), n);
    address[n] = '\0';
  }

  std::string_view address_view() const { return address.data(); }
};

// Fixed-size interface name matches Linux IFNAMSIZ (16 bytes including null).
struct SensorCanMetadata
{
  std::array<char, 16> interface_name{};
  uint32_t can_id{0};
  bool is_extended_id{false};
  uint8_t dlc{0};

  void set_interface_name(std::string_view name)
  {
    auto n = name.size() < interface_name.size() - 1 ? name.size() : interface_name.size() - 1;
    std::memcpy(interface_name.data(), name.data(), n);
    interface_name[n] = '\0';
  }

  std::string_view interface_name_view() const { return interface_name.data(); }
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

// Non-owning view into a SensorPacket used on the decoder hot path.
// Avoids copying the payload vector. Lifetime is bounded by the originating
// SensorPacket — the packet must remain alive for the duration of any call
// that receives this view.
struct SensorPacketView
{
  SensorTransportKind transport{SensorTransportKind::UDP};
  SensorPacketChannel channel{SensorPacketChannel::Unknown};  // written by PacketRouter
  uint64_t timestamp_ns{0};
  bool from_replay{false};
  const SensorEndpoint * source{nullptr};
  const SensorEndpoint * destination{nullptr};
  const SensorCanMetadata * can{nullptr};
  const uint8_t * payload_data{nullptr};
  size_t payload_size{0};

  static SensorPacketView from(const SensorPacket & p) noexcept
  {
    SensorPacketView v;
    v.transport = p.transport;
    v.channel = p.channel;
    v.timestamp_ns = p.timestamp_ns;
    v.from_replay = p.from_replay;
    v.source = p.source.has_value() ? &*p.source : nullptr;
    v.destination = p.destination.has_value() ? &*p.destination : nullptr;
    v.can = p.can.has_value() ? &*p.can : nullptr;
    v.payload_data = p.payload.empty() ? nullptr : p.payload.data();
    v.payload_size = p.payload.size();
    return v;
  }

  // Prevent views from being created from temporaries: the view holds raw
  // pointers into the packet, so the packet must outlive the view.
  static SensorPacketView from(SensorPacket &&) = delete;
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
