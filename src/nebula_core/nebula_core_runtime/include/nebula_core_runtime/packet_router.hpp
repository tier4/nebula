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

#ifndef NEBULA_PACKET_ROUTER_HPP
#define NEBULA_PACKET_ROUTER_HPP

#include <nebula_core_common/sensor_packet.hpp>
#include <nebula_core_common/sensor_runtime_common.hpp>
#include <nebula_core_decoders/sensor_requirements.hpp>

#include <vector>

namespace nebula::drivers
{
class PacketRouter
{
public:
  PacketRouter() = default;

  // Build lookup tables from the plugin's channel requirements. Must be called
  // before route(); entries are sorted so route() is allocation-free.
  void configure(const std::vector<PacketChannelRequirement> & requirements);

  // Assign the channel field of the view based on transport, port, and CAN ID.
  // Returns true if the packet matched a requirement.
  //
  // Thread-safety: route() mutates non-atomic metrics_, so it is safe to call
  // only from a single RT thread once configure() has returned. Concurrent
  // route() calls from multiple threads, or a concurrent get_metrics() against
  // an in-flight route(), require external serialization. LiveTransportGraph
  // provides this serialization via processing_mutex_.
  bool route(SensorPacketView & packet) noexcept;

  // Returns a reference to the live metrics. Not thread-safe against in-flight
  // route() calls; see route() above for the synchronization contract.
  const SensorProgress & get_metrics() const;

private:
  struct UdpEntry
  {
    uint16_t port{0};
    PacketChannelRequirement req;
  };

  struct CanEntry
  {
    uint32_t can_id{0};
    PacketChannelRequirement req;
  };

  // Both vectors are sorted by key after configure() and never modified during route().
  std::vector<UdpEntry> udp_entries_;
  std::vector<CanEntry> can_entries_;
  SensorProgress metrics_;

  bool match_signature(
    const uint8_t * data, size_t size, const PayloadSignature & signature) const noexcept;
};

}  // namespace nebula::drivers

#endif  // NEBULA_PACKET_ROUTER_HPP
