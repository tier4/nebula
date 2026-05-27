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

#include <nebula_core_runtime/packet_router.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

namespace nebula::drivers
{

void PacketRouter::configure(const std::vector<PacketChannelRequirement> & requirements)
{
  udp_entries_.clear();
  can_entries_.clear();

  for (const auto & req : requirements) {
    if (
      (req.transport == SensorTransportKind::UDP || req.transport == SensorTransportKind::TCP) &&
      req.destination_port.has_value()) {
      udp_entries_.push_back({*req.destination_port, req});
    } else if (req.transport == SensorTransportKind::CAN && req.can_id.has_value()) {
      can_entries_.push_back({*req.can_id, req});
    } else if (
      (req.transport == SensorTransportKind::UDP || req.transport == SensorTransportKind::TCP) &&
      !req.destination_port.has_value()) {
      if (req.required) {
        throw std::invalid_argument(
          "Required UDP/TCP packet channel requirement is missing destination_port");
      }
    } else if (req.transport == SensorTransportKind::CAN && !req.can_id.has_value()) {
      if (req.required) {
        throw std::invalid_argument("Required CAN packet channel requirement is missing can_id");
      }
    } else if (
      req.transport != SensorTransportKind::UDP && req.transport != SensorTransportKind::TCP &&
      req.transport != SensorTransportKind::CAN) {
      // HTTP and other transports are not routed through PacketRouter;
      // they are handled as control endpoints in LiveTransportGraph.
      std::cerr << "PacketRouter: ignoring unsupported transport requirement"
                << " (not UDP/TCP/CAN)" << std::endl;
    }
  }

  std::sort(udp_entries_.begin(), udp_entries_.end(), [](const UdpEntry & a, const UdpEntry & b) {
    return a.port < b.port;
  });
  std::sort(can_entries_.begin(), can_entries_.end(), [](const CanEntry & a, const CanEntry & b) {
    return a.can_id < b.can_id;
  });
}

bool PacketRouter::route(SensorPacketView & packet) noexcept
{
  metrics_.processed_packets++;

  if (
    (packet.transport == SensorTransportKind::UDP ||
     packet.transport == SensorTransportKind::TCP) &&
    packet.destination) {
    const uint16_t port = packet.destination->port;
    auto lo = std::lower_bound(
      udp_entries_.begin(), udp_entries_.end(), port,
      [](const UdpEntry & e, uint16_t p) { return e.port < p; });
    for (auto it = lo; it != udp_entries_.end() && it->port == port; ++it) {
      if (
        it->req.transport == packet.transport &&
        (!it->req.payload_signature.has_value() ||
         match_signature(packet.payload_data, packet.payload_size, *it->req.payload_signature))) {
        packet.channel = it->req.channel;
        metrics_.matched_packets++;
        return true;
      }
    }
  } else if (packet.transport == SensorTransportKind::CAN && packet.can) {
    const uint32_t can_id = packet.can->can_id;
    auto lo = std::lower_bound(
      can_entries_.begin(), can_entries_.end(), can_id,
      [](const CanEntry & e, uint32_t id) { return e.can_id < id; });
    for (auto it = lo; it != can_entries_.end() && it->can_id == can_id; ++it) {
      if (
        !it->req.payload_signature.has_value() ||
        match_signature(packet.payload_data, packet.payload_size, *it->req.payload_signature)) {
        packet.channel = it->req.channel;
        metrics_.matched_packets++;
        return true;
      }
    }
  }

  metrics_.dropped_packets++;
  return false;
}

const SensorProgress & PacketRouter::get_metrics() const
{
  return metrics_;
}

bool PacketRouter::match_signature(
  const uint8_t * data, size_t size, const PayloadSignature & signature) const noexcept
{
  if (signature.bytes.empty()) return true;
  if (signature.mask.has_value() && signature.mask->size() != signature.bytes.size()) return false;
  // Guard 1 must precede guard 2: if offset >= size then (size - offset) wraps for unsigned types.
  if (signature.offset >= size) return false;
  if (signature.bytes.size() > size - signature.offset) return false;

  for (size_t i = 0; i < signature.bytes.size(); ++i) {
    const uint8_t payload_byte = data[signature.offset + i];
    const uint8_t expected = signature.bytes[i];
    if (signature.mask.has_value()) {
      const uint8_t mask = (*signature.mask)[i];
      if ((payload_byte & mask) != (expected & mask)) return false;
    } else if (payload_byte != expected) {
      return false;
    }
  }
  return true;
}

}  // namespace nebula::drivers
