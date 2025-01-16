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

#pragma once

#include "nebula_decoders/nebula_decoders_common/angles.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_sensor.hpp"

namespace nebula::drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

struct Tail40P
{
  uint8_t reserved1[5];
  uint8_t high_temperature_shutdown_flag;
  uint8_t reserved2[2];
  uint16_t motor_speed;
  uint32_t timestamp;
  uint8_t return_mode;
  uint8_t factory_information;
  DateTime<2000> date_time;

  /* Ignored optional fields */

  // uint32_t udp_sequence;
};

struct Packet40P : public PacketBase<10, 40, 2, 100>
{
  using body_t = Body<SOBBlock<Unit3B, Packet40P::n_channels>, Packet40P::n_blocks>;
  body_t body;
  Tail40P tail;
};

#pragma pack(pop)

/// @brief Get the distance unit of the given @ref Packet40P packet in meters. This is the only
/// packet type without a header.
/// @return 0.004 (4mm)
template <>
inline double get_dis_unit<Packet40P>(const Packet40P & /* packet */)
{
  return 4 / 1000.;
}

}  // namespace hesai_packet

class Pandar40 : public HesaiSensor<hesai_packet::Packet40P>
{
private:
  static constexpr int firing_time_offset_ns_[40] = {
    -42220, -28470, -16040, -3620,  -45490, -31740, -47460, -54670, -20620, -33710,
    -40910, -8190,  -20620, -27160, -50730, -8190,  -14740, -36980, -45490, -52700,
    -23890, -31740, -38950, -11470, -18650, -25190, -48760, -6230,  -12770, -35010,
    -21920, -9500,  -43520, -29770, -17350, -4920,  -42220, -28470, -16040, -3620};

public:
  static constexpr float min_range = 0.3f;
  static constexpr float max_range = 200.f;
  static constexpr size_t max_scan_buffer_points = 144000;
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{{0, 360'000}, {-25'000, 15'000}};
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{200, 334};

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns = -28580 - 55560 * ((10 - block_id - 1) / n_returns);
    return block_offset_ns + firing_time_offset_ns_[channel_id];
  }
};

}  // namespace nebula::drivers
