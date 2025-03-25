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

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_sensor.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_40.hpp"

namespace nebula::drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

using Tail64 = Tail40P;
struct Packet64 : public PacketBase<6, 64, 2, 100>
{
  using body_t = Body<Block<Unit3B, Packet64::n_channels>, Packet64::n_blocks>;
  Header8B header;
  body_t body;
  Tail64 tail;
};

#pragma pack(pop)

}  // namespace hesai_packet

class Pandar64 : public HesaiSensor<hesai_packet::Packet64>
{
private:
  static constexpr int firing_time_offset_ns[64] = {
    -23180, -21876, -20572, -19268, -17964, -16660, -11444, -46796, -7532,  -36956, -50732,
    -54668, -40892, -44828, -31052, -34988, -48764, -52700, -38924, -42860, -29084, -33020,
    -46796, -25148, -36956, -50732, -27116, -40892, -44828, -31052, -34988, -48764, -25148,
    -38924, -42860, -29084, -33020, -52700, -6228,  -54668, -15356, -27116, -10140, -23180,
    -4924,  -21876, -14052, -17964, -8836,  -19268, -3620,  -20572, -12748, -16660, -7532,
    -11444, -6228,  -15356, -10140, -4924,  -3620,  -14052, -8836,  -12748};

public:
  static constexpr float min_range = 0.3f;
  static constexpr float max_range = 200.f;
  static constexpr size_t max_scan_buffer_points = 230400;
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{{0, 360'000}, {-25'000, 15'000}};
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{200, 167};

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns = -42580 - 55560 * ((6 - block_id - 1) / n_returns);
    return block_offset_ns + firing_time_offset_ns[channel_id];
  }
};

}  // namespace nebula::drivers
