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
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xt32.hpp"

namespace nebula::drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

using TailXT32M2X = TailXT32;
struct PacketXT32M2X : public PacketBase<6, 32, 3, 100>
{
  using body_t = Body<Block<Unit4B, PacketXT32M2X::n_channels>, PacketXT32M2X::n_blocks>;
  Header12B header;
  body_t body;
  TailXT32M2X tail;
  uint32_t udp_sequence;
};

#pragma pack(pop)

}  // namespace hesai_packet

class PandarXT32M : public HesaiSensor<hesai_packet::PacketXT32M2X>
{
public:
  static constexpr float min_range = 0.5f;
  static constexpr float max_range = 300.0f;
  static constexpr size_t max_scan_buffer_points = 384000;
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{{0, 360'000}, {-20'800, 19'500}};
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{180, 1'300};

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns = 0;
    if (n_returns < 3) {
      block_offset_ns = 5632 - 50000 * ((8 - block_id - 1) / n_returns);
    } else /* n_returns == 3 */ {
      block_offset_ns = 5632 - 50000 * ((6 - block_id - 1) / 3);
    }

    if (channel_id >= 16) {
      channel_id -= 16;
    }
    int channel_offset_ns = 368 + 2888 * channel_id;

    return block_offset_ns + channel_offset_ns;
  }
};

}  // namespace nebula::drivers
