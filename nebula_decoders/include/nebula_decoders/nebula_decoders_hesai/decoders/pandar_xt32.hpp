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

#include <vector>

namespace nebula::drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

struct TailXT32
{
  uint8_t reserved1[10];
  uint8_t return_mode;
  uint16_t motor_speed;
  DateTime<1900> date_time;
  uint32_t timestamp;
  uint8_t factory_information;
};

struct PacketXT32 : public PacketBase<8, 32, 2, 100>
{
  using body_t = Body<Block<Unit4B, PacketXT32::n_channels>, PacketXT32::n_blocks>;
  Header12B header;
  body_t body;
  TailXT32 tail;
  uint32_t udp_sequence;
};

#pragma pack(pop)

}  // namespace hesai_packet

class PandarXT32 : public HesaiSensor<hesai_packet::PacketXT32>
{
public:
  static constexpr float min_range = 0.05f;
  static constexpr float max_range = 120.0f;
  static constexpr size_t max_scan_buffer_points = 256000;
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{{0, 360'000}, {-16'000, 15'000}};
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{180, 1'000};

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns = 5632 - 50000 * ((8 - block_id - 1) / n_returns);
    int channel_offset_ns = 368 + 1512 * channel_id;
    return block_offset_ns + channel_offset_ns;
  }

  ReturnType get_return_type(
    hesai_packet::return_mode::ReturnMode return_mode, unsigned int return_idx,
    const std::vector<const typename packet_t::body_t::block_t::unit_t *> & return_units) override
  {
    auto return_type =
      HesaiSensor<packet_t>::get_return_type(return_mode, return_idx, return_units);
    if (return_type == ReturnType::IDENTICAL) {
      return return_type;
    }

    // This sensor orders returns in the opposite order, so the return_type needs to be flipped too
    if (return_mode == hesai_packet::return_mode::DUAL_FIRST_LAST) {
      if (return_type == ReturnType::FIRST)
        return_type = ReturnType::LAST;
      else if (return_type == ReturnType::LAST)
        return_type = ReturnType::FIRST;
    }

    return return_type;
  }
};

}  // namespace nebula::drivers
