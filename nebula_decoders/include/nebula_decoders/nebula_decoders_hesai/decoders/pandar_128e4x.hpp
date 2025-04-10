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
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_128e3x.hpp"

#include <iostream>
#include <vector>

namespace nebula::drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

using Packet128E4X = Packet128E3X;

#pragma pack(pop)

}  // namespace hesai_packet

// FIXME(mojomex):
// The OT128 datasheet has entirely different numbers (and more azimuth states).
// With the current sensor version, the numbers from the new datasheet are incorrect
// (clouds do not sync to ToS but ToS+.052s)
class Pandar128E4X : public HesaiSensor<hesai_packet::Packet128E4X>
{
private:
  enum OperationalState { HIGH_RESOLUTION = 0, STANDARD = 1 };

  static constexpr int firing_time_offset_static_ns[128] = {
    49758, 43224, 36690, 30156, 21980, 15446, 8912,  2378,  49758, 43224, 36690, 30156, 2378,
    15446, 8912,  21980, 43224, 30156, 49758, 15446, 36690, 2378,  21980, 8912,  34312, 45002,
    38468, 40846, 40846, 34312, 51536, 47380, 31934, 47380, 31934, 51536, 38468, 27778, 27778,
    45002, 38468, 40846, 51536, 45002, 31934, 47380, 34312, 27778, 38468, 40846, 51536, 45002,
    31934, 34312, 27778, 38468, 40846, 51536, 45002, 31934, 47380, 34312, 27778, 45002, 45002,
    51536, 38468, 40846, 47380, 40846, 31934, 45002, 27778, 38468, 40846, 31934, 27778, 38468,
    34312, 34312, 34312, 47380, 51536, 31934, 51536, 47380, 27778, 49758, 21980, 43224, 15446,
    43224, 36690, 8912,  30156, 2378,  49758, 21980, 36690, 15446, 8912,  30156, 2378,  49758,
    43224, 36690, 30156, 21980, 15446, 8912,  2378,  43224, 49758, 30156, 36690, 21980, 15446,
    2378,  8912,  49758, 43224, 36690, 30156, 21980, 15446, 8912,  2378,  30156};

  static constexpr int firing_time_offset_as0_ns[128] = {
    -1,    -1,    -1,    -1,    21980, 15446, 8912,  2378,  -1,    -1,    -1,    -1,    2378,
    15446, 8912,  21980, -1,    2378,  21980, 8912,  6534,  17224, 10690, 13068, 13068, 6534,
    23758, 19602, 4156,  19602, 4156,  23758, 13068, 13068, 23758, 10690, 4156,  19602, 19602,
    0,     10690, 6534,  23758, 17224, 23758, 19602, 6534,  17224, 4156,  0,     6534,  0,
    17224, 10690, 17224, 17224, 23758, 23758, 10690, 13068, 13068, 13068, 19602, 19602, 6534,
    13068, 4156,  4156,  17224, 17224, 17224, 0,     10690, 10690, 13068, 13068, 4156,  0,
    10690, 6534,  6534,  6534,  19602, 23758, 4156,  23758, 19602, 0,     -1,    21980, -1,
    15446, -1,    8912,  -1,    2378,  -1,    -1,    21980, -1,    15446, 8912,  -1,    2378,
    -1,    -1,    -1,    -1,    21980, 15446, 8912,  2378,  -1,    -1,    -1,    -1,    21980,
    15446, 2378,  8912,  -1,    -1,    -1,    -1,    21980, 15446, 8912,  2378};

  static constexpr int firing_time_offset_as1_ns[128] = {
    21980, 15446, 8912,  2378,  -1,    -1,    -1,    -1,    21980, 15446, 8912,  2378,  -1,
    -1,    -1,    -1,    8912,  -1,    -1,    -1,    6534,  17224, 10690, 13068, 13068, 6534,
    23758, 19602, 4156,  19602, 4156,  23758, 13068, 13068, 23758, 10690, 4156,  19602, 19602,
    0,     10690, 6534,  23758, 17224, 23758, 19602, 6534,  17224, 4156,  0,     6534,  0,
    17224, 10690, 17224, 17224, 23758, 23758, 10690, 13068, 13068, 13068, 19602, 19602, 6534,
    13068, 4156,  4156,  17224, 17224, 17224, 0,     10690, 10690, 13068, 13068, 4156,  0,
    10690, 6534,  6534,  6534,  19602, 23758, 4156,  23758, 19602, 0,     21980, -1,    15446,
    -1,    8912,  -1,    2378,  -1,    21980, 15446, -1,    8912,  -1,    -1,    2378,  -1,
    21980, 15446, 8912,  2378,  -1,    -1,    -1,    -1,    15446, 21980, 2378,  8912,  -1,
    -1,    -1,    -1,    21980, 15446, 8912,  2378,  -1,    -1,    -1,    -1};

public:
  static constexpr float min_range = 0.1;
  static constexpr float max_range = 230.0;
  static constexpr size_t max_scan_buffer_points = 691200;
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{{0, 360'000}, {-24'800, 14'400}};
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{100, 125};

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns = 0;
    if (n_returns == 1) {
      block_offset_ns = -27778 * 2 * (2 - block_id - 1);
    } else {
      block_offset_ns = 0;
    }

    int channel_offset_ns = 0;
    bool is_hires_mode = packet.tail.operational_state == OperationalState::HIGH_RESOLUTION;
    auto azimuth_state = packet.tail.get_azimuth_state(block_id);

    if (!is_hires_mode) {
      channel_offset_ns = firing_time_offset_static_ns[channel_id];
    } else {
      if (azimuth_state == 0) {
        channel_offset_ns = firing_time_offset_as0_ns[channel_id];
      } else /* azimuth_state == 1 */ {
        channel_offset_ns = firing_time_offset_as1_ns[channel_id];
      }
    }

    return block_offset_ns + 43346 + channel_offset_ns;
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

  [[nodiscard]] point_filters::DitherTransform get_dither_transform() const override
  {
    return [](size_t x, size_t y) {
      // Dithering in hi-res mode is done in groups of 4 channels, with the first and second
      // pair being alternated per block (cycle length 2) like this (* is active, . is inactive):
      // channel |  block
      //         | 1 2 3 4
      // --------+--------
      //     y+0 | * . * .
      //     y+1 | * . * .
      //     y+2 | . * . *
      //     y+3 | . * . *

      const size_t pair_n_channels = 2;
      const size_t group_n_channels = 4;
      const size_t cycle_n_blocks = 2;

      // The dithering pattern is 2 blocks wide and 4 channels tall, quantize the positions in
      // each 2x4 tile to the same position (x/2*2, y/4*4). This eliminates flicker that would
      // otherwise happen.
      size_t x_quant = (x / cycle_n_blocks * cycle_n_blocks);
      size_t y_quant = (y / group_n_channels * group_n_channels);

      // In each channel pair, the normal 45deg dithering offset can be applied normally
      // as the relative position within each pair remains constant, regardless of the dithering
      // cycle.
      size_t y_offset = (y % pair_n_channels);

      return x_quant + y_quant + y_offset;
    };
  }
};

}  // namespace nebula::drivers
