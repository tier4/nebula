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

namespace nebula::drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

struct TailAT128E2X
{
  uint8_t reserved1[6];
  uint8_t high_temperature_shutdown_flag;
  uint8_t reserved2[11];
  uint16_t motor_speed;
  uint32_t timestamp;
  uint8_t return_mode;
  uint8_t factory_information;
  SecondsSinceEpoch date_time;

  /* Ignored optional fields */

  // uint32_t udp_sequence;
  // uint32_t crc_tail;
};

struct PacketAT128E2X : public PacketBase<2, 128, 2, 100 * 256>
{
  using body_t =
    Body<FineAzimuthBlock<Unit4B, PacketAT128E2X::n_channels>, PacketAT128E2X::n_blocks>;
  Header12B header;
  body_t body;
  uint32_t crc_body;
  TailAT128E2X tail;

  /* Ignored optional fields */

  // uint8_t cyber_security[32];
};

#pragma pack(pop)

}  // namespace hesai_packet

class PandarAT128
: public HesaiSensor<hesai_packet::PacketAT128E2X, AngleCorrectionType::CORRECTION>
{
private:
  static constexpr int firing_time_offset_ns[128] = {
    0,     0,     8240,  4112,  4144,  8240,  0,     0,     12424, 4144,  4112,  8264,  12376,
    12376, 8264,  12424, 0,     0,     4112,  8240,  4144,  0,     0,     4144,  12424, 8264,
    4112,  12376, 12376, 12424, 8264,  848,   2504,  4976,  6616,  6616,  9112,  2504,  848,
    10768, 13280, 13280, 4976,  9112,  14928, 14928, 10768, 2504,  848,   6616,  4976,  9112,
    6616,  848,   2504,  13280, 10768, 4976,  13280, 14928, 9112,  10768, 14928, 13280, 848,
    9112,  13280, 2504,  4976,  848,   2504,  14928, 10768, 10768, 14928, 4976,  6616,  6616,
    9112,  848,   13280, 13280, 9112,  4976,  2504,  2504,  848,   10768, 14928, 14928, 10768,
    6616,  4976,  9112,  6616,  4112,  12424, 0,     4144,  0,     0,     12424, 0,     8264,
    4112,  4144,  8240,  8240,  8264,  12376, 12376, 12424, 4112,  4144,  0,     0,     0,
    0,     0,     12424, 8264,  8240,  4144,  8264,  8240,  12376, 12376, 8264};

public:
  static constexpr float min_range = 1.f;
  static constexpr float max_range = 180.0f;
  static constexpr size_t max_scan_buffer_points = 307200;
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{
    {30'000, 150'000}, {-12'500, 12'900}};
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{100, 200};

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns = 0;
    if (n_returns == 1) {
      block_offset_ns = -9249 - 41666 * (2 - block_id);
    } else {
      block_offset_ns = -9249 - 41666;
    }

    return block_offset_ns + firing_time_offset_ns[channel_id];
  }
};

}  // namespace nebula::drivers
