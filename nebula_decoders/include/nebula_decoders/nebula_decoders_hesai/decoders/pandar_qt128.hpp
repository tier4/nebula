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

struct TailQT128C2X
{
  uint8_t reserved1[5];
  uint8_t mode_flag;
  uint8_t reserved2[6];
  uint8_t return_mode;
  uint16_t motor_speed;
  DateTime<1900> date_time;
  uint32_t timestamp;
  uint8_t factory_information;

  /* Ignored optional fields */

  // uint32_t udp_sequence;
  // uint32_t crc_tail;
};

struct PacketQT128C2X : public PacketBase<2, 128, 2, 100>
{
  using body_t = Body<Block<Unit4B, PacketQT128C2X::n_channels>, PacketQT128C2X::n_blocks>;
  Header12B header;
  body_t body;
  uint32_t crc_body;
  FunctionalSafety fs;
  TailQT128C2X tail;

  /* Ignored optional fields */

  // uint8_t cyber_security[32];
};

#pragma pack(pop)

}  // namespace hesai_packet

class PandarQT128 : public HesaiSensor<hesai_packet::PacketQT128C2X>
{
private:
  // Channels 0-31 (starting at 0) do not fire, delay set to 0
  static constexpr int loop1[128] = {
    0,     0,     0,     0,     0,     0,     0,     0,     0,      0,     0,     0,     0,
    0,     0,     0,     0,     0,     0,     0,     0,     0,      0,     0,     0,     0,
    0,     0,     0,     0,     0,     0,     27656, 53000, 2312,   78344, 81512, 5480,  56168,
    30824, 33992, 59336, 8648,  84680, 87848, 11816, 62504, 37160,  40328, 65672, 14984, 91016,
    94184, 18152, 68840, 43496, 46664, 72008, 21320, 97352, 100520, 24488, 75176, 49832, 1456,
    77488, 26800, 52144, 55312, 29968, 80656, 4624,  7792,  83824,  33136, 58480, 61648, 36304,
    86992, 10960, 14128, 90160, 39472, 64816, 67984, 42640, 93328,  17296, 20464, 96496, 45808,
    71152, 74320, 48976, 99664, 23632, 25944, 51288, 600,   76632,  79800, 3768,  54456, 29112,
    32280, 57624, 6936,  82968, 86136, 10104, 60792, 35448, 38616,  63960, 13272, 89304, 92472,
    16440, 67128, 41784, 44952, 70296, 19608, 95640, 98808, 22776,  73464, 48120};

  // Channels 32-63 (starting at 0) do not fire, delay set to 0
  static constexpr int loop2[128] = {
    2312,  78344, 27656, 53000, 56168,  30824, 81512, 5480,  8648,  84680, 33992, 59336, 62504,
    37160, 87848, 11816, 14984, 91016,  40328, 65672, 68840, 43496, 94184, 18152, 21320, 97352,
    46664, 72008, 75176, 49832, 100520, 24488, 0,     0,     0,     0,     0,     0,     0,
    0,     0,     0,     0,     0,      0,     0,     0,     0,     0,     0,     0,     0,
    0,     0,     0,     0,     0,      0,     0,     0,     0,     0,     0,     0,     600,
    76632, 25944, 51288, 54456, 29112,  79800, 3768,  6936,  82968, 32280, 57624, 60792, 35448,
    86136, 10104, 13272, 89304, 38616,  63960, 67128, 41784, 92472, 16440, 19608, 95640, 44952,
    70296, 73464, 48120, 98808, 22776,  26800, 52144, 1456,  77488, 80656, 4624,  55312, 29968,
    33136, 58480, 7792,  83824, 86992,  10960, 61648, 36304, 39472, 64816, 14128, 90160, 93328,
    17296, 67984, 42640, 45808, 71152,  20464, 96496, 99664, 23632, 74320, 48976};

public:
  static constexpr float min_range = 0.05;
  static constexpr float max_range = 50.0;
  static constexpr size_t max_scan_buffer_points = 172800;
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{{0, 360'000}, {-52'630, 52'630}};
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{400, 100};

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns = 9000 + 111110 * (2 - block_id - 1) / n_returns;

    int channel_offset_ns = 0;
    if (n_returns == 1) {
      channel_offset_ns = block_id % 2 == 0 ? loop1[channel_id] : loop2[channel_id];
    } else {
      channel_offset_ns = packet.tail.mode_flag & 0x01 ? loop1[channel_id] : loop2[channel_id];
    }

    return block_offset_ns + channel_offset_ns;
  }
};

}  // namespace nebula::drivers
