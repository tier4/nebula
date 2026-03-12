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

#include "nebula_hesai_decoders/decoders/functional_safety.hpp"
#include "nebula_hesai_decoders/decoders/hesai_packet.hpp"
#include "nebula_hesai_decoders/decoders/hesai_sensor.hpp"

#include <nebula_core_common/util/bitfield.hpp>

#include <vector>

namespace nebula::drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

struct TailFT120
{
  uint8_t reserved1[7];
  uint16_t column_id;
  uint8_t frame_id;  // counter, 0-255; incremented at each new scan
  uint8_t reserved2;
  uint8_t return_mode;
  uint16_t frame_period;  // 100, ms (sensor works @ 10Hz)
  SecondsSinceEpoch date_time;
  uint32_t timestamp;
  uint8_t factory_information;  // fixed, 0x42
  uint32_t udp_sequence;
  uint32_t crc_tail;
  uint32_t
    signature[4];  // packet AES signature, pre-header to crc_tail; 0, if no key set in sensor
};

struct PacketFT120
: public PacketBase<1, 120, 2, 160>  // using degreeSubdivisions as the column count, to be supplied
                                     // in AngleCorrectorCalibrationBasedSolidState
{
  using body_t = Body<
    NoAzimuthBlock<Unit5B, PacketFT120::n_channels>, PacketFT120::n_blocks>;  // manual, 3.1.2.3
  Header19B header;
  body_t body;
  TailFT120 tail;  // tail contains ColumnID value, used to identify the column of sensor readings
                   // inside the packet

  /* Ignored optional fields */
  // 3.1.3. Ethernet tail, 4 more bytes for frame check sequence
  // uint8_t cyber_security[32];
};

#pragma pack(pop)

}  // namespace hesai_packet

class PandarFT120 : public HesaiSensor<hesai_packet::PacketFT120, AngleCorrectionType::SOLIDSTATE>
{
private:
public:
  static constexpr float min_range = 0.05;
  static constexpr float max_range = 25.0;
  static constexpr int32_t col_N = 160;
  static constexpr int32_t row_N = 120;
  static constexpr size_t max_scan_buffer_points = col_N * row_N;
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{
    {40'000, 140'000}, {-37'500, 37'500}};
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{
    (fov_mdeg.azimuth.end - fov_mdeg.azimuth.start) / col_N,
    (fov_mdeg.elevation.end - fov_mdeg.elevation.start) / row_N,
  };

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    // avoid warning "unused parameter"
    (void)block_id;
    (void)channel_id;
    (void)packet;

    return 0;  // all measurements are took at the same time
  }

  ReturnType get_return_type(
    hesai_packet::return_mode::ReturnMode return_mode, unsigned int return_idx,
    const std::vector<const typename packet_t::body_t::block_t::unit_t *> & return_units) override
  {
    // we could get info directly from packet contents:
    // - return_mode is a copy of PandarFT120.tail.return_mode
    // - return_idx is a copy of PandarFT120.header.return_num

    (void)return_units;

    switch (return_mode) {
      case hesai_packet::return_mode::SINGLE_FIRST:
        return ReturnType::FIRST;
      case hesai_packet::return_mode::SINGLE_STRONGEST:
        return ReturnType::STRONGEST;

      case hesai_packet::return_mode::DUAL_FIRST_STRONGEST:
        // return_idx is 1 or 2
        return return_idx == 1 ? ReturnType::FIRST : ReturnType::STRONGEST;

      default:
        return ReturnType::UNKNOWN;
    }
  }
};

}  // namespace nebula::drivers
