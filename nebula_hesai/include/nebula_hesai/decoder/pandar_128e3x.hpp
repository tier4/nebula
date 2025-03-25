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

struct Tail128E3X
{
  uint8_t reserved1[9];
  uint16_t azimuth_state;
  uint8_t operational_state;
  uint8_t return_mode;
  uint16_t motor_speed;
  DateTime<1900> date_time;
  uint32_t timestamp;
  uint8_t factory_information;

  /* Ignored optional fields */

  // uint32_t udp_sequence;

  // uint16_t imu_temperature;
  // uint16_t imu_acceleration_unit;
  // uint16_t imu_angular_velocity_unit;
  // uint32_t imu_timestamp;
  // uint16_t imu_x_axis_acceleration;
  // uint16_t imu_y_axis_acceleration;
  // uint16_t imu_z_axis_acceleration;
  // uint16_t imu_x_axis_angular_velocity;
  // uint16_t imu_y_axis_angular_velocity;
  // uint16_t imu_z_axis_angular_velocity;

  // uint32_t crc_tail;

  /// @brief Get the azimuth state of the given block in the packet
  /// @param block_id The block ID (i.e. its index in the packet). Valid IDs are 0 and 1.
  /// @return The azimuth state number of the block
  uint8_t get_azimuth_state(unsigned int block_id) const
  {
    return (azimuth_state >> (14 - block_id * 2)) & 0b11;
  }
};

struct Packet128E3X : public PacketBase<2, 128, 2, 100>
{
  using body_t = Body<Block<Unit3B, Packet128E3X::n_channels>, Packet128E3X::n_blocks>;
  Header12B header;
  body_t body;
  uint32_t crc_body;
  FunctionalSafety fs;
  Tail128E3X tail;

  /* Ignored optional fields */

  // uint8_t cyber_security[32];
};

#pragma pack(pop)

}  // namespace hesai_packet

class Pandar128E3X : public HesaiSensor<hesai_packet::Packet128E3X>
{
private:
  enum OperationalState { HIGH_RESOLUTION = 0, SHUTDOWN = 1, STANDARD = 2, ENERGY_SAVING = 3 };

  static constexpr int hires_as0_far_offset_ns[128] = {
    4436,  -1,    776,   2431,  4436,  -1,    6441,  -1,    -1,    776,   2431,  6441,  -1,
    -1,    -1,    -1,    -1,    6441,  -1,    776,   2431,  -1,    -1,    -1,    4436,  10381,
    14951, 12666, 14951, 19521, 19521, 8096,  12666, 12666, 10381, 24091, 17236, 24091, 14951,
    14951, 19521, 17236, 12666, 21806, 8096,  21806, 10381, 10381, 21806, 8096,  8096,  19521,
    12666, 12666, 24091, 24091, 17236, 21806, 17236, 14951, 10381, 14951, 17236, 17236, 8096,
    19521, 19521, 10381, 24091, 10381, 21806, 12666, 10381, 14951, 21806, 8096,  19521, 17236,
    8096,  19521, 24091, 24091, 24091, 17236, 21806, 8096,  12666, 21806, 14951, 2431,  776,
    4436,  6441,  -1,    -1,    776,   -1,    2431,  2431,  4436,  -1,    -1,    -1,    6441,
    4436,  -1,    -1,    -1,    6441,  -1,    -1,    -1,    776,   4436,  -1,    2431,  -1,
    -1,    776,   -1,    4436,  6441,  -1,    -1,    2431,  776,   6441,  -1};

  static constexpr int hires_as0_near_offset_ns[128] = {
    5201, -1,    1541, -1, -1, -1, -1, -1, -1, -1,   -1, -1, -1, -1,   -1, -1, -1, 7206,  -1,
    -1,   3196,  -1,   -1, -1, -1, -1, -1, -1, -1,   -1, -1, -1, -1,   -1, -1, -1, -1,    -1,
    -1,   27056, -1,   -1, -1, -1, -1, -1, -1, -1,   -1, -1, -1, -1,   -1, -1, -1, -1,    -1,
    -1,   -1,    -1,   -1, -1, -1, -1, -1, -1, -1,   -1, -1, -1, -1,   -1, -1, -1, 23201, -1,
    -1,   -1,    -1,   -1, -1, -1, -1, -1, -1, -1,   -1, -1, -1, 3676, -1, -1, -1, -1,    -1,
    -1,   -1,    -1,   -1, -1, -1, -1, -1, -1, 5681, -1, -1, -1, -1,   -1, -1, -1, -1,    -1,
    -1,   -1,    -1,   -1, -1, -1, -1, -1, -1, -1,   -1, -1, -1, -1};

  static constexpr int hires_as1_far_offset_ns[128] = {
    -1,    776,   -1,    -1,    -1,    2781,  -1,    4786,  6441,  -1,    -1,    -1,    776,
    6441,  2781,  776,   4786,  -1,    4786,  -1,    -1,    2781,  6441,  4786,  -1,    10731,
    15301, 13016, 15301, 19871, 19871, 8446,  13016, 13016, 10731, 24441, 17586, 24441, 15301,
    15301, 19871, 17586, 13016, 22156, 8446,  22156, 10731, 10731, 22156, 8446,  8446,  19871,
    13016, 13016, 24441, 24441, 17586, 22156, 17586, 15301, 10731, 15301, 17586, 17586, 8446,
    19871, 19871, 10731, 24441, 10731, 22156, 13016, 10731, 15301, 22156, 8446,  19871, 17586,
    8446,  19871, 24441, 24441, 24441, 17586, 22156, 8446,  13016, 22156, 15301, -1,    -1,
    -1,    -1,    6441,  2781,  -1,    776,   -1,    -1,    -1,    4786,  776,   2781,  -1,
    -1,    2781,  776,   4786,  -1,    6441,  6441,  4786,  -1,    -1,    4786,  -1,    2781,
    6441,  -1,    776,   -1,    -1,    6441,  2781,  -1,    -1,    -1,    776};

  static constexpr int hires_as1_near_offset_ns[128] = {
    -1, -1, -1, -1, -1, 4026, -1,    -1, 7206,  -1, -1, -1, -1, -1, 3546, -1,   -1, -1, -1,
    -1, -1, -1, -1, -1, -1,   12126, -1, -1,    -1, -1, -1, -1, -1, -1,   -1,   -1, -1, -1,
    -1, -1, -1, -1, -1, -1,   -1,    -1, 27406, -1, -1, -1, -1, -1, -1,   -1,   -1, -1, -1,
    -1, -1, -1, -1, -1, -1,   -1,    -1, -1,    -1, -1, -1, -1, -1, -1,   -1,   -1, -1, -1,
    -1, -1, -1, -1, -1, -1,   -1,    -1, -1,    -1, -1, -1, -1, -1, -1,   -1,   -1, -1, -1,
    -1, -1, -1, -1, -1, -1,   2021,  -1, -1,    -1, -1, -1, -1, -1, -1,   7686, -1, -1, -1,
    -1, -1, -1, -1, -1, 1541, -1,    -1, -1,    -1, -1, -1, -1, -1};

  static constexpr int hires_as2_far_offset_ns[128] = {
    4436,  -1,    776,   2781,  4436,  -1,    6 - 191, -1,    -1,    776,   2781,  6091,  -1,
    -1,    -1,    -1,    -1,    6091,  -1,    776,     2781,  -1,    -1,    -1,    4436,  10381,
    14951, 12666, 14951, 19521, 19521, 8096,  12666,   12666, 10381, 24091, 17236, 24091, 14951,
    14951, 19521, 17236, 12666, 21806, 8096,  21806,   10381, 10381, 21806, 8096,  8096,  19521,
    12666, 12666, 24091, 24091, 17236, 21806, 17236,   14951, 10381, 14951, 17236, 17236, 8096,
    19521, 19521, 10381, 24091, 10381, 21806, 12666,   10381, 14951, 21806, 8096,  19521, 17236,
    8096,  19521, 24091, 24091, 24091, 17236, 21806,   8096,  12666, 21806, 14951, 2781,  776,
    4436,  6091,  -1,    -1,    776,   -1,    2781,    2781,  4436,  -1,    -1,    -1,    6091,
    4436,  -1,    -1,    -1,    6091,  -1,    -1,      -1,    776,   4436,  -1,    2781,  -1,
    -1,    776,   -1,    4436,  6091,  -1,    -1,      2781,  776,   6091,  -1};

  static constexpr int hires_as2_near_offset_ns[128] = {
    -1,   -1, -1, -1,   -1, -1, -1, -1, -1, -1, -1, 7336, -1, -1,    -1, -1,    -1,   -1, -1,
    -1,   -1, -1, -1,   -1, -1, -1, -1, -1, -1, -1, -1,   -1, 14061, -1, -1,    -1,   -1, -1,
    -1,   -1, -1, -1,   -1, -1, -1, -1, -1, -1, -1, -1,   -1, -1,    -1, 27056, -1,   -1, -1,
    -1,   -1, -1, -1,   -1, -1, -1, -1, -1, -1, -1, -1,   -1, -1,    -1, -1,    -1,   -1, -1,
    -1,   -1, -1, -1,   -1, -1, -1, -1, -1, -1, -1, -1,   -1, -1,    -1, -1,    6856, -1, -1,
    2021, -1, -1, 3546, -1, -1, -1, -1, -1, -1, -1, -1,   -1, -1,    -1, -1,    -1,   -1, 5201,
    -1,   -1, -1, -1,   -1, -1, -1, -1, -1, -1, -1, 1541, -1, -1};

  static constexpr int hires_as3_far_offset_ns[128] = {
    -1,    776,   -1,    -1,    -1,    2431,  -1,    4086,  6091,  -1,    -1,    -1,    776,
    6091,  2431,  776,   4086,  -1,    4086,  -1,    -1,    2431,  6091,  4086,  -1,    10031,
    14601, 12316, 14601, 19171, 19171, 7746,  12316, 12316, 10031, 23741, 16886, 23741, 14601,
    14601, 19171, 16886, 12316, 21456, 7746,  21456, 10031, 10031, 21456, 7746,  7746,  19171,
    12316, 12316, 23741, 23741, 16886, 21456, 16886, 14601, 10031, 14601, 16886, 16886, 7746,
    19171, 19171, 10031, 23741, 10031, 21456, 12316, 10031, 14601, 21456, 7746,  19171, 16886,
    7746,  19171, 23741, 23741, 23741, 16886, 21456, 7746,  12316, 21456, 14601, -1,    -1,
    -1,    -1,    6091,  2431,  -1,    776,   -1,    -1,    -1,    4086,  776,   2431,  -1,
    -1,    2431,  776,   4086,  -1,    6091,  6091,  4086,  -1,    -1,    4086,  -1,    2431,
    6091,  -1,    776,   -1,    -1,    6091,  2431,  -1,    -1,    -1,    776};

  static constexpr int hires_as3_near_offset_ns[128] = {
    -1, -1, -1,   -1,    -1,   -1,    -1, -1, -1,   -1, -1,    -1, -1,   -1,  -1, -1, -1, -1, -1,
    -1, -1, -1,   -1,    4851, -1,    -1, -1, -1,   -1, -1,    -1, -1,   -1,  -1, -1, -1, -1, -1,
    -1, -1, -1,   -1,    -1,   -1,    -1, -1, -1,   -1, -1,    -1, -1,   -1,  -1, -1, -1, -1, -1,
    -1, -1, -1,   26706, -1,   -1,    -1, -1, -1,   -1, 11426, -1, -1,   -1,  -1, -1, -1, -1, -1,
    -1, -1, -1,   -1,    -1,   25136, -1, -1, -1,   -1, -1,    -1, -1,   -1,  -1, -1, -1, -1, -1,
    -1, -1, -1,   -1,    -1,   -1,    -1, -1, -1,   -1, -1,    -1, 5331, -1,  -1, -1, -1, -1, -1,
    -1, -1, 3196, -1,    -1,   -1,    -1, -1, 6856, -1, -1,    -1, -1,   1541};

  static constexpr int standard_as0_far_offset_ns[128] = {
    4436,  28554, 776,   2431,  4436,  30559, 6441,  32564, 34219, 776,   2431,  6441,  28554,
    34219, 30559, 28554, 32564, 6441,  32564, 776,   2431,  30559, 34219, 32564, 4436,  38509,
    43079, 12666, 43079, 19521, 19521, 36224, 12666, 12666, 38509, 52219, 17236, 52219, 43079,
    43079, 19521, 17236, 12666, 21806, 36224, 21806, 38509, 38509, 21806, 36224, 36224, 19521,
    12666, 12666, 52219, 52219, 17236, 21806, 17236, 43079, 38509, 43079, 17236, 17236, 36224,
    19521, 19521, 38509, 52219, 38509, 21806, 12666, 38509, 43079, 21806, 36224, 19521, 17236,
    36224, 19521, 52219, 52219, 52219, 17236, 21806, 36224, 12666, 21806, 43079, 2431,  776,
    4436,  6441,  34219, 30559, 776,   28554, 2431,  2431,  4436,  32564, 28554, 30559, 6441,
    4436,  30559, 28554, 32564, 6441,  34219, 34219, 32564, 776,   4436,  32564, 2431,  30559,
    34219, 776,   28554, 4436,  6441,  34219, 30559, 2431,  776,   6441,  28554};

  static constexpr int standard_as0_near_offset_ns[128] = {
    5201, -1,   1541, -1, -1,   31804, -1, -1,    34984, -1,    -1,    -1, -1, -1, 31324, -1,
    -1,   7206, -1,   -1, 3196, -1,    -1, -1,    -1,    39904, -1,    -1, -1, -1, -1,    -1,
    -1,   -1,   -1,   -1, -1,   -1,    -1, 27056, -1,    -1,    -1,    -1, -1, -1, 55184, -1,
    -1,   -1,   -1,   -1, -1,   -1,    -1, -1,    -1,    -1,    -1,    -1, -1, -1, -1,    -1,
    -1,   -1,   -1,   -1, -1,   -1,    -1, -1,    -1,    -1,    23201, -1, -1, -1, -1,    -1,
    -1,   -1,   -1,   -1, -1,   -1,    -1, -1,    -1,    3676,  -1,    -1, -1, -1, -1,    -1,
    -1,   -1,   -1,   -1, -1,   29799, -1, -1,    5681,  -1,    -1,    -1, -1, -1, 35464, -1,
    -1,   -1,   -1,   -1, -1,   -1,    -1, 29319, -1,    -1,    -1,    -1, -1, -1, -1,    -1};

  static constexpr int standard_as1_far_offset_ns[128] = {
    4436,  28554, 776,   2781,  4436,  30209, 6091,  31864, 33869, 776,   2781,  6091,  28554,
    33869, 30209, 28554, 31864, 6091,  31864, 776,   2781,  30209, 33869, 31864, 4436,  37809,
    42379, 12666, 42379, 19521, 19521, 35524, 12666, 12666, 37809, 51519, 17236, 51519, 42379,
    42379, 19521, 17236, 12666, 21806, 35524, 21806, 37809, 37809, 21806, 35524, 35524, 19521,
    12666, 12666, 51519, 51519, 17236, 21806, 17236, 42379, 37809, 42379, 17236, 17236, 35524,
    19521, 19521, 37809, 51519, 37809, 21806, 12666, 37809, 42379, 21806, 35524, 19521, 17236,
    35524, 19521, 51519, 51519, 51519, 17236, 21806, 35524, 12666, 21806, 42379, 2781,  776,
    4436,  6091,  33869, 30209, 776,   28554, 2781,  2781,  4436,  31864, 28554, 30209, 6091,
    4436,  30209, 28554, 31864, 6091,  33869, 33869, 31864, 776,   4436,  31864, 2781,  30209,
    33869, 776,   28554, 4436,  6091,  33869, 30209, 2781,  776,   6091,  28554};

  static constexpr int standard_as1_near_offset_ns[128] = {
    -1,    -1, -1,    -1, -1, -1,   -1,    -1,    -1,    -1, -1, 7336,  -1, -1, -1,
    -1,    -1, -1,    -1, -1, -1,   -1,    -1,    32629, -1, -1, -1,    -1, -1, -1,
    -1,    -1, 14061, -1, -1, -1,   -1,    -1,    -1,    -1, -1, -1,    -1, -1, -1,
    -1,    -1, -1,    -1, -1, -1,   -1,    -1,    27056, -1, -1, -1,    -1, -1, -1,
    54484, -1, -1,    -1, -1, -1,   -1,    39204, -1,    -1, -1, -1,    -1, -1, -1,
    -1,    -1, -1,    -1, -1, -1,   52914, -1,    -1,    -1, -1, -1,    -1, -1, -1,
    -1,    -1, 6856,  -1, -1, 2021, -1,    -1,    3546,  -1, -1, -1,    -1, -1, -1,
    -1,    -1, 33109, -1, -1, -1,   -1,    -1,    5201,  -1, -1, 30974, -1, -1, -1,
    -1,    -1, 34634, -1, -1, 1541, -1,    29319};

public:
  static constexpr float min_range = 0.1;
  static constexpr float max_range = 230.0;
  static constexpr size_t max_scan_buffer_points = 691200;
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{{0, 360'000}, {-25'000, 14'400}};
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{100, 125};

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns = 3148 - 27778 * 2 * (2 - block_id - 1) / n_returns;

    int channel_offset_ns = 0;
    bool is_hires_mode = packet.tail.operational_state == OperationalState::HIGH_RESOLUTION;
    bool is_nearfield = (hesai_packet::get_dis_unit(packet) *
                         packet.body.blocks[block_id].units[channel_id].distance) <= 2.85f;
    auto azimuth_state = packet.tail.get_azimuth_state(block_id);

    if (is_hires_mode && azimuth_state == 0 && !is_nearfield)
      channel_offset_ns = hires_as0_far_offset_ns[channel_id];
    else if (is_hires_mode && azimuth_state == 0 && is_nearfield)
      channel_offset_ns = hires_as0_near_offset_ns[channel_id];
    else if (is_hires_mode && azimuth_state == 1 && !is_nearfield)
      channel_offset_ns = hires_as1_far_offset_ns[channel_id];
    else if (is_hires_mode && azimuth_state == 1 && is_nearfield)
      channel_offset_ns = hires_as1_near_offset_ns[channel_id];
    else if (is_hires_mode && azimuth_state == 2 && !is_nearfield)
      channel_offset_ns = hires_as2_far_offset_ns[channel_id];
    else if (is_hires_mode && azimuth_state == 2 && is_nearfield)
      channel_offset_ns = hires_as2_near_offset_ns[channel_id];
    else if (is_hires_mode && azimuth_state == 3 && !is_nearfield)
      channel_offset_ns = hires_as3_far_offset_ns[channel_id];
    else if (is_hires_mode && azimuth_state == 3 && is_nearfield)
      channel_offset_ns = hires_as3_near_offset_ns[channel_id];
    else if (!is_hires_mode && azimuth_state == 0 && !is_nearfield)
      channel_offset_ns = standard_as0_far_offset_ns[channel_id];
    else if (!is_hires_mode && azimuth_state == 0 && is_nearfield)
      channel_offset_ns = standard_as0_near_offset_ns[channel_id];
    else if (!is_hires_mode && azimuth_state == 1 && !is_nearfield)
      channel_offset_ns = standard_as1_far_offset_ns[channel_id];
    else if (!is_hires_mode && azimuth_state == 1 && is_nearfield)
      channel_offset_ns = standard_as1_near_offset_ns[channel_id];
    else
      throw std::runtime_error(
        "Invalid combination of operational state and azimuth state and nearfield firing");

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
