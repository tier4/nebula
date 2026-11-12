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

#include <cstdint>
#include <iostream>
#include <ostream>
#include <vector>

namespace nebula::drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

struct Tail128E4X
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

  uint32_t udp_sequence;

  uint16_t imu_temperature;
  uint16_t imu_acceleration_unit;
  uint16_t imu_angular_velocity_unit;
  uint32_t imu_timestamp;
  uint16_t imu_x_axis_acceleration;
  uint16_t imu_y_axis_acceleration;
  uint16_t imu_z_axis_acceleration;
  uint16_t imu_x_axis_angular_velocity;
  uint16_t imu_y_axis_angular_velocity;
  uint16_t imu_z_axis_angular_velocity;

  uint32_t crc_tail;

  /// @brief Get the azimuth state of the given block in the packet
  /// @param block_id The block ID (i.e. its index in the packet). Valid IDs are 0 and 1.
  /// @return The azimuth state number of the block
  uint8_t get_azimuth_state(unsigned int block_id) const
  {
    return (azimuth_state >> (14 - block_id * 2)) & 0b11;
  }
};

struct ChannelHealth128E4X
{
  uint16_t fault_info_num;
  uint32_t channel_health_status;
  uint8_t reserved[2];
};

struct FunctionalSafety128E4X
{
  static constexpr uint64_t update_cycle_ns = 5'000'000;

  enum class LidarState : uint8_t {
    INITIALIZATION = 0,
    NORMAL = 1,
    WARNING = 2,
    PRE_PERFORMANCE_DEGRADATION = 3,
    PERFORMANCE_DEGRADATION = 4,
    PRE_SHUTDOWN = 5,
    SHUTDOWN_OR_OUTPUT_UNTRUSTED = 6,
    STANDBY = 7,
  };

  enum class FaultCodeType : uint8_t {
    NONE = 0,
    CURRENT_FAULT = 1,
    PAST_FAULT = 2  /// Currently unsupported by the sensor
  };

  uint8_t fs_version;

  uint8_t bitfield1;
  BITFIELD_ACCESSOR(LidarState, lidar_state, 5, 7, bitfield1)
  BITFIELD_ACCESSOR(FaultCodeType, fault_code_type, 3, 4, bitfield1)
  BITFIELD_ACCESSOR(uint8_t, rolling_counter, 0, 2, bitfield1)

  uint8_t bitfield2;
  BITFIELD_ACCESSOR(uint8_t, total_fault_code_num, 4, 7, bitfield2)
  BITFIELD_ACCESSOR(uint8_t, fault_code_id, 0, 3, bitfield2)

  uint16_t fault_code;
  ChannelHealth128E4X channel_health;
  uint32_t crc_fs;

  [[nodiscard]] FunctionalSafetySeverity severity() const
  {
    switch (lidar_state()) {
      case LidarState::INITIALIZATION:
      case LidarState::NORMAL:
      case LidarState::WARNING:
        return FunctionalSafetySeverity::OK;
      case LidarState::PRE_PERFORMANCE_DEGRADATION:
      case LidarState::PERFORMANCE_DEGRADATION:
      case LidarState::PRE_SHUTDOWN:
        return FunctionalSafetySeverity::WARNING;
      case LidarState::SHUTDOWN_OR_OUTPUT_UNTRUSTED:
      case LidarState::STANDBY:
      default:
        return FunctionalSafetySeverity::ERROR;
    }
  }

  friend bool operator==(const FunctionalSafety128E4X & lhs, const FunctionalSafety128E4X & rhs)
  {
    return lhs.lidar_state() == rhs.lidar_state() &&
           lhs.fault_code_type() == rhs.fault_code_type() &&
           lhs.rolling_counter() == rhs.rolling_counter() &&
           lhs.total_fault_code_num() == rhs.total_fault_code_num() &&
           lhs.fault_code == rhs.fault_code;
  }

  friend bool operator!=(const FunctionalSafety128E4X & lhs, const FunctionalSafety128E4X & rhs)
  {
    return !(lhs == rhs);
  }
};

struct Packet128E4X : public PacketBase<2, 128, 2, 100>
{
  using body_t = BodyWithCrc<Block<Unit3B, Packet128E4X::n_channels>, Packet128E4X::n_blocks>;
  Header12B header;
  body_t body;
  FunctionalSafety128E4X fs;
  Tail128E4X tail;

  /* Ignored optional fields */

  // uint8_t cyber_security[32];
};

#pragma pack(pop)

}  // namespace hesai_packet

// FIXME(mojomex):
// The OT128 datasheet has entirely different numbers (and more azimuth states).
// With the current sensor version, the numbers from the new datasheet are incorrect
// (clouds do not sync to ToS but ToS+.052s)
class Pandar128E4X : public HesaiSensor<hesai_packet::Packet128E4X>
{
private:
  enum BlockageType : uint16_t {
    /// There was no laser emission, this return is a placeholder
    VALID_NO_EMISSION = 0,
    /// The return is too close (within 0.3m of the sensor)
    INVALID_TOO_CLOSE = 1,
    /// The return is too close for a far-field channel (within 1.4m of the sensor)
    INVALID_TOO_CLOSE_FAR_FIELD = 2,
    /// The return is too far, has been absorbed completely, or is filtered out due to noise etc.
    INVALID_NO_RETURN = 3,
  };

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

  [[nodiscard]] point_filters::BlockageState get_blockage_type(uint16_t raw_distance) const override
  {
    switch (raw_distance) {
      case BlockageType::VALID_NO_EMISSION:
      case BlockageType::INVALID_TOO_CLOSE_FAR_FIELD:
      case BlockageType::INVALID_NO_RETURN:
        return point_filters::BlockageState::UNSURE;
      case BlockageType::INVALID_TOO_CLOSE:
        return point_filters::BlockageState::BLOCKAGE;
      default:
        return point_filters::BlockageState::NO_BLOCKAGE;
    }
  }
};

}  // namespace nebula::drivers
