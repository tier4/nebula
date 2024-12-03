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

#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_sensor.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <cmath>

namespace nebula::drivers
{

class VLS128 : public VelodyneSensor
{
public:
  // To ignore an empty data blocks in VLS128 case
  /// @brief VLS128 Dual return mode data structure in VLS128 User manual p.57
  int get_num_padding_blocks(bool dual_return) override
  {
    if (dual_return) return 4;
    return 0;
  }

  // calculate and stack the firing timing for each laser timeing
  /// @brief laser timing for VLS128 from VLS128 User manual in p.61
  bool fill_azimuth_cache() override
  {
    for (uint8_t i = 0; i < 16; i++) {
      laser_azimuth_cache_[i] = (vls128_channel_duration / vls128_seq_duration) * (i + i / 8);
    }
    return true;
  }

  /// @brief formula from VLS128 User manual in p.65
  /// @param azimuth Azimuth angle
  /// @param azimuth_diff Azimuth difference
  /// @param firing_order Firing order
  /// @return Corrected azimuth
  uint16_t get_azimuth_corrected(
    uint16_t azimuth, float azimuth_diff, int /* firing_sequence */, int firing_order) override
  {
    float azimuth_corrected = azimuth + (azimuth_diff * laser_azimuth_cache_[firing_order]);

    return static_cast<uint16_t>(round(azimuth_corrected)) % 36000;
  }

  // Not succeed nebula_test on only VLP32 so add this function
  // Choose the correct azimuth from the 2 azimuths
  static uint16_t get_true_rotation(
    uint16_t azimuth_corrected, uint16_t /* current_block_rotation */)
  {
    return azimuth_corrected;
  }

  uint32_t get_bank(uint32_t bank, uint32_t header) override
  {
    // Used to detect which bank of 32 lasers is in this block.
    switch (header) {
      case bank_1:
        bank = 0;
        break;
      case bank_2:
        bank = 32;
        break;
      case bank_3:
        bank = 64;
        break;
      case bank_4:
        bank = 96;
        break;
      default:
        RCLCPP_ERROR(
          rclcpp::get_logger("VelodyneDecoder"),
          "Invalid bank origin detected in packet. Skipping packet.");
        return 0;  // bad packet: skip the rest
    }
    return bank;
  }

  int get_firing_order(int channels, int scans_per_firing) override
  {
    return channels / scans_per_firing;
  }

  int get_channel_number(int unit_idx) override { return unit_idx % channels_per_firing_sequence; }

  constexpr static int num_maintenance_periods = 1;

  constexpr static int num_simultaneous_firings = 8;

  constexpr static double firing_sequences_per_block = 0.25;

  constexpr static int channels_per_firing_sequence = 128;

  constexpr static float distance_resolution_m = 0.004f;

  constexpr static double full_firing_cycle_s = 53.3 * 1e-6;

  constexpr static double single_firing_s = 2.665 * 1e-6;

  constexpr static double offset_packet_time = 8.7 * 1e-6;

  /** Special Definitions for VLS128 support **/
  constexpr static const float vls128_distance_resolution = 0.004f;  // [m]

  constexpr static const float vls128_channel_duration =
    2.665f;  // [µs] Channels corresponds to one laser firing

  constexpr static const float vls128_seq_duration =
    53.3f;  // [µs] Sequence is a set of laser firings including recharging
  // These are used to detect which bank of 32 lasers is in this block
  constexpr static const uint16_t bank_1 = 0xeeff;
  constexpr static const uint16_t bank_2 = 0xddff;
  constexpr static const uint16_t bank_3 = 0xccff;
  constexpr static const uint16_t bank_4 = 0xbbff;

private:
  float laser_azimuth_cache_[16]{};
};
}  // namespace nebula::drivers
