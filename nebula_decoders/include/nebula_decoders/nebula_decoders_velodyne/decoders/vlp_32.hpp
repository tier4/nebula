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

#include <cmath>

namespace nebula::drivers
{
class VLP32 : public VelodyneSensor
{
public:
  // calculate and stack the firing timing for each laser timeing
  /// @brief laser timing for VLP32 from VLP32 User manual in p.61
  bool fill_azimuth_cache() override
  {
    for (uint8_t i = 0; i < 16; i++) {
      laser_azimuth_cache_[i] = (vlp32_channel_duration / vlp32_seq_duration) * (i + i / 2);
    }
    return true;
  }

  /// @brief formula from VLP32 User manual in p.62
  /// @param azimuth Azimuth angle
  /// @param azimuth_diff Azimuth difference between a current azimuth and a next azimuth
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
  uint16_t get_true_rotation(uint16_t /* azimuth_corrected */, uint16_t current_block_rotation)
  {
    return current_block_rotation;
  }

  int get_firing_order(int channels, int scans_per_firing) override
  {
    return channels / scans_per_firing;
  }

  int get_channel_number(int unit_idx) override { return unit_idx % channels_per_firing_sequence; }

  constexpr static int num_maintenance_periods = 0;

  constexpr static int num_simultaneous_firings = 2;

  constexpr static double firing_sequences_per_block = 1.0;

  constexpr static int channels_per_firing_sequence = 32;

  constexpr static float distance_resolution_m = 0.004f;

  constexpr static double full_firing_cycle_s = 55.296 * 1e-6;

  constexpr static double single_firing_s = 2.304 * 1e-6;

  constexpr static double offset_packet_time = 0;

  /** Special Definitions for VLS32 support **/
  constexpr static const float vlp32_channel_duration =
    2.304f;  // [µs] Channels corresponds to one laser firing
  constexpr static const float vlp32_seq_duration =
    55.296f;  // [µs] Sequence is a set of laser firings including recharging

private:
  float laser_azimuth_cache_[16];
};
}  // namespace nebula::drivers
