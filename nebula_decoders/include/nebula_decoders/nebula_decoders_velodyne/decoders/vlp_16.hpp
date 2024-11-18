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

class VLP16 : public VelodyneSensor
{
public:
  /// @brief formula from VLP16 User manual in p.64
  /// @param azimuth Azimuth angle
  /// @param azimuth_diff Azimuth difference
  /// @param firing_sequence Firing sequence
  /// @param firing_order Firing order
  /// @return Corrected azimuth
  uint16_t get_azimuth_corrected(
    uint16_t azimuth, float azimuth_diff, int firing_sequence, int firing_order) override
  {
    float azimuth_corrected =
      azimuth + (azimuth_diff *
                 ((firing_order * vlp16_dsr_toffset) + (firing_sequence * vlp16_firing_toffset)) /
                 vlp16_block_duration);

    return static_cast<uint16_t>(round(azimuth_corrected)) % 36000;
  }

  // Not succeed nebula_test on only VLP32 so add this function
  // Choose the correct azimuth from the 2 azimuths
  static uint16_t get_true_rotation(
    uint16_t azimuth_corrected, uint16_t /* current_block_rotation */)
  {
    return azimuth_corrected;
  }

  int get_firing_order(int channels, int scans_per_firing) override
  {
    return channels / scans_per_firing;
  }

  int get_channel_number(int unit_idx) override { return unit_idx % channels_per_firing_sequence; }

  constexpr static int num_maintenance_periods = 0;

  constexpr static int num_simultaneous_firings = 1;

  constexpr static double firing_sequences_per_block = 2.0;

  constexpr static int channels_per_firing_sequence = 16;

  constexpr static float distance_resolution_m = 0.002f;

  constexpr static double full_firing_cycle_s = 55.296 * 1e-6;

  constexpr static double single_firing_s = 2.304 * 1e-6;

  constexpr static double offset_packet_time = 0;

  /** Special Defines for VLP16 support **/
  constexpr static const int vlp16_firings_per_block = 2;
  constexpr static const int vlp16_scans_per_firing = 16;
  constexpr static const float vlp16_block_duration = 110.592f;  // [µs]
  constexpr static const float vlp16_dsr_toffset = 2.304f;       // [µs]
  constexpr static const float vlp16_firing_toffset = 55.296f;   // [µs]
};
}  // namespace nebula::drivers
