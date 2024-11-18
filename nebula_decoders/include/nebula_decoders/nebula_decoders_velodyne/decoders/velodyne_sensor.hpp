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

#include <cstdint>

namespace nebula::drivers
{
class VelodyneSensor
{
public:
  VelodyneSensor() = default;
  VelodyneSensor(const VelodyneSensor &) = default;
  VelodyneSensor(VelodyneSensor &&) = default;
  VelodyneSensor & operator=(const VelodyneSensor &) = default;
  VelodyneSensor & operator=(VelodyneSensor &&) = default;
  virtual ~VelodyneSensor() = default;

  /// @brief each VLP lidars packat structure in user manual. If you know details, see commens in
  /// each <vlp_list>.hpp file. To ignore an empty data blocks which is created by only VLS128 dual
  /// return mode case
  virtual int get_num_padding_blocks(bool /* dual_return */) { return 0; }

  /// @brief each VLP lidar laser timing in user manual. If you know details, see commens in each
  /// <vlp_list>.hpp file. calculate and stack the firing timing for each laser timeing used in
  /// getAzimuthCorrected to calculate the corrected azimuth
  virtual bool fill_azimuth_cache() { return false; }

  /// @brief VSL128User manual p. Packet structure
  virtual uint32_t get_bank(uint32_t bank, uint32_t /* header */) { return bank; }

  /// @brief each VLP calculating sample code and formula in user manual. If you know details, see
  /// commens in each <vlp_list>.hpp file. calculate the corrected azimuth from each firing timing.
  virtual uint16_t get_azimuth_corrected(
    uint16_t azimuth, float azimuth_diff, int firing_sequence, int firing_order) = 0;

  /// @brief each VLP calculating sample code and formula in user manual. Check packet structure.
  /// Get a correct firing order
  virtual int get_firing_order(int /* channels_per_block */, int /* scans_per_firing */)
  {
    return 0;
  }

  /// @brief each VLP calculating sample code and formula in user manual. Check packet structure.
  /// Get a correct channel number
  virtual int get_channel_number(int /* unit_idx */) { return 0; }
};
}  // namespace nebula::drivers
