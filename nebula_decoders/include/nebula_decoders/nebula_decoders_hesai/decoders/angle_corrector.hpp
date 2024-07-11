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

#include <rclcpp/rclcpp.hpp>

#include <cstdint>

namespace nebula
{
namespace drivers
{

struct CorrectedAngleData
{
  float azimuth_rad;
  float elevation_rad;
  float sin_azimuth;
  float cos_azimuth;
  float sin_elevation;
  float cos_elevation;
};

/// @brief Handles angle correction for given azimuth/channel combinations, as well as trigonometry
/// lookup tables
template <typename CorrectionDataT>
class AngleCorrector
{
public:
  using correction_data_t = CorrectionDataT;

  /// @brief Get the corrected azimuth and elevation for a given block and channel, along with their
  /// sin/cos values.
  /// @param block_azimuth The block's azimuth (including optional fine azimuth), in the sensor's
  /// angle unit
  /// @param channel_id The laser channel's id
  /// @return The corrected angles (azimuth, elevation) in radians and their sin/cos values
  virtual CorrectedAngleData getCorrectedAngleData(uint32_t block_azimuth, uint32_t channel_id) = 0;

  /**
   * @brief Determines whether the given channel passed the end angle of the FoV when going from
   * `last_raw_azimuth` to `current_raw_azimuth`.
   *
   * @param current_raw_azimuth The current raw block azimuth
   * @param last_raw_azimuth the last processed block azimuth
   * @param channel_id The channel's ID
   * @return true if the channel passed the FoV end angle
   * @return false otherwise
   */
  [[nodiscard]] virtual bool didChannelPassFovEnd(
    uint32_t current_raw_azimuth, uint32_t last_raw_azimuth, uint32_t channel_id) const = 0;

  /**
   * @brief Determines whether the given channel passed the start angle of the FoV when going from
   * `last_raw_azimuth` to `current_raw_azimuth`.
   *
   * @param current_raw_azimuth The current raw block azimuth
   * @param last_raw_azimuth the last processed block azimuth
   * @param channel_id The channel's ID
   * @return true if the channel passed the FoV start angle
   * @return false otherwise
   */
  [[nodiscard]] virtual bool didChannelPassFovStart(
    uint32_t current_raw_azimuth, uint32_t last_raw_azimuth, uint32_t channel_id) const = 0;

  /**
   * @brief Determines if the given channel's corrected azimuth is within FoV bounds for the given
   * `raw_azimuth`
   *
   * @param raw_azimuth The raw block azimuth
   * @param channel_id The channel's ID
   * @return true if the channel is within FoV bounds
   * @return false otherwise
   */
  [[nodiscard]] virtual bool isChannelInFov(uint32_t raw_azimuth, uint32_t channel_id) const = 0;
};

}  // namespace drivers
}  // namespace nebula
