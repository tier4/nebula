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

namespace nebula::drivers
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
  virtual CorrectedAngleData get_corrected_angle_data(
    uint32_t block_azimuth, uint32_t channel_id) = 0;

  virtual bool passed_emit_angle(uint32_t last_azimuth, uint32_t current_azimuth) = 0;
  virtual bool passed_timestamp_reset_angle(uint32_t last_azimuth, uint32_t current_azimuth) = 0;
  virtual bool is_inside_fov(uint32_t last_azimuth, uint32_t current_azimuth) = 0;
  virtual bool is_inside_overlap(uint32_t last_azimuth, uint32_t current_azimuth) = 0;
};

}  // namespace nebula::drivers
