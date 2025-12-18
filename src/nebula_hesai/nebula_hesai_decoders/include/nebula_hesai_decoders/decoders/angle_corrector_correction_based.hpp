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

#include "nebula_core_decoders/angles.hpp"
#include "nebula_hesai_common/hesai_common.hpp"
#include "nebula_hesai_decoders/decoders/angle_corrector.hpp"

#include <nebula_core_common/nebula_common.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{

template <size_t ChannelN, size_t AngleUnit>
class AngleCorrectorCorrectionBased : public AngleCorrector<HesaiCorrection, ChannelN>
{
private:
  static constexpr size_t max_azimuth = 360 * AngleUnit;
  std::shared_ptr<const HesaiCorrection> correction_;
  rclcpp::Logger logger_;

  std::array<float, max_azimuth> cos_{};
  std::array<float, max_azimuth> sin_{};

  /// @brief For a given azimuth value, find its corresponding output field
  /// @param azimuth The azimuth to get the field for
  /// @return The correct output field, as specified in @ref HesaiCorrection
  [[nodiscard]] size_t find_field(uint32_t azimuth) const
  {
    // Assumes that:
    // * none of the startFrames are defined as > 360 deg (< 0 not possible since they are unsigned)
    // * the fields are arranged in ascending order (e.g. field 1: 20-140deg, field 2: 140-260deg
    // etc.) These assumptions hold for AT128E2X.
    size_t field = correction_->frameNumber - 1;
    for (size_t i = 0; i < correction_->frameNumber; ++i) {
      if (azimuth < correction_->startFrame[i]) return field;
      field = i;
    }

    return field;
  }

  [[nodiscard]] int32_t to_exact_angle(double angle_deg) const
  {
    return std::round(angle_deg * AngleUnit);
  }

  [[nodiscard]] float to_radians(int32_t angle_exact) const
  {
    return deg2rad(angle_exact / static_cast<double>(AngleUnit));
  }

  [[nodiscard]] int32_t get_corrected_azimuth(
    size_t field, uint32_t block_azimuth, size_t channel_id) const
  {
    assert(field < correction_->frameNumber);
    assert(channel_id < ChannelN);

    /*
        int32_t azimuth_exact = (block_azimuth + max_azimuth - correction_->startFrame[field]) * 2 -
                                correction_->azimuth[channel_id] +
                                correction_->get_azimuth_adjust_v3(channel_id, block_azimuth) *
                                  static_cast<int32_t>(AngleUnit / 100);
        azimuth_exact = normalize_angle(azimuth_exact, max_azimuth);
    */

    int32_t azimuth_exact = (block_azimuth - correction_->startFrame[field]) * 2;
    azimuth_exact += correction_->azimuth[channel_id];
    azimuth_exact += correction_->get_azimuth_adjust_v3(channel_id, block_azimuth) *
                     static_cast<int32_t>(AngleUnit / 100);
    azimuth_exact = normalize_angle(azimuth_exact, max_azimuth);

    return azimuth_exact;
  }

  [[nodiscard]] int32_t get_corrected_elevation(uint32_t block_azimuth, size_t channel_id) const
  {
    assert(field < correction_->frameNumber);
    assert(channel_id < ChannelN);

    int32_t elevation_exact = correction_->elevation[channel_id];
    elevation_exact += correction_->get_elevation_adjust_v3(channel_id, block_azimuth) *
                       static_cast<int32_t>(AngleUnit / 100);
    return elevation_exact;
  }

public:
  explicit AngleCorrectorCorrectionBased(
    const std::shared_ptr<const HesaiCorrection> & sensor_correction)
  : correction_(sensor_correction), logger_(rclcpp::get_logger("AngleCorrectorCorrectionBased"))
  {
    if (sensor_correction == nullptr) {
      throw std::runtime_error(
        "Cannot instantiate AngleCorrectorCorrectionBased without correction data");
    }

    logger_.set_level(rclcpp::Logger::Level::Debug);

    // ////////////////////////////////////////
    // Trigonometry lookup tables
    // ////////////////////////////////////////

    for (size_t i = 0; i < max_azimuth; ++i) {
      float rad = 2.f * i * M_PIf / max_azimuth;
      cos_[i] = cosf(rad);
      sin_[i] = sinf(rad);
    }
  }

  [[nodiscard]] CorrectedAngleData get_corrected_angle_data(
    uint32_t block_azimuth, uint32_t channel_id) const override
  {
    int32_t elevation_exact = get_corrected_elevation(block_azimuth, channel_id);

    // Allow negative angles in the radian value. This makes visualization of this field nicer and
    // should have no other mathematical implications in downstream modules.
    float elevation_rad = 2.f * elevation_exact * M_PI / max_azimuth;
    // Then, normalize the integer value to the positive [0, MAX_AZIMUTH] range for array indexing
    elevation_exact = normalize_angle(elevation_exact, max_azimuth);

    size_t field = find_field(block_azimuth);
    int32_t azimuth_exact = get_corrected_azimuth(field, block_azimuth, channel_id);
    float azimuth_rad = to_radians(azimuth_exact);

    return {
      static_cast<uint32_t>(azimuth_exact),
      azimuth_rad,
      elevation_rad,
      sin_[azimuth_exact],
      cos_[azimuth_exact],
      sin_[elevation_exact],
      cos_[elevation_exact]};
  }

  [[nodiscard]] std::array<int32_t, ChannelN> get_corrected_azimuths(
    uint32_t block_azimuth) const override
  {
    std::array<int32_t, ChannelN> corrected_azimuths;
    size_t field = find_field(block_azimuth);

    for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
      corrected_azimuths[channel_id] = get_corrected_azimuth(field, block_azimuth, channel_id);
    }

    return corrected_azimuths;
  }
};

}  // namespace nebula::drivers
