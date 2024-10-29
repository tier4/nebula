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

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_decoders/nebula_decoders_common/angles.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector.hpp"

#include <nebula_common/nebula_common.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>

namespace nebula::drivers
{

template <size_t ChannelN, size_t AngleUnit>
class AngleCorrectorCalibrationBased : public AngleCorrector<HesaiCalibrationConfiguration>
{
private:
  static constexpr size_t max_azimuth = 360 * AngleUnit;

  std::array<float, ChannelN> elevation_angle_rad_{};
  std::array<float, ChannelN> azimuth_offset_rad_{};
  std::array<float, max_azimuth> block_azimuth_rad_{};

  std::array<float, ChannelN> elevation_cos_{};
  std::array<float, ChannelN> elevation_sin_{};
  std::array<std::array<float, ChannelN>, max_azimuth> azimuth_cos_{};
  std::array<std::array<float, ChannelN>, max_azimuth> azimuth_sin_{};

public:
  uint32_t emit_angle_raw_;
  uint32_t timestamp_reset_angle_raw_;
  uint32_t fov_start_raw_;
  uint32_t fov_end_raw_;

  bool is_360_;

  explicit AngleCorrectorCalibrationBased(
    const std::shared_ptr<const HesaiCalibrationConfiguration> & sensor_calibration,
    double fov_start_azimuth_deg, double fov_end_azimuth_deg, double scan_cut_azimuth_deg)
  {
    if (sensor_calibration == nullptr) {
      throw std::runtime_error(
        "Cannot instantiate AngleCorrectorCalibrationBased without calibration data");
    }

    // ////////////////////////////////////////
    // Elevation lookup tables
    // ////////////////////////////////////////

    int32_t correction_min = INT32_MAX;
    int32_t correction_max = INT32_MIN;

    auto round_away_from_zero = [](float value) {
      return (value < 0) ? std::floor(value) : std::ceil(value);
    };

    for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
      float elevation_angle_deg = sensor_calibration->elev_angle_map.at(channel_id);
      float azimuth_offset_deg = sensor_calibration->azimuth_offset_map.at(channel_id);

      int32_t azimuth_offset_raw = round_away_from_zero(azimuth_offset_deg * AngleUnit);
      correction_min = std::min(correction_min, azimuth_offset_raw);
      correction_max = std::max(correction_max, azimuth_offset_raw);

      elevation_angle_rad_[channel_id] = deg2rad(elevation_angle_deg);
      azimuth_offset_rad_[channel_id] = deg2rad(azimuth_offset_deg);

      elevation_cos_[channel_id] = cosf(elevation_angle_rad_[channel_id]);
      elevation_sin_[channel_id] = sinf(elevation_angle_rad_[channel_id]);
    }

    // ////////////////////////////////////////
    // Raw azimuth threshold angles
    // ////////////////////////////////////////

    int32_t emit_angle_raw = std::ceil(scan_cut_azimuth_deg * AngleUnit);
    emit_angle_raw -= correction_min;
    emit_angle_raw_ = normalize_angle<int32_t>(emit_angle_raw, max_azimuth);

    int32_t fov_start_raw = std::floor(fov_start_azimuth_deg * AngleUnit);
    fov_start_raw -= correction_max;
    fov_start_raw_ = normalize_angle<int32_t>(fov_start_raw, max_azimuth);

    int32_t fov_end_raw = std::ceil(fov_end_azimuth_deg * AngleUnit);
    fov_end_raw -= correction_min;
    fov_end_raw_ = normalize_angle<int32_t>(fov_end_raw, max_azimuth);

    // Reset timestamp on FoV start if FoV < 360 deg and scan is cut at FoV end.
    // Otherwise, reset timestamp on publish
    is_360_ =
      normalize_angle(fov_start_azimuth_deg, 360.) == normalize_angle(fov_end_azimuth_deg, 360.);
    bool reset_timestamp_on_publish = is_360_ || (normalize_angle(fov_end_azimuth_deg, 360.) !=
                                                  normalize_angle(scan_cut_azimuth_deg, 360.));

    if (reset_timestamp_on_publish) {
      int32_t timestamp_reset_angle_raw = std::floor(scan_cut_azimuth_deg * AngleUnit);
      timestamp_reset_angle_raw -= correction_max;
      timestamp_reset_angle_raw_ = normalize_angle<int32_t>(timestamp_reset_angle_raw, max_azimuth);
    } else {
      timestamp_reset_angle_raw_ = fov_start_raw_;
    }

    // ////////////////////////////////////////
    // Azimuth lookup tables
    // ////////////////////////////////////////

    for (size_t block_azimuth = 0; block_azimuth < max_azimuth; block_azimuth++) {
      block_azimuth_rad_[block_azimuth] = deg2rad(block_azimuth / static_cast<double>(AngleUnit));

      for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
        float precision_azimuth =
          block_azimuth_rad_[block_azimuth] + azimuth_offset_rad_[channel_id];

        azimuth_cos_[block_azimuth][channel_id] = cosf(precision_azimuth);
        azimuth_sin_[block_azimuth][channel_id] = sinf(precision_azimuth);
      }
    }
  }

  CorrectedAngleData get_corrected_angle_data(uint32_t block_azimuth, uint32_t channel_id) override
  {
    float azimuth_rad = block_azimuth_rad_[block_azimuth] + azimuth_offset_rad_[channel_id];
    azimuth_rad = normalize_angle(azimuth_rad, M_PIf * 2);

    float elevation_rad = elevation_angle_rad_[channel_id];

    return {
      azimuth_rad,
      elevation_rad,
      azimuth_sin_[block_azimuth][channel_id],
      azimuth_cos_[block_azimuth][channel_id],
      elevation_sin_[channel_id],
      elevation_cos_[channel_id]};
  }

  bool passed_emit_angle(uint32_t last_azimuth, uint32_t current_azimuth) override
  {
    return angle_is_between(last_azimuth, current_azimuth, emit_angle_raw_, false);
  }

  bool passed_timestamp_reset_angle(uint32_t last_azimuth, uint32_t current_azimuth) override
  {
    return angle_is_between(last_azimuth, current_azimuth, timestamp_reset_angle_raw_, false);
  }

  bool is_inside_fov(uint32_t last_azimuth, uint32_t current_azimuth) override
  {
    if (is_360_) return true;
    return angle_is_between(fov_start_raw_, fov_end_raw_, current_azimuth) ||
           angle_is_between(timestamp_reset_angle_raw_, emit_angle_raw_, last_azimuth);
  }

  bool is_inside_overlap(uint32_t last_azimuth, uint32_t current_azimuth) override
  {
    return angle_is_between(timestamp_reset_angle_raw_, emit_angle_raw_, current_azimuth) ||
           angle_is_between(timestamp_reset_angle_raw_, emit_angle_raw_, last_azimuth);
  }
};

}  // namespace nebula::drivers
