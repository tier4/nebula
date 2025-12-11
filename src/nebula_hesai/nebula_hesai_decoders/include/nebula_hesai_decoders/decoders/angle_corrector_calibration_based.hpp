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
  std::array<int32_t, ChannelN> azimuth_offset_exact_{};

  std::array<float, ChannelN> elevation_cos_{};
  std::array<float, ChannelN> elevation_sin_{};
  std::array<std::array<float, ChannelN>, max_azimuth> azimuth_cos_{};
  std::array<std::array<float, ChannelN>, max_azimuth> azimuth_sin_{};

  uint32_t encoder_emit_angle_;
  uint32_t encoder_timestamp_reset_angle_;
  uint32_t encoder_fov_start_angle_;
  uint32_t encoder_fov_end_angle_;

  uint32_t spatial_fov_start_angle_;
  uint32_t spatial_fov_end_angle_;
  uint32_t spatial_cut_angle_;

  bool is_360_;

  [[nodiscard]] int32_t to_exact_angle(double angle_deg) const
  {
    return std::round(angle_deg * AngleUnit);
  }

  [[nodiscard]] float to_radians(int32_t angle_exact) const
  {
    return deg2rad(angle_exact / static_cast<double>(AngleUnit));
  }

public:
  /// @brief Construct an AngleCorrectorCalibrationBased and pre-compute trigonometry lookup tables
  ///
  /// @param sensor_calibration The sensor calibration data
  /// @param fov_start_azimuth_deg The start of the FoV in spatial degrees
  /// @param fov_end_azimuth_deg The end of the FoV in spatial degrees
  /// @param scan_cut_azimuth_deg The angle at which the scan is cut in spatial degrees
  /// @throws std::runtime_error if the sensor calibration data is nullptr
  /// @return The constructed AngleCorrectorCalibrationBased
  explicit AngleCorrectorCalibrationBased(
    const std::shared_ptr<const HesaiCalibrationConfiguration> & sensor_calibration,
    double fov_start_azimuth_deg, double fov_end_azimuth_deg, double scan_cut_azimuth_deg)
  : spatial_fov_start_angle_(to_exact_angle(fov_start_azimuth_deg)),
    spatial_fov_end_angle_(to_exact_angle(fov_end_azimuth_deg)),
    spatial_cut_angle_(normalize_angle(to_exact_angle(scan_cut_azimuth_deg), max_azimuth))
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

    for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
      float elevation_angle_deg = sensor_calibration->elev_angle_map.at(channel_id);
      float azimuth_offset_deg = sensor_calibration->azimuth_offset_map.at(channel_id);

      int32_t azimuth_offset = to_exact_angle(azimuth_offset_deg);
      correction_min = std::min(correction_min, azimuth_offset);
      correction_max = std::max(correction_max, azimuth_offset);

      elevation_angle_rad_[channel_id] = deg2rad(elevation_angle_deg);
      azimuth_offset_exact_[channel_id] = azimuth_offset;

      elevation_cos_[channel_id] = cosf(elevation_angle_rad_[channel_id]);
      elevation_sin_[channel_id] = sinf(elevation_angle_rad_[channel_id]);
    }

    // ////////////////////////////////////////
    // Raw azimuth threshold angles
    // ////////////////////////////////////////

    int32_t encoder_emit_angle = spatial_cut_angle_ - correction_min;
    encoder_emit_angle_ = normalize_angle(encoder_emit_angle, max_azimuth);

    int32_t encoder_fov_start_angle = spatial_fov_start_angle_ - correction_max;
    encoder_fov_start_angle_ = normalize_angle(encoder_fov_start_angle, max_azimuth);

    int32_t encoder_fov_end_angle = spatial_fov_end_angle_ - correction_min;
    encoder_fov_end_angle_ = normalize_angle(encoder_fov_end_angle, max_azimuth);

    // Reset timestamp on FoV start if FoV < 360 deg and scan is cut at FoV end.
    // Otherwise, reset timestamp on publish
    is_360_ = normalize_angle(spatial_fov_start_angle_, max_azimuth) ==
              normalize_angle(spatial_fov_end_angle_, max_azimuth);
    bool reset_timestamp_on_publish = is_360_ || (spatial_fov_end_angle_ != spatial_cut_angle_);

    if (reset_timestamp_on_publish) {
      int32_t encoder_timestamp_reset_angle = spatial_cut_angle_ - correction_max;
      encoder_timestamp_reset_angle_ = normalize_angle(encoder_timestamp_reset_angle, max_azimuth);
    } else {
      encoder_timestamp_reset_angle_ = encoder_fov_start_angle_;
    }

    // ////////////////////////////////////////
    // Azimuth lookup tables
    // ////////////////////////////////////////

    for (size_t block_azimuth = 0; block_azimuth < max_azimuth; block_azimuth++) {
      for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
        int32_t spatial_azimuth = block_azimuth + azimuth_offset_exact_[channel_id];
        float spatial_azimuth_rad = to_radians(spatial_azimuth);
        azimuth_cos_[block_azimuth][channel_id] = cosf(spatial_azimuth_rad);
        azimuth_sin_[block_azimuth][channel_id] = sinf(spatial_azimuth_rad);
      }
    }
  }

  [[nodiscard]] CorrectedAngleData get_corrected_angle_data(
    uint32_t block_azimuth, uint32_t channel_id) const override
  {
    int32_t spatial_azimuth = block_azimuth + azimuth_offset_exact_[channel_id];
    spatial_azimuth = normalize_angle(spatial_azimuth, max_azimuth);
    float azimuth_rad = to_radians(spatial_azimuth);

    float elevation_rad = elevation_angle_rad_[channel_id];
    elevation_rad = normalize_angle(elevation_rad, M_PIf * 2);

    return {
      static_cast<uint32_t>(spatial_azimuth),
      azimuth_rad,
      elevation_rad,
      azimuth_sin_[block_azimuth][channel_id],
      azimuth_cos_[block_azimuth][channel_id],
      elevation_sin_[channel_id],
      elevation_cos_[channel_id]};
  }

  [[nodiscard]] bool passed_emit_angle(
    uint32_t last_azimuth, uint32_t current_azimuth) const override
  {
    return angle_is_between(last_azimuth, current_azimuth, encoder_emit_angle_, false);
  }

  [[nodiscard]] bool passed_timestamp_reset_angle(
    uint32_t last_azimuth, uint32_t current_azimuth) const override
  {
    return angle_is_between(last_azimuth, current_azimuth, encoder_timestamp_reset_angle_, false);
  }

  [[nodiscard]] bool is_inside_fov(uint32_t current_azimuth) const override
  {
    if (is_360_) return true;
    return angle_is_between(encoder_fov_start_angle_, encoder_fov_end_angle_, current_azimuth);
  }

  [[nodiscard]] bool is_inside_overlap(uint32_t current_azimuth) const override
  {
    return angle_is_between(encoder_timestamp_reset_angle_, encoder_emit_angle_, current_azimuth);
  }

  [[nodiscard]] uint32_t fov_min_spatial() const override { return spatial_fov_start_angle_; }

  [[nodiscard]] uint32_t fov_max_spatial() const override { return spatial_fov_end_angle_; }

  [[nodiscard]] uint32_t cut_angle_spatial() const override { return spatial_cut_angle_; }
};

}  // namespace nebula::drivers
