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
class AngleCorrectorCorrectionBased : public AngleCorrector<HesaiCorrection>
{
private:
  static constexpr size_t max_azimuth = 360 * AngleUnit;
  std::shared_ptr<const HesaiCorrection> correction_;
  rclcpp::Logger logger_;

  std::array<float, max_azimuth> cos_{};
  std::array<float, max_azimuth> sin_{};

  struct FrameAngleInfo
  {
    static constexpr uint32_t unset = UINT32_MAX;
    uint32_t encoder_fov_start = unset;
    uint32_t encoder_fov_end = unset;
    uint32_t encoder_timestamp_reset = unset;
    uint32_t encoder_scan_emit = unset;
  };

  std::vector<FrameAngleInfo> frame_angle_info_;

  uint32_t spatial_fov_start_angle_;
  uint32_t spatial_fov_end_angle_;
  uint32_t spatial_cut_angle_;

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

  /// @brief For raw encoder angle `azi`, return whether all (any if `any == true`) channels'
  /// corrected azimuths are greater (or equal if `eq_ok == true`) than `threshold`.
  [[nodiscard]] bool are_corrected_angles_above_threshold(
    uint32_t azi, uint32_t threshold, bool any, bool eq_ok) const
  {
    for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
      auto azi_corr = get_corrected_angle_data(azi, channel_id).azimuth_exact;
      if (!any && (azi_corr < threshold || (!eq_ok && azi_corr == threshold))) return false;
      if (any && (azi_corr > threshold || (eq_ok && azi_corr == threshold))) return true;
    }

    return !any;
  }

  /// @brief Find and return the first raw encoder angle between the raw `start` and `end` endcoder
  /// angles for which all (any if `any == true`) channels' corrected azimuth is greater than (or
  /// equal to if `eq_ok == true`) `threshold`. Return `FrameAngleInfo::unset` if no angle is found.
  [[nodiscard]] uint32_t bin_search(
    uint32_t start, uint32_t end, uint32_t threshold, bool any, bool eq_ok) const
  {
    if (start > end) return FrameAngleInfo::unset;

    if (end - start <= 1) {
      bool result_start = are_corrected_angles_above_threshold(
        normalize_angle(start, max_azimuth), threshold, any, eq_ok);
      if (result_start) return start;
      return end;
    }

    uint32_t next = (start + end) / 2;

    bool result_next = are_corrected_angles_above_threshold(
      normalize_angle(next, max_azimuth), threshold, any, eq_ok);
    if (result_next) return bin_search(start, next, threshold, any, eq_ok);
    return bin_search(next + 1, end, threshold, any, eq_ok);
  }

  [[nodiscard]] int32_t to_exact_angle(double angle_deg) const
  {
    return std::round(angle_deg * AngleUnit);
  }

  [[nodiscard]] float to_radians(int32_t angle_exact) const
  {
    return deg2rad(angle_exact / static_cast<double>(AngleUnit));
  }

public:
  explicit AngleCorrectorCorrectionBased(
    const std::shared_ptr<const HesaiCorrection> & sensor_correction, double fov_start_azimuth_deg,
    double fov_end_azimuth_deg, double scan_cut_azimuth_deg)
  : correction_(sensor_correction),
    logger_(rclcpp::get_logger("AngleCorrectorCorrectionBased")),
    spatial_fov_start_angle_(normalize_angle(to_exact_angle(fov_start_azimuth_deg), max_azimuth)),
    spatial_fov_end_angle_(normalize_angle(to_exact_angle(fov_end_azimuth_deg), max_azimuth)),
    spatial_cut_angle_(normalize_angle(to_exact_angle(scan_cut_azimuth_deg), max_azimuth))
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

    // ////////////////////////////////////////
    // Scan start/end correction lookups
    // ////////////////////////////////////////

    // For each field (= mirror), find the raw block azimuths corresponding FoV start and end
    for (size_t field_id = 0; field_id < correction_->frameNumber; ++field_id) {
      auto frame_start = correction_->startFrame[field_id];
      auto frame_end = correction_->endFrame[field_id];
      if (frame_end < frame_start) frame_end += max_azimuth;

      FrameAngleInfo & angle_info = frame_angle_info_.emplace_back();

      angle_info.encoder_fov_start =
        bin_search(frame_start, frame_end, spatial_fov_start_angle_, true, true);
      angle_info.encoder_fov_end =
        bin_search(angle_info.encoder_fov_start, frame_end, spatial_fov_end_angle_, false, true);
      angle_info.encoder_scan_emit = bin_search(
        angle_info.encoder_fov_start, angle_info.encoder_fov_end, spatial_cut_angle_, false, true);
      angle_info.encoder_timestamp_reset = bin_search(
        angle_info.encoder_fov_start, angle_info.encoder_fov_end, spatial_cut_angle_, true, true);

      if (
        angle_info.encoder_fov_start == FrameAngleInfo::unset ||
        angle_info.encoder_fov_end == FrameAngleInfo::unset ||
        angle_info.encoder_scan_emit == FrameAngleInfo::unset ||
        angle_info.encoder_timestamp_reset == FrameAngleInfo::unset) {
        throw std::runtime_error("Not all necessary angles found!");
      }

      if (spatial_fov_start_angle_ == spatial_cut_angle_) {
        angle_info.encoder_timestamp_reset = angle_info.encoder_fov_start;
        angle_info.encoder_scan_emit = angle_info.encoder_fov_start;
      } else if (spatial_fov_end_angle_ == spatial_cut_angle_) {
        angle_info.encoder_timestamp_reset = angle_info.encoder_fov_start;
        angle_info.encoder_scan_emit = angle_info.encoder_fov_end;
      }
    }
  }

  [[nodiscard]] CorrectedAngleData get_corrected_angle_data(
    uint32_t block_azimuth, uint32_t channel_id) const override
  {
    size_t field = find_field(block_azimuth);

    int32_t elevation_exact = correction_->elevation[channel_id] +
                              correction_->get_elevation_adjust_v3(channel_id, block_azimuth) *
                                static_cast<int32_t>(AngleUnit / 100);

    // Allow negative angles in the radian value. This makes visualization of this field nicer and
    // should have no other mathematical implications in downstream modules.
    float elevation_rad = 2.f * elevation_exact * M_PI / max_azimuth;
    // Then, normalize the integer value to the positive [0, MAX_AZIMUTH] range for array indexing
    elevation_exact = normalize_angle(elevation_exact, max_azimuth);

    int32_t azimuth_exact = (block_azimuth + max_azimuth - correction_->startFrame[field]) * 2 -
                            correction_->azimuth[channel_id] +
                            correction_->get_azimuth_adjust_v3(channel_id, block_azimuth) *
                              static_cast<int32_t>(AngleUnit / 100);
    azimuth_exact = normalize_angle(azimuth_exact, max_azimuth);

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

  [[nodiscard]] bool passed_emit_angle(
    uint32_t last_azimuth, uint32_t current_azimuth) const override
  {
    for (const auto & frame_angles : frame_angle_info_) {
      if (angle_is_between(last_azimuth, current_azimuth, frame_angles.encoder_scan_emit, false))
        return true;
    }

    return false;
  }

  [[nodiscard]] bool passed_timestamp_reset_angle(
    uint32_t last_azimuth, uint32_t current_azimuth) const override
  {
    for (const auto & frame_angles : frame_angle_info_) {
      if (angle_is_between(
            last_azimuth, current_azimuth, frame_angles.encoder_timestamp_reset, false))
        return true;
    }

    return false;
  }

  [[nodiscard]] bool is_inside_fov(uint32_t current_azimuth) const override
  {
    for (const auto & frame_angles : frame_angle_info_) {
      if (angle_is_between(
            frame_angles.encoder_fov_start, frame_angles.encoder_fov_end, current_azimuth, false))
        return true;
    }

    return false;
  }

  [[nodiscard]] bool is_inside_overlap(uint32_t current_azimuth) const override
  {
    for (const auto & frame_angles : frame_angle_info_) {
      if (angle_is_between(
            frame_angles.encoder_timestamp_reset, frame_angles.encoder_scan_emit, current_azimuth))
        return true;
    }

    return false;
  }

  [[nodiscard]] uint32_t fov_min_spatial() const override { return spatial_fov_start_angle_; }

  [[nodiscard]] uint32_t fov_max_spatial() const override { return spatial_fov_end_angle_; }

  [[nodiscard]] uint32_t cut_angle_spatial() const override { return spatial_cut_angle_; }
};

}  // namespace nebula::drivers
