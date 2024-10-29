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
  const std::shared_ptr<const HesaiCorrection> correction_;
  rclcpp::Logger logger_;

  std::array<float, max_azimuth> cos_{};
  std::array<float, max_azimuth> sin_{};

  struct FrameAngleInfo
  {
    static constexpr uint32_t unset = UINT32_MAX;
    uint32_t fov_start = unset;
    uint32_t fov_end = unset;
    uint32_t timestamp_reset = unset;
    uint32_t scan_emit = unset;
  };

  std::vector<FrameAngleInfo> frame_angle_info_;

  /// @brief For a given azimuth value, find its corresponding output field
  /// @param azimuth The azimuth to get the field for
  /// @return The correct output field, as specified in @ref HesaiCorrection
  int find_field(uint32_t azimuth)
  {
    // Assumes that:
    // * none of the startFrames are defined as > 360 deg (< 0 not possible since they are unsigned)
    // * the fields are arranged in ascending order (e.g. field 1: 20-140deg, field 2: 140-260deg
    // etc.) These assumptions hold for AT128E2X.
    int field = correction_->frameNumber - 1;
    for (size_t i = 0; i < correction_->frameNumber; ++i) {
      if (azimuth < correction_->startFrame[i]) return field;
      field = i;
    }

    // This is never reached if correction_ is correct
    return field;
  }

  /// @brief For raw encoder angle `azi`, return whether all (any if `any == true`) channels'
  /// corrected azimuths are greater (or equal if `eq_ok == true`) than `threshold`.
  bool are_corrected_angles_above_threshold(uint32_t azi, double threshold, bool any, bool eq_ok)
  {
    for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
      auto azi_corr = get_corrected_angle_data(azi, channel_id).azimuth_rad;
      if (!any && (azi_corr < threshold || (!eq_ok && azi_corr == threshold))) return false;
      if (any && (azi_corr > threshold || (eq_ok && azi_corr == threshold))) return true;
    }

    return !any;
  }

  /// @brief Find and return the first raw encoder angle between the raw `start` and `end` endcoder
  /// angles for which all (any if `any == true`) channels' corrected azimuth is greater than (or
  /// equal to if `eq_ok == true`) `threshold`. Return `FrameAngleInfo::unset` if no angle is found.
  uint32_t bin_search(uint32_t start, uint32_t end, double threshold, bool any, bool eq_ok)
  {
    if (start > end) return FrameAngleInfo::unset;

    if (end - start <= 1) {
      bool result_start = are_corrected_angles_above_threshold(
        normalize_angle<uint32_t>(start, max_azimuth), threshold, any, eq_ok);
      if (result_start) return start;
      return end;
    }

    uint32_t next = (start + end) / 2;

    bool result_next = are_corrected_angles_above_threshold(
      normalize_angle<uint32_t>(next, max_azimuth), threshold, any, eq_ok);
    if (result_next) return bin_search(start, next, threshold, any, eq_ok);
    return bin_search(next + 1, end, threshold, any, eq_ok);
  }

public:
  explicit AngleCorrectorCorrectionBased(
    const std::shared_ptr<const HesaiCorrection> & sensor_correction, double fov_start_azimuth_deg,
    double fov_end_azimuth_deg, double scan_cut_azimuth_deg)
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

    // ////////////////////////////////////////
    // Scan start/end correction lookups
    // ////////////////////////////////////////

    auto fov_start_rad = deg2rad(fov_start_azimuth_deg);
    auto fov_end_rad = deg2rad(fov_end_azimuth_deg);
    auto scan_cut_rad = deg2rad(scan_cut_azimuth_deg);

    // For each field (= mirror), find the raw block azimuths corresponding FoV start and end
    for (size_t field_id = 0; field_id < correction_->frameNumber; ++field_id) {
      auto frame_start = correction_->startFrame[field_id];
      auto frame_end = correction_->endFrame[field_id];
      if (frame_end < frame_start) frame_end += max_azimuth;

      FrameAngleInfo & angle_info = frame_angle_info_.emplace_back();

      angle_info.fov_start = bin_search(frame_start, frame_end, fov_start_rad, true, true);
      angle_info.fov_end = bin_search(angle_info.fov_start, frame_end, fov_end_rad, false, true);
      angle_info.scan_emit =
        bin_search(angle_info.fov_start, angle_info.fov_end, scan_cut_rad, false, true);
      angle_info.timestamp_reset =
        bin_search(angle_info.fov_start, angle_info.fov_end, scan_cut_rad, true, true);

      if (
        angle_info.fov_start == FrameAngleInfo::unset ||
        angle_info.fov_end == FrameAngleInfo::unset ||
        angle_info.scan_emit == FrameAngleInfo::unset ||
        angle_info.timestamp_reset == FrameAngleInfo::unset) {
        throw std::runtime_error("Not all necessary angles found!");
      }

      if (fov_start_rad == scan_cut_rad) {
        angle_info.timestamp_reset = angle_info.fov_start;
        angle_info.scan_emit = angle_info.fov_start;
      } else if (fov_end_rad == scan_cut_rad) {
        angle_info.timestamp_reset = angle_info.fov_start;
        angle_info.scan_emit = angle_info.fov_end;
      }
    }
  }

  CorrectedAngleData get_corrected_angle_data(uint32_t block_azimuth, uint32_t channel_id) override
  {
    int field = find_field(block_azimuth);

    int32_t elevation = correction_->elevation[channel_id] +
                        correction_->get_elevation_adjust_v3(channel_id, block_azimuth) *
                          static_cast<int32_t>(AngleUnit / 100);

    // Allow negative angles in the radian value. This makes visualization of this field nicer and
    // should have no other mathematical implications in downstream modules.
    float elevation_rad = 2.f * elevation * M_PI / max_azimuth;
    // Then, normalize the integer value to the positive [0, MAX_AZIMUTH] range for array indexing
    elevation = (max_azimuth + elevation) % max_azimuth;

    int32_t azimuth = (block_azimuth + max_azimuth - correction_->startFrame[field]) * 2 -
                      correction_->azimuth[channel_id] +
                      correction_->get_azimuth_adjust_v3(channel_id, block_azimuth) *
                        static_cast<int32_t>(AngleUnit / 100);
    azimuth = (max_azimuth + azimuth) % max_azimuth;

    float azimuth_rad = 2.f * azimuth * M_PI / max_azimuth;

    return {azimuth_rad,   elevation_rad,   sin_[azimuth],
            cos_[azimuth], sin_[elevation], cos_[elevation]};
  }

  bool passed_emit_angle(uint32_t last_azimuth, uint32_t current_azimuth) override
  {
    for (const auto & frame_angles : frame_angle_info_) {
      if (angle_is_between(last_azimuth, current_azimuth, frame_angles.scan_emit, false))
        return true;
    }

    return false;
  }

  bool passed_timestamp_reset_angle(uint32_t last_azimuth, uint32_t current_azimuth) override
  {
    for (const auto & frame_angles : frame_angle_info_) {
      if (angle_is_between(last_azimuth, current_azimuth, frame_angles.timestamp_reset, false))
        return true;
    }

    return false;
  }

  bool is_inside_fov(uint32_t last_azimuth, uint32_t current_azimuth) override
  {
    for (const auto & frame_angles : frame_angle_info_) {
      if (
        angle_is_between(frame_angles.fov_start, frame_angles.fov_end, current_azimuth, false) ||
        angle_is_between(frame_angles.fov_start, frame_angles.fov_end, last_azimuth, false))
        return true;
    }

    return false;
  }

  bool is_inside_overlap(uint32_t last_azimuth, uint32_t current_azimuth) override
  {
    for (const auto & frame_angles : frame_angle_info_) {
      if (
        angle_is_between(frame_angles.timestamp_reset, frame_angles.scan_emit, current_azimuth) ||
        angle_is_between(frame_angles.timestamp_reset, frame_angles.scan_emit, last_azimuth))
        return true;
    }

    return false;
  }
};

}  // namespace nebula::drivers
