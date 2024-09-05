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
  static constexpr size_t MAX_AZIMUTH = 360 * AngleUnit;
  const std::shared_ptr<const HesaiCorrection> correction_;
  rclcpp::Logger logger_;

  std::array<float, MAX_AZIMUTH> cos_{};
  std::array<float, MAX_AZIMUTH> sin_{};

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
  int findField(uint32_t azimuth)
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

  uint32_t all_channels(uint32_t azi, double threshold, bool any, bool eq_ok)
  {
    for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
      auto azi_corr = getCorrectedAngleData(azi, channel_id).azimuth_rad;
      if (!any && (azi_corr < threshold || (!eq_ok && azi_corr == threshold))) return false;
      if (any && (azi_corr > threshold || (eq_ok && azi_corr == threshold))) return true;
    }

    return !any;
  }

  uint32_t bin_search(uint32_t start, uint32_t end, double threshold, bool any, bool eq_ok)
  {
    if (start > end) return FrameAngleInfo::unset;

    if (end - start <= 1) {
      bool result_start =
        all_channels(normalize_angle<uint32_t>(start, MAX_AZIMUTH), threshold, any, eq_ok);
      if (result_start) return start;
      return end;
    }

    uint32_t next = (start + end) / 2;

    bool result_next =
      all_channels(normalize_angle<uint32_t>(next, MAX_AZIMUTH), threshold, any, eq_ok);
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

    for (size_t i = 0; i < MAX_AZIMUTH; ++i) {
      float rad = 2.f * i * M_PI / MAX_AZIMUTH;
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
      if (frame_end < frame_start) frame_end += MAX_AZIMUTH;

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

  CorrectedAngleData getCorrectedAngleData(uint32_t block_azimuth, uint32_t channel_id) override
  {
    int field = findField(block_azimuth);

    auto elevation =
      correction_->elevation[channel_id] +
      correction_->getElevationAdjustV3(channel_id, block_azimuth) * (AngleUnit / 100);
    elevation = (MAX_AZIMUTH + elevation) % MAX_AZIMUTH;

    auto azimuth = (block_azimuth + MAX_AZIMUTH - correction_->startFrame[field]) * 2 -
                   correction_->azimuth[channel_id] +
                   correction_->getAzimuthAdjustV3(channel_id, block_azimuth) * (AngleUnit / 100);
    azimuth = (MAX_AZIMUTH + azimuth) % MAX_AZIMUTH;

    float azimuth_rad = 2.f * azimuth * M_PI / MAX_AZIMUTH;
    float elevation_rad = 2.f * elevation * M_PI / MAX_AZIMUTH;

    return {azimuth_rad,   elevation_rad,   sin_[azimuth],
            cos_[azimuth], sin_[elevation], cos_[elevation]};
  }

  bool passedEmitAngle(uint32_t last_azimuth, uint32_t current_azimuth) override
  {
    for (const auto & frame_angles : frame_angle_info_) {
      if (angle_is_between(last_azimuth, current_azimuth, frame_angles.scan_emit, false))
        return true;
    }

    return false;
  }

  bool passedTimestampResetAngle(uint32_t last_azimuth, uint32_t current_azimuth) override
  {
    for (const auto & frame_angles : frame_angle_info_) {
      if (angle_is_between(last_azimuth, current_azimuth, frame_angles.timestamp_reset, false))
        return true;
    }

    return false;
  }

  bool isInsideFoV(uint32_t last_azimuth, uint32_t current_azimuth) override
  {
    for (const auto & frame_angles : frame_angle_info_) {
      if (
        angle_is_between(frame_angles.fov_start, frame_angles.fov_end, current_azimuth, false) ||
        angle_is_between(frame_angles.fov_start, frame_angles.fov_end, last_azimuth, false))
        return true;
    }

    return false;
  }

  bool isInsideOverlap(uint32_t last_azimuth, uint32_t current_azimuth) override
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
