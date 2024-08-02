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
#include <vector>

namespace nebula
{
namespace drivers
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

  /// For N mirrors (= pointclouds output per 360deg rotation), the vector has length N
  std::vector<std::array<uint32_t, ChannelN>> scan_start_block_azimuths_;
  /// For N mirrors (= pointclouds output per 360deg rotation), the vector has length N
  std::vector<std::array<uint32_t, ChannelN>> scan_end_block_azimuths_;

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

public:
  explicit AngleCorrectorCorrectionBased(
    const std::shared_ptr<const HesaiCorrection> & sensor_correction, double fov_start_azimuth_rad,
    double fov_end_azimuth_rad, double /* scan_cut_azimuth_rad */)
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

    // For each field (= mirror), find the raw block azimuths corresponding FoV start and end
    for (size_t field_id = 0; field_id < correction_->frameNumber; ++field_id) {
      auto frame_start = correction_->startFrame[field_id];
      auto frame_end = correction_->endFrame[field_id];

      // For each channel, find the raw start/end azimuths for the current frame
      auto & channel_start_azimuths = scan_start_block_azimuths_.emplace_back();
      auto & channel_end_azimuths = scan_end_block_azimuths_.emplace_back();
      for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
        auto current_azimuth = frame_start;
        auto start_angle_found = false;
        auto end_angle_found = false;
        for (; current_azimuth != frame_end;
             current_azimuth = (current_azimuth + 1) % MAX_AZIMUTH) {
          auto corrected_azimuth = getCorrectedAngleData(current_azimuth, channel_id).azimuth_rad;

          // First, iterate through azimuths until FoV start is reached, record as start angle.
          // Then, search for azimuth until the FoV end is reached, record as end angle and end
          // iteration. In all other cases, skip to the next azimuth,
          if (!start_angle_found && corrected_azimuth >= fov_start_azimuth_rad) {
            start_angle_found = true;
            channel_start_azimuths[channel_id] = current_azimuth;
          } else if (!start_angle_found || corrected_azimuth < fov_end_azimuth_rad) {
            continue;
          } else {
            end_angle_found = true;
            channel_end_azimuths[channel_id] = current_azimuth;
            break;
          }
        }

        if (!start_angle_found || !end_angle_found) {
          throw std::runtime_error("Fatal error while initializing angle correction");
        }
      }
    }

    if (
      scan_start_block_azimuths_.size() != sensor_correction->frameNumber ||
      scan_end_block_azimuths_.size() != sensor_correction->frameNumber) {
      throw std::runtime_error("Fatal error while initializing angle correction");
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

  bool passedEmitAngle(uint32_t /* last_azimuth */, uint32_t /* current_azimuth */) override
  {
    throw std::runtime_error("not implemented");
  }

  bool passedTimestampResetAngle(
    uint32_t /* last_azimuth */, uint32_t /* current_azimuth */) override
  {
    throw std::runtime_error("not implemented");
  }

  bool isInsideFoV(uint32_t /* current_azimuth */) override
  {
    throw std::runtime_error("not implemented");
  }

  bool isInsideOverlap(uint32_t /* current_azimuth */) override
  {
    throw std::runtime_error("not implemented");
  }
};

}  // namespace drivers
}  // namespace nebula
