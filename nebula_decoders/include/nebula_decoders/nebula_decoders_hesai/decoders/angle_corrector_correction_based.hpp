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
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector.hpp"

#include <cstdint>
#include <memory>
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
  static constexpr size_t MAX_AZIMUTH_LENGTH = 360 * AngleUnit;
  const std::shared_ptr<const HesaiCorrection> correction_;
  rclcpp::Logger logger_;

  std::array<float, MAX_AZIMUTH_LENGTH> cos_{};
  std::array<float, MAX_AZIMUTH_LENGTH> sin_{};

  std::vector<uint32_t> cut_azimuths_;

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
    const std::shared_ptr<const HesaiCorrection> & sensor_correction, float scan_cut_azimuth_rad)
  : correction_(sensor_correction), logger_(rclcpp::get_logger("AngleCorrectorCorrectionBased"))
  {
    if (sensor_correction == nullptr) {
      throw std::runtime_error(
        "Cannot instantiate AngleCorrectorCorrectionBased without correction data");
    }

    logger_.set_level(rclcpp::Logger::Level::Debug);

    for (size_t i = 0; i < MAX_AZIMUTH_LENGTH; ++i) {
      float rad = 2.f * i * M_PI / MAX_AZIMUTH_LENGTH;
      cos_[i] = cosf(rad);
      sin_[i] = sinf(rad);
    }

    for (size_t field_id = 0; field_id < correction_->frameNumber; ++field_id) {
      auto start = correction_->startFrame[field_id];
      auto end = correction_->endFrame[field_id];
      auto raw_azimuth = start;
      for (; raw_azimuth != end; raw_azimuth = (raw_azimuth + 1) % MAX_AZIMUTH_LENGTH) {
        for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
          auto corrected_azimuth = getCorrectedAngleData(raw_azimuth, channel_id).azimuth_rad;
          if (corrected_azimuth < scan_cut_azimuth_rad) {
            break;  // Not all channels are past the cut azimuth, search at next raw azimuth
          }
        }

        // All channels are past the cut azimuth, add this raw_azimuths to the cut azimuths
        break;
      }

      cut_azimuths_.push_back(raw_azimuth);
    }

    if (cut_azimuths_.size() != correction_->frameNumber) {
      throw std::runtime_error(
        "Sensor has " + std::to_string(correction_->frameNumber) +
        " fields but calculation resulted in " + std::to_string(cut_azimuths_.size()));
    }
  }

  CorrectedAngleData getCorrectedAngleData(uint32_t block_azimuth, uint32_t channel_id) override
  {
    int field = findField(block_azimuth);

    auto elevation =
      correction_->elevation[channel_id] +
      correction_->getElevationAdjustV3(channel_id, block_azimuth) * (AngleUnit / 100);
    elevation = (MAX_AZIMUTH_LENGTH + elevation) % MAX_AZIMUTH_LENGTH;

    auto azimuth = (block_azimuth + MAX_AZIMUTH_LENGTH - correction_->startFrame[field]) * 2 -
                   correction_->azimuth[channel_id] +
                   correction_->getAzimuthAdjustV3(channel_id, block_azimuth) * (AngleUnit / 100);
    azimuth = (MAX_AZIMUTH_LENGTH + azimuth) % MAX_AZIMUTH_LENGTH;

    float azimuth_rad = 2.f * azimuth * M_PI / MAX_AZIMUTH_LENGTH;
    float elevation_rad = 2.f * elevation * M_PI / MAX_AZIMUTH_LENGTH;

    return {azimuth_rad,   elevation_rad,   sin_[azimuth],
            cos_[azimuth], sin_[elevation], cos_[elevation]};
  }

  bool blockCompletesScan(uint32_t block_azimuth, uint32_t last_azimuth) override
  {
    for (auto cut_azimuth : cut_azimuths_) {
      if (cut_azimuth < last_azimuth) {
        cut_azimuth += MAX_AZIMUTH_LENGTH;
      }

      auto current_azimuth = block_azimuth;
      if (current_azimuth < last_azimuth) {
        current_azimuth += MAX_AZIMUTH_LENGTH;
      }

      return current_azimuth >= cut_azimuth && last_azimuth < cut_azimuth;
    }
  }
};

}  // namespace drivers
}  // namespace nebula
