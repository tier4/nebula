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

namespace nebula
{
namespace drivers
{

template <size_t ChannelN, size_t AngleUnit>
class AngleCorrectorCalibrationBased : public AngleCorrector<HesaiCalibrationConfiguration>
{
private:
  static constexpr size_t MAX_AZIMUTH_LEN = 360 * AngleUnit;

  std::array<float, ChannelN> elevation_angle_rad_{};
  std::array<float, ChannelN> azimuth_offset_rad_{};
  std::array<float, MAX_AZIMUTH_LEN> block_azimuth_rad_{};

  std::array<float, ChannelN> elevation_cos_{};
  std::array<float, ChannelN> elevation_sin_{};
  std::array<std::array<float, ChannelN>, MAX_AZIMUTH_LEN> azimuth_cos_{};
  std::array<std::array<float, ChannelN>, MAX_AZIMUTH_LEN> azimuth_sin_{};

public:
  AngleCorrectorCalibrationBased(
    const std::shared_ptr<const HesaiCalibrationConfiguration> & sensor_calibration)
  {
    if (sensor_calibration == nullptr) {
      throw std::runtime_error(
        "Cannot instantiate AngleCorrectorCalibrationBased without calibration data");
    }

    for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
      float elevation_angle_deg = sensor_calibration->elev_angle_map.at(channel_id);
      float azimuth_offset_deg = sensor_calibration->azimuth_offset_map.at(channel_id);

      elevation_angle_rad_[channel_id] = deg2rad(elevation_angle_deg);
      azimuth_offset_rad_[channel_id] = deg2rad(azimuth_offset_deg);

      elevation_cos_[channel_id] = cosf(elevation_angle_rad_[channel_id]);
      elevation_sin_[channel_id] = sinf(elevation_angle_rad_[channel_id]);
    }

    for (size_t block_azimuth = 0; block_azimuth < MAX_AZIMUTH_LEN; block_azimuth++) {
      block_azimuth_rad_[block_azimuth] = deg2rad(block_azimuth / static_cast<double>(AngleUnit));

      for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
        float precision_azimuth =
          block_azimuth_rad_[block_azimuth] + azimuth_offset_rad_[channel_id];

        azimuth_cos_[block_azimuth][channel_id] = cosf(precision_azimuth);
        azimuth_sin_[block_azimuth][channel_id] = sinf(precision_azimuth);
      }
    }
  }

  CorrectedAngleData getCorrectedAngleData(uint32_t block_azimuth, uint32_t channel_id) override
  {
    float azimuth_rad = block_azimuth_rad_[block_azimuth] + azimuth_offset_rad_[channel_id];
    float elevation_rad = elevation_angle_rad_[channel_id];

    return {
      azimuth_rad,
      elevation_rad,
      azimuth_sin_[block_azimuth][channel_id],
      azimuth_cos_[block_azimuth][channel_id],
      elevation_sin_[channel_id],
      elevation_cos_[channel_id]};
  }

  bool hasScanned(uint32_t current_azimuth, uint32_t last_azimuth, uint32_t sync_azimuth) override
  {
    // Cut the scan when the azimuth passes over the sync_azimuth
    uint32_t current_diff_from_sync =
      (MAX_AZIMUTH_LEN + current_azimuth - sync_azimuth) % MAX_AZIMUTH_LEN;
    uint32_t last_diff_from_sync =
      (MAX_AZIMUTH_LEN + last_azimuth - sync_azimuth) % MAX_AZIMUTH_LEN;

    return current_diff_from_sync < last_diff_from_sync;
  }
};

}  // namespace drivers
}  // namespace nebula
