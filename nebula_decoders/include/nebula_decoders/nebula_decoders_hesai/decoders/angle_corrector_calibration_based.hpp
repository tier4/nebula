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
#include <ostream>

namespace nebula
{
namespace drivers
{

template <size_t ChannelN, size_t AngleUnit>
class AngleCorrectorCalibrationBased : public AngleCorrector<HesaiCalibrationConfiguration>
{
private:
  static constexpr size_t MAX_AZIMUTH = 360 * AngleUnit;

  std::array<float, ChannelN> elevation_angle_rad_{};
  std::array<float, ChannelN> azimuth_offset_rad_{};
  std::array<float, MAX_AZIMUTH> block_azimuth_rad_{};

  std::array<float, ChannelN> elevation_cos_{};
  std::array<float, ChannelN> elevation_sin_{};
  std::array<std::array<float, ChannelN>, MAX_AZIMUTH> azimuth_cos_{};
  std::array<std::array<float, ChannelN>, MAX_AZIMUTH> azimuth_sin_{};

  std::array<uint32_t, ChannelN> scan_start_block_azimuths_{};
  std::array<uint32_t, ChannelN> scan_end_block_azimuths_{};

public:
  AngleCorrectorCalibrationBased(
    const std::shared_ptr<const HesaiCalibrationConfiguration> & sensor_calibration,
    float fov_start_azimuth_rad, float fov_end_azimuth_rad)
  {
    std::cout << "Config'd FoV: " << rad2deg(fov_start_azimuth_rad) << " -- "
              << rad2deg(fov_end_azimuth_rad) << std::endl;

    if (sensor_calibration == nullptr) {
      throw std::runtime_error(
        "Cannot instantiate AngleCorrectorCalibrationBased without calibration data");
    }

    // ////////////////////////////////////////
    // Elevation lookup tables
    // ////////////////////////////////////////

    for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
      float elevation_angle_deg = sensor_calibration->elev_angle_map.at(channel_id);
      float azimuth_offset_deg = sensor_calibration->azimuth_offset_map.at(channel_id);

      elevation_angle_rad_[channel_id] = deg2rad(elevation_angle_deg);
      azimuth_offset_rad_[channel_id] = deg2rad(azimuth_offset_deg);

      elevation_cos_[channel_id] = cosf(elevation_angle_rad_[channel_id]);
      elevation_sin_[channel_id] = sinf(elevation_angle_rad_[channel_id]);

      // Calculate block azimuth where this channel's corrected azimuth aligns with the FoV start
      auto start_azimuth_rad = fov_start_azimuth_rad - azimuth_offset_rad_[channel_id];
      int32_t start_azimuth = std::floor(rad2deg(start_azimuth_rad) * AngleUnit);
      scan_start_block_azimuths_[channel_id] = normalize_angle<int32_t>(start_azimuth, MAX_AZIMUTH);

      // Calculate block azimuth where this channel's corrected azimuth aligns with the FoV end
      auto end_azimuth_rad = fov_end_azimuth_rad - azimuth_offset_rad_[channel_id];
      int32_t end_azimuth = std::floor(rad2deg(end_azimuth_rad) * AngleUnit);
      scan_end_block_azimuths_[channel_id] = normalize_angle<int32_t>(end_azimuth, MAX_AZIMUTH);

      std::cout << "Channel " << channel_id << ": " << scan_start_block_azimuths_[channel_id]
                << " -- " << scan_end_block_azimuths_[channel_id] << std::endl;
    }

    // ////////////////////////////////////////
    // Azimuth lookup tables
    // ////////////////////////////////////////

    for (size_t block_azimuth = 0; block_azimuth < MAX_AZIMUTH; block_azimuth++) {
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

  [[nodiscard]] bool didChannelPassFovEnd(
    uint32_t current_raw_azimuth, uint32_t last_raw_azimuth, uint32_t channel_id) const override
  {
    auto end = scan_end_block_azimuths_[channel_id];
    return angle_is_between(last_raw_azimuth, current_raw_azimuth, end, false, true);
  }

  [[nodiscard]] bool didChannelPassFovStart(
    uint32_t current_raw_azimuth, uint32_t last_raw_azimuth, uint32_t channel_id) const override
  {
    auto start = scan_start_block_azimuths_[channel_id];
    return angle_is_between(last_raw_azimuth, current_raw_azimuth, start, false, true);
  }

  [[nodiscard]] bool isChannelInFov(uint32_t raw_azimuth, uint32_t channel_id) const override
  {
    auto start = scan_start_block_azimuths_[channel_id];
    auto end = scan_end_block_azimuths_[channel_id];
    return angle_is_between(start, end, raw_azimuth);
  }
};

}  // namespace drivers
}  // namespace nebula
