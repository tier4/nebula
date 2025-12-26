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

namespace nebula::drivers
{

template <size_t ChannelN, size_t AngleUnit>
class AngleCorrectorCalibrationBased
: public AngleCorrector<HesaiCalibrationConfiguration, ChannelN>
{
private:
  static constexpr size_t max_azimuth = 360 * AngleUnit;

  std::array<float, ChannelN> elevation_angle_rad_{};
  std::array<int32_t, ChannelN> azimuth_offset_exact_{};

  std::array<float, ChannelN> elevation_cos_{};
  std::array<float, ChannelN> elevation_sin_{};
  std::array<std::array<float, ChannelN>, max_azimuth> azimuth_cos_{};
  std::array<std::array<float, ChannelN>, max_azimuth> azimuth_sin_{};

  size_t min_correction_index_{};
  size_t max_correction_index_{};

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
    const std::shared_ptr<const HesaiCalibrationConfiguration> & sensor_calibration)
  {
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

      int32_t azimuth_offset = to_exact_angle(azimuth_offset_deg);

      elevation_angle_rad_[channel_id] = deg2rad(elevation_angle_deg);
      azimuth_offset_exact_[channel_id] = azimuth_offset;

      elevation_cos_[channel_id] = cosf(elevation_angle_rad_[channel_id]);
      elevation_sin_[channel_id] = sinf(elevation_angle_rad_[channel_id]);
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

    const auto & az = azimuth_offset_exact_;
    min_correction_index_ = std::min_element(az.begin(), az.end()) - az.begin();
    max_correction_index_ = std::max_element(az.begin(), az.end()) - az.begin();
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

  [[nodiscard]] CorrectedAzimuths<ChannelN> get_corrected_azimuths(
    uint32_t block_azimuth) const override
  {
    CorrectedAzimuths<ChannelN> corrected_azimuths;

    for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
      int32_t exact_azimuth = block_azimuth + azimuth_offset_exact_[channel_id];
      exact_azimuth = normalize_angle(exact_azimuth, max_azimuth);
      corrected_azimuths.azimuths[channel_id] = exact_azimuth;
    }

    corrected_azimuths.min_correction_index = min_correction_index_;
    corrected_azimuths.max_correction_index = max_correction_index_;

    return corrected_azimuths;
  }
};

}  // namespace nebula::drivers
