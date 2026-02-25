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

template <size_t RowN, size_t ColumnN>
class AngleCorrectorCalibrationBasedSolidState : public AngleCorrector<HesaiSolidStateCalibration, ColumnN>
{
private:
  std::array<std::array<CorrectedAngleData, RowN>, ColumnN> correctedAngleData;

public:

  explicit AngleCorrectorCalibrationBasedSolidState(
    const std::shared_ptr<const HesaiSolidStateCalibration> & sensor_calibration,
    double fov_start_azimuth_deg, double fov_end_azimuth_deg, double scan_cut_azimuth_deg)
  {
    // not used parameters
    (void) fov_start_azimuth_deg;
    (void) fov_end_azimuth_deg;
    (void) scan_cut_azimuth_deg;

    if (sensor_calibration == nullptr) {
      throw std::runtime_error(
        "Cannot instantiate AngleCorrectorCalibrationBasedSolidState without calibration data");
    }

    // ////////////////////////////////////////
    // Build lookup table
    // ////////////////////////////////////////

    size_t calib_i = 0;

    const double res_coeff = 0.01 * sensor_calibration->resolution * M_PI / 180.;  // also, convert to rad

    for (size_t j = 0; j < ColumnN; j++)  // column
    {
      for (size_t i = 0; i < RowN; i++)  // row
      {
        // calibration vectors contain column-major ordered elevation and azimuth angles in degrees,
        // in "cartesian order":
        // - first value is lower left sensor point;
        // - first column last point is top left sensor point;
        // - last column first point is lower right sensor point;
        // - last column last point is top right sensor point;
        // Note: azimuth has a value of 0° when the point lies in the plane X=0 (so
        // to get correct point coordinates, sine and cosine have to be used correctly,
        // as described in user manual. See hesai_decoder.hpp for the implementation)
        const double azi = sensor_calibration->azimuth_adjust.at(calib_i) * res_coeff ;
        const double ele = sensor_calibration->elevation_adjust.at(calib_i) * res_coeff;

        ++calib_i;

        auto C = CorrectedAngleData();

        C.azimuth_rad = static_cast<float>(azi);
        C.elevation_rad = static_cast<float>(ele);
        C.sin_azimuth = static_cast<float>(sin(azi));
        C.cos_azimuth = static_cast<float>(cos(azi));
        C.sin_elevation = static_cast<float>(sin(ele));
        C.cos_elevation = static_cast<float>(cos(ele));

        correctedAngleData[j][i] = C;
      }
    }
  }

  [[nodiscard]] CorrectedAngleData get_corrected_angle_data(uint32_t row_id, uint32_t col_id) const
  {
    return correctedAngleData[col_id][row_id];
  }

  // this base method is not used for solid state sensor, as all angles came from get_corrected_angle_data
  [[nodiscard]] CorrectedAzimuths<ColumnN, float> get_corrected_azimuths(uint32_t block_azimuth) const
  {
    // not used parameters
    (void) block_azimuth;
    return CorrectedAzimuths<ColumnN, float>();
  };

  static bool passed_emit_angle(uint32_t last_azimuth, uint32_t current_azimuth)
  {
    return last_azimuth > current_azimuth;
  }

  static bool passed_timestamp_reset_angle(uint32_t last_azimuth, uint32_t current_azimuth)
  {
    return last_azimuth > current_azimuth;
  }
};

}  // namespace nebula::drivers
