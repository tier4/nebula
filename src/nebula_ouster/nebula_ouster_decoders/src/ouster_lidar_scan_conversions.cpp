// Copyright 2026 TIER IV, Inc.
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

#include "nebula_ouster_decoders/ouster_lidar_scan_conversions.hpp"

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_decoders/angles.hpp>

#include <ouster/core/chanfield.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>

namespace nebula::drivers
{

using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::RANGE_UNIT;
using ouster::sdk::core::SensorInfo;
namespace ChanField = ouster::sdk::core::ChanField;

OusterPointCloudConverter::OusterPointCloudConverter(
  const SensorInfo & sensor_info, const FieldOfView<float, Degrees> & fov)
: h_(sensor_info.format.pixels_per_column),
  w_(sensor_info.format.columns_per_frame),
  // TODO(unaal) retrieve return type from the sensor_info config
  return_type_(sensor_info.num_returns() > 1 ? ReturnType::LAST : ReturnType::STRONGEST),
  lut_(sensor_info, false),
  azimuth_rad_(h_ * w_),
  elevation_rad_(h_),
  in_fov_(h_ * w_)
{
  const float fov_azimuth_start_rad = deg2rad(fov.azimuth.start);
  const float fov_azimuth_end_rad = deg2rad(fov.azimuth.end);

  for (size_t row = 0; row < h_; ++row) {
    const auto altitude_deg = static_cast<float>(sensor_info.beam_altitude_angles[row]);
    elevation_rad_[row] = deg2rad(altitude_deg);
    const bool row_in_fov =
      altitude_deg >= fov.elevation.start && altitude_deg <= fov.elevation.end;

    for (size_t col = 0; col < w_; ++col) {
      const size_t i = row * w_ + col;
      // The direction vector is in the same (sensor) frame as the output points, so its azimuth
      // is the azimuth of the beam, independent of the measured range.
      const auto ei = static_cast<Eigen::Index>(i);
      const double azimuth = std::atan2(lut_.direction(ei, 1), lut_.direction(ei, 0));
      azimuth_rad_[i] = static_cast<float>(normalize_angle(azimuth, 2 * M_PI));
      in_fov_[i] = static_cast<uint8_t>(
        row_in_fov && azimuth_rad_[i] >= fov_azimuth_start_rad &&
        azimuth_rad_[i] <= fov_azimuth_end_rad);
    }
  }
}

NebulaPointCloudPtr OusterPointCloudConverter::convert(const LidarFrame & scan) const
{
  if (scan.h != h_ || scan.w != w_ || h_ == 0 || w_ == 0 || !scan.has_field(ChanField::RANGE)) {
    return std::make_shared<NebulaPointCloud>();
  }

  int first_col = 0;
  try {
    first_col = scan.get_first_valid_column();
  } catch (const std::runtime_error &) {
    // No valid columns, hence no valid points.
    return std::make_shared<NebulaPointCloud>();
  }

  // Resolve all fields once: each has_field()/field() call is a string-keyed hash map lookup.
  const auto range_img = scan.field<uint32_t>(ChanField::RANGE);
  const bool has_reflectivity = scan.has_field(ChanField::REFLECTIVITY);
  const bool has_signal = !has_reflectivity && scan.has_field(ChanField::SIGNAL);
  std::optional<Eigen::Ref<const ouster::sdk::core::img_t<uint8_t>>> reflectivity;
  std::optional<Eigen::Ref<const ouster::sdk::core::img_t<uint16_t>>> signal;
  if (has_reflectivity) {
    reflectivity.emplace(scan.field<uint8_t>(ChanField::REFLECTIVITY));
  } else if (has_signal) {
    signal.emplace(scan.field<uint16_t>(ChanField::SIGNAL));
  }
  const auto timestamps = scan.timestamp();
  const auto status = scan.status();
  const uint64_t base_ts = timestamps(first_col);

  const double * direction = lut_.direction.data();
  const double * offset = lut_.offset.data();
  const auto range_unit = static_cast<float>(RANGE_UNIT);

  auto cloud = std::make_shared<NebulaPointCloud>();
  cloud->reserve(h_ * w_);

  for (size_t row = 0; row < h_; ++row) {
    for (size_t col = 0; col < w_; ++col) {
      const size_t i = row * w_ + col;
      const auto erow = static_cast<Eigen::Index>(row);
      const auto ecol = static_cast<Eigen::Index>(col);
      const uint32_t r_mm = range_img(erow, ecol);
      if (r_mm == 0U || in_fov_[i] == 0U) {
        continue;
      }

      // Skip invalid columns.
      if ((status(ecol) & 0x01U) == 0U) {
        continue;
      }

      NebulaPoint tgt_pt{};
      // Same as ouster::sdk::core::cartesian(), but only for the points that are kept.
      const double r = static_cast<double>(r_mm);
      tgt_pt.x = static_cast<float>(r * direction[3 * i + 0] + offset[3 * i + 0]);
      tgt_pt.y = static_cast<float>(r * direction[3 * i + 1] + offset[3 * i + 1]);
      tgt_pt.z = static_cast<float>(r * direction[3 * i + 2] + offset[3 * i + 2]);
      if (has_reflectivity) {
        tgt_pt.intensity = (*reflectivity)(erow, ecol);
      } else if (has_signal) {
        const uint16_t v = (*signal)(erow, ecol);
        tgt_pt.intensity = static_cast<std::uint8_t>(std::min<uint16_t>(255U, v >> 4));
      }
      tgt_pt.return_type = static_cast<std::uint8_t>(return_type_);
      tgt_pt.channel = static_cast<std::uint16_t>(row);
      tgt_pt.azimuth = azimuth_rad_[i];
      tgt_pt.elevation = elevation_rad_[row];
      tgt_pt.distance = static_cast<float>(r_mm) * range_unit;

      const uint64_t col_ts = timestamps(ecol);
      if (base_ts > 0U && col_ts >= base_ts) {
        tgt_pt.time_stamp = static_cast<std::uint32_t>((col_ts - base_ts) & 0xFFFFFFFFU);
      } else {
        tgt_pt.time_stamp = static_cast<std::uint32_t>(col_ts & 0xFFFFFFFFU);
      }

      cloud->push_back(tgt_pt);
    }
  }

  return cloud;
}

}  // namespace nebula::drivers
