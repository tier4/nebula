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

#include <ouster/chanfield.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>

namespace nebula::drivers
{

namespace
{
float normalize_azimuth_deg(float az_deg)
{
  while (az_deg < 0.0f) {
    az_deg += 360.0F;
  }
  while (az_deg >= 360.0f) {
    az_deg -= 360.0F;
  }
  return az_deg;
}

}  // namespace

using ouster::sdk::core::LidarScan;
using ouster::sdk::core::RANGE_UNIT;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::XYZLut;
namespace ChanField = ouster::sdk::core::ChanField;

NebulaPointCloudPtr nebula_point_cloud_from_lidar_scan(
  const LidarScan & scan, const XYZLut & lut, const FieldOfView<float, Degrees> & fov)
{
  if (!scan.has_field(ChanField::RANGE) || scan.w == 0 || scan.h == 0) {
    return std::make_shared<NebulaPointCloud>();
  }

  const auto range_img = scan.field<uint32_t>(ChanField::RANGE);
  const ouster::sdk::core::PointCloudXYZd xyz = ouster::sdk::core::cartesian(range_img, lut);

  const bool dual = scan.sensor_info->num_returns() > 1;
  const Eigen::Index h = static_cast<Eigen::Index>(scan.h);
  const Eigen::Index w = static_cast<Eigen::Index>(scan.w);

  uint64_t base_ts = 0U;
  const int first_col = scan.get_first_valid_column();
  if (first_col >= 0 && first_col < static_cast<int>(scan.w)) {
    base_ts = scan.timestamp()(first_col);
  }

  auto cloud = std::make_shared<NebulaPointCloud>();
  cloud->reserve(static_cast<size_t>(scan.h * scan.w));

  Eigen::ArrayX<uint32_t> status = scan.status();

  for (Eigen::Index row = 0; row < h; ++row) {
    for (Eigen::Index col = 0; col < w; ++col) {
      // Skip invalid columns.
      if ((status(col) & 0x01) == 0) {
        continue;
      }

      const uint32_t r_mm = range_img(row, col);
      if (r_mm == 0U) {
        continue;
      }

      const size_t i =
        static_cast<size_t>(row) * static_cast<size_t>(scan.w) + static_cast<size_t>(col);

      Eigen::Vector3d pt = xyz.row(i);

      const float altitude_deg = scan.sensor_info->beam_altitude_angles[row];
      if (altitude_deg < fov.elevation.start || altitude_deg > fov.elevation.end) {
        continue;
      }

      // TODO(unaal) could we read this directly from beam_azimuth_angles?
      const float azimuth_deg =
        normalize_azimuth_deg(static_cast<float>(rad2deg(std::atan2(pt.y(), pt.x()))));
      if (azimuth_deg < fov.azimuth.start || azimuth_deg > fov.azimuth.end) {
        continue;
      }

      NebulaPoint tgt_pt{};
      tgt_pt.x = static_cast<float>(pt.x());
      tgt_pt.y = static_cast<float>(pt.y());
      tgt_pt.z = static_cast<float>(pt.z());
      if (scan.has_field(ChanField::REFLECTIVITY)) {
        tgt_pt.intensity = scan.field<uint8_t>(ChanField::REFLECTIVITY)(row, col);
      } else if (scan.has_field(ChanField::SIGNAL)) {
        const auto v = scan.field<uint16_t>(ChanField::SIGNAL)(row, col);
        tgt_pt.intensity = static_cast<std::uint8_t>(std::min<uint16_t>(255U, v >> 4));
      }
      // TODO(unaal) retrieve return type from the sensor_info config
      tgt_pt.return_type =
        static_cast<std::uint8_t>(dual ? ReturnType::LAST : ReturnType::STRONGEST);
      tgt_pt.channel = static_cast<std::uint16_t>(row);
      tgt_pt.azimuth = deg2rad(azimuth_deg);
      tgt_pt.elevation = deg2rad(altitude_deg);
      tgt_pt.distance = static_cast<float>(r_mm) * static_cast<float>(RANGE_UNIT);

      const uint64_t col_ts = scan.timestamp()(col);
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
