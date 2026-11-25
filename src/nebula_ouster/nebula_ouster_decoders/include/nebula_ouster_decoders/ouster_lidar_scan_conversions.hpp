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

#ifndef NEBULA_OUSTER_DECODERS__OUSTER_LIDAR_SCAN_CONVERSIONS_HPP_
#define NEBULA_OUSTER_DECODERS__OUSTER_LIDAR_SCAN_CONVERSIONS_HPP_

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_decoders/angles.hpp>

#include <ouster/core/lidar_frame.h>
#include <ouster/core/types.h>
#include <ouster/core/xyzlut.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace nebula::drivers
{

/// @brief Converts completed Ouster SDK @c LidarFrame objects into Nebula point clouds.
///
/// Everything that depends only on the sensor geometry and the FoV crop (XYZ lookup table,
/// per-pixel azimuth/elevation and the FoV mask) is computed once at construction so that the
/// per-scan conversion does no trigonometry and no field-name lookups per pixel.
class OusterPointCloudConverter
{
public:
  /// @param sensor_info Sensor metadata used to build the lookup tables.
  /// @param fov Angular crop in sensor spherical coordinates (degrees).
  OusterPointCloudConverter(
    const ouster::sdk::core::SensorInfo & sensor_info, const FieldOfView<float, Degrees> & fov);

  /// @brief Convert a completed lidar frame into a point cloud.
  /// @param scan Staggered lidar frame from @c ouster::sdk::core::FrameBatcher.
  /// @return Shared point cloud; empty if @p scan has no RANGE field, has no valid columns or does
  ///         not match the dimensions of the sensor this converter was built for.
  [[nodiscard]] NebulaPointCloudPtr convert(const ouster::sdk::core::LidarFrame & scan) const;

private:
  size_t h_;
  size_t w_;
  ReturnType return_type_;
  ouster::sdk::core::XYZLut lut_;
  /// Per-pixel (row-major, h x w) azimuth of the beam direction in [0, 2pi).
  std::vector<float> azimuth_rad_;
  /// Per-row elevation in radians.
  std::vector<float> elevation_rad_;
  /// Per-pixel (row-major, h x w) flag: 1 if the pixel lies inside the configured FoV.
  std::vector<uint8_t> in_fov_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_OUSTER_DECODERS__OUSTER_LIDAR_SCAN_CONVERSIONS_HPP_
