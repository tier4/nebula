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

#include <nebula_core_common/point_types.hpp>
#include <nebula_core_decoders/angles.hpp>

#include <ouster/core/lidar_frame.h>
#include <ouster/core/types.h>
#include <ouster/core/xyzlut.h>

#include <memory>

namespace nebula::drivers
{

/// @brief Convert a completed Ouster SDK @c LidarFrame into Nebula's dense point cloud type.
/// @param scan Staggered lidar frame from @c ouster::sdk::core::FrameBatcher.
/// @param lut XYZ lookup table constructed from the sensor's @c SensorInfo.
/// @param fov Angular crop in sensor spherical coordinates (degrees).
/// @return Shared point cloud; empty if @p scan has no RANGE field or zero size.
NebulaPointCloudPtr nebula_point_cloud_from_lidar_scan(
  const ouster::sdk::core::LidarFrame & scan, const ouster::sdk::core::XYZLut & lut,
  const FieldOfView<float, Degrees> & fov);

}  // namespace nebula::drivers

#endif  // NEBULA_OUSTER_DECODERS__OUSTER_LIDAR_SCAN_CONVERSIONS_HPP_
