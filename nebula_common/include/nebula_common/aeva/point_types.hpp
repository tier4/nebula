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

#include <pcl/point_types.h>

namespace nebula::drivers::aeva
{

struct EIGEN_ALIGN16 PointXYZVIRCAEDT
{
  float x;
  float y;
  float z;
  float range_rate;
  uint8_t intensity;
  uint8_t return_type;
  uint16_t channel;
  float azimuth;
  float elevation;
  float distance;
  uint32_t time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace nebula::drivers::aeva

POINT_CLOUD_REGISTER_POINT_STRUCT(  // NOLINT
  nebula::drivers::aeva::PointXYZVIRCAEDT,
  (float, x,
   x)(float, y, y)(float, z, z)(float, range_rate, range_rate)(std::uint8_t, intensity, intensity)(
    std::uint8_t, return_type,
    return_type)(std::uint16_t, channel, channel)(float, azimuth, azimuth)(
    float, elevation, elevation)(float, distance, distance)(std::uint32_t, time_stamp, time_stamp))
