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

// # --8<-- [start:include]
#include "nebula_core_common/nebula_common.hpp"
#include "nebula_core_common/util/angles.hpp"
// # --8<-- [end:include]

#include <cassert>
#include <cmath>

namespace nebula::drivers::examples
{

void angle_utilities_usage_example()
{
  // # --8<-- [start:usage]
  const float azimuth_deg = 45.0F;

  const auto azimuth_rad = deg2rad(azimuth_deg);
  const float fov_min_rad = 0.0F;
  const auto fov_max_rad = deg2rad(90.0F);
  assert(angle_is_between(fov_min_rad, fov_max_rad, azimuth_rad));

  // Normalize any angle to [0, 360) or [0, 2pi) (depending on the chosen max_angle):
  const float azimuth_deg_norm = normalize_angle(azimuth_deg, 360.0F);
  assert(azimuth_deg_norm >= 0.0F && azimuth_deg_norm < 360.0F);
  const float azimuth_rad_norm = normalize_angle(azimuth_rad, 2 * M_PI);
  assert(azimuth_rad_norm >= 0.0F && azimuth_rad_norm < 2 * M_PI);
  // # --8<-- [end:usage]

  (void)fov_min_rad;
  (void)fov_max_rad;
  (void)azimuth_deg_norm;
  (void)azimuth_rad_norm;
}

}  // namespace nebula::drivers::examples
