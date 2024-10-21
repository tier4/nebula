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
#include "nebula_common/nebula_common.hpp"

#include <cstdint>

namespace nebula::drivers
{

using TutorialSensorConfiguration = typename nebula::drivers::HesaiSensorConfiguration;

struct ExampleSensorConfiguration : public LidarConfigurationBase
{
  /// @brief UDP Port for GNSS data packets
  uint16_t gnss_port;
  /// @brief At which angle the sensor syncs to the 1.00000...s mark
  double scan_phase;
  /// @brief The distance in m below which two points from the same ray are fused into one
  double dual_return_distance_threshold;
  /// @brief The motor RPM. This directly influences the output frequency of pointclouds
  uint16_t rotation_speed;
  /// @brief The angle at which to start outputting points
  uint16_t cloud_min_angle;
  /// @brief The angle at which to stop outputting points
  uint16_t cloud_max_angle;
};

}  // namespace nebula::drivers
