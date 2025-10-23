// Copyright 2025 TIER IV, Inc.
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

#include <cstdint>

namespace nebula::drivers
{

/// @brief A timestamped IMU reading.
struct ImuReading
{
  /// @brief Absolute timestamp in nanoseconds
  uint64_t absolute_timestamp_ns{0};
  /// @brief Acceleration in m/s^2 on the IMU X axis
  float accel_mps2_x{0.0F};
  /// @brief Acceleration in m/s^2 on the IMU Y axis
  float accel_mps2_y{0.0F};
  /// @brief Acceleration in m/s^2 on the IMU Z axis
  float accel_mps2_z{0.0F};
  /// @brief Angular velocity in rad/s around the IMU X axis
  float ang_vel_rps_x{0.0F};
  /// @brief Angular velocity in rad/s around the IMU Y axis
  float ang_vel_rps_y{0.0F};
  /// @brief Angular velocity in rad/s around the IMU Z axis
  float ang_vel_rps_z{0.0F};
};

}  // namespace nebula::drivers
