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

#include <cmath>
namespace nebula::drivers
{

template <typename T>
struct AnglePair
{
  T azimuth;
  T elevation;
};

template <typename T>
struct AngleRange
{
  T min;
  T max;

  [[nodiscard]] T extent() const { return max - min; }
};

template <typename T>
struct FieldOfView
{
  AngleRange<T> azimuth;
  AngleRange<T> elevation;
};

/**
 * @brief Tests if `angle` is in the region of the circle defined by `start_angle` and `end_angle`.
 * Notably, `end_angle` can be smaller than `start_angle`, in which case the region passes over the
 * 360/0 deg bound. This function is unit-independent (but all angles have to have the same unit),
 * so degrees, radians, and arbitrary scale factors can be used.
 */
template <typename T>
bool angle_is_between(
  T start_angle, T end_angle, T angle, bool start_inclusive = true, bool end_inclusive = true)
{
  if (!start_inclusive && angle == start_angle) return false;
  if (!end_inclusive && angle == end_angle) return false;

  return (start_angle <= angle && angle <= end_angle) ||
         ((end_angle < start_angle) && (angle <= end_angle || start_angle <= angle));
}

/**
 * @brief Normalizes an angle to the interval [0; max_angle]. This function is unit-independent.
 * `max_angle` is 360 for degrees, 2 * M_PI for radians, and the corresponding scaled value for
 * scaled units such as centi-degrees (36000).
 */
template <typename T>
T normalize_angle(T angle, T max_angle)
{
  T factor = std::floor((1.0 * angle) / max_angle);
  return angle - (factor * max_angle);
}

}  // namespace nebula::drivers
