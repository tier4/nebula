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
#include <cstdint>

namespace nebula::drivers
{

struct Radians
{
  static constexpr double circle_modulus = 2 * M_PI;
};

template <uint32_t Subdivisions>
struct ScaledDegrees
{
  static constexpr double circle_modulus = 360 * Subdivisions;
};

using Degrees = ScaledDegrees<1>;
using DeciDegrees = ScaledDegrees<10>;
using CentiDegrees = ScaledDegrees<100>;
using MilliDegrees = ScaledDegrees<1000>;

template <typename T, typename AngleUnit>
struct AnglePair
{
  T azimuth;
  T elevation;
};

/// @brief A range defined by a start and end angle. Crossing the 0/circle_modulus boundary (end <
/// start) is allowed.
/// @tparam T The type of the angle.
/// @tparam AngleUnit The unit of the angle.
template <typename T, typename AngleUnit>
struct AngleRange
{
  T start;
  T end;

  /// @brief The extent of the range, taking into account the 0/circle_modulus boundary. Will always
  /// be positive.
  [[nodiscard]] T extent() const
  {
    return (end < start) ? AngleUnit::circle_modulus - start + end : end - start;
  }
};

template <typename T, typename AngleUnit>
struct FieldOfView
{
  AngleRange<T, AngleUnit> azimuth;
  AngleRange<T, AngleUnit> elevation;
};

/**
 * @brief Tests if `angle` is in the angular sector defined by `start_angle` and `end_angle`.
 *
 * Assumes that all three angles are normalized to the interval [0; max_angle) before comparison.
 * This function is unit-independent (max_angle can be 360 for degrees, 2 * M_PI for radians, 36000
 * for centi-degrees, etc.) as long as the above assumption is fulfilled.
 *
 * If `start_angle` equals `end_angle`, the sector is considered to cover the full circle. In this
 * case, an `angle` is always considered to be in the sector, except when it coincides with the
 * `start_angle`/`end_angle` and both boundaries are exclusive. There is no way to define a
 * zero-width sector.
 *
 * When `start_angle` and `end_angle` differ, `angle` is considered to be in the sector if
 * - `start_angle <= angle <= end_angle`, or
 * - `end_angle < start_angle` and (`angle <= end_angle` or `start_angle <= angle`)
 *
 * Examples:
 * ```c++
 * assert(angle_is_between(90, 270, 180) == true);
 * assert(angle_is_between(270, 90, 0) == true);  // wrap-around case
 * assert(angle_is_between(0, 0, 0) == true);
 * assert(angle_is_between(0, 0, 0, false, false) == false);
 * ```
 */
template <typename T>
bool angle_is_between(
  T start_angle, T end_angle, T angle, bool start_inclusive = true, bool end_inclusive = true)
{
  // Note: comments in this function refer to 360 degrees as the max_angle for simplicity, but the
  // logic works equally for other units, such as radians or centi-degrees.
  // 360-degree sector, angle coincides with boundary
  if (start_angle == end_angle && start_angle == angle) return start_inclusive || end_inclusive;

  // 360-degree sector, angle is fully inside
  if (start_angle == end_angle) return true;

  // Non-360-degree sector, angle coincides with boundary
  if (!start_inclusive && angle == start_angle) return false;
  if (!end_inclusive && angle == end_angle) return false;

  // Sector wraps around at 360/0
  if (end_angle < start_angle) return (angle <= end_angle) || (start_angle <= angle);

  // Sector does not wrap around
  return (start_angle <= angle) && (angle <= end_angle);
}

/**
 * @brief Normalizes an angle to the interval [0; max_angle). This function is unit-independent.
 * `max_angle` is 360 for degrees, 2 * M_PI for radians, and the corresponding scaled value for
 * scaled units such as centi-degrees (36000).
 *
 * Mathematically, the normalization is a modulo operation, yielding an angle with winding number 0.
 */
template <typename T>
T normalize_angle(T angle, T max_angle)
{
  T factor = std::floor((1.0 * angle) / max_angle);
  return angle - (factor * max_angle);
}

}  // namespace nebula::drivers
