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

#include "nebula_core_decoders/angles.hpp"

#include <gtest/gtest.h>

#include <cmath>

using nebula::drivers::angle_is_between;
using nebula::drivers::AnglePair;
using nebula::drivers::AngleRange;
using nebula::drivers::CentiDegrees;
using nebula::drivers::DeciDegrees;
using nebula::drivers::Degrees;
using nebula::drivers::FieldOfView;
using nebula::drivers::MilliDegrees;
using nebula::drivers::normalize_angle;
using nebula::drivers::Radians;

// ============================================================================
// AngleRange::extent tests
// ============================================================================

TEST(AngleRangeTest, ExtentSimpleRange)
{
  AngleRange<int, Degrees> range{0, 180};
  EXPECT_EQ(range.extent(), 180);
}

TEST(AngleRangeTest, ExtentFullCircle)
{
  AngleRange<int, Degrees> range{0, 360};
  EXPECT_EQ(range.extent(), 360);
}

TEST(AngleRangeTest, ExtentCrossingZero)
{
  // Range that crosses the 0/360 boundary
  AngleRange<int, Degrees> range{350, 10};
  EXPECT_EQ(range.extent(), 20);  // 360 - 350 + 10 = 20
}

TEST(AngleRangeTest, ExtentCrossingZeroLarger)
{
  AngleRange<int, Degrees> range{270, 90};
  EXPECT_EQ(range.extent(), 180);  // 360 - 270 + 90 = 180
}

TEST(AngleRangeTest, ExtentMilliDegrees)
{
  AngleRange<int32_t, MilliDegrees> range{0, 180'000};
  EXPECT_EQ(range.extent(), 180'000);

  AngleRange<int32_t, MilliDegrees> range_wrap{350'000, 10'000};
  EXPECT_EQ(range_wrap.extent(), 20'000);  // Crosses zero
}

TEST(AngleRangeTest, ExtentRadians)
{
  AngleRange<double, Radians> range{0.0, M_PI};
  EXPECT_DOUBLE_EQ(range.extent(), M_PI);

  AngleRange<double, Radians> range_wrap{1.5 * M_PI, 0.5 * M_PI};
  EXPECT_DOUBLE_EQ(range_wrap.extent(), M_PI);  // Crosses zero
}

// ============================================================================
// angle_is_between tests - basic cases
// ============================================================================

TEST(AngleIsBetweenTest, AngleInsideRange)
{
  EXPECT_TRUE(angle_is_between(0, 180, 90));
  EXPECT_TRUE(angle_is_between(10, 350, 180));
  EXPECT_TRUE(angle_is_between(0, 360, 180));
}

TEST(AngleIsBetweenTest, AngleOutsideRange)
{
  EXPECT_FALSE(angle_is_between(0, 90, 180));
  EXPECT_FALSE(angle_is_between(100, 200, 50));
  EXPECT_FALSE(angle_is_between(100, 200, 250));
}

TEST(AngleIsBetweenTest, AngleAtBoundaryInclusive)
{
  // Default: both boundaries inclusive
  EXPECT_TRUE(angle_is_between(0, 180, 0));    // At start
  EXPECT_TRUE(angle_is_between(0, 180, 180));  // At end
}

TEST(AngleIsBetweenTest, AngleAtBoundaryStartExclusive)
{
  EXPECT_FALSE(angle_is_between(0, 180, 0, false, true));   // Start exclusive
  EXPECT_TRUE(angle_is_between(0, 180, 180, false, true));  // End inclusive
  EXPECT_TRUE(angle_is_between(0, 180, 90, false, true));   // Inside
}

TEST(AngleIsBetweenTest, AngleAtBoundaryEndExclusive)
{
  EXPECT_TRUE(angle_is_between(0, 180, 0, true, false));     // Start inclusive
  EXPECT_FALSE(angle_is_between(0, 180, 180, true, false));  // End exclusive
  EXPECT_TRUE(angle_is_between(0, 180, 90, true, false));    // Inside
}

TEST(AngleIsBetweenTest, AngleAtBoundaryBothExclusive)
{
  EXPECT_FALSE(angle_is_between(0, 180, 0, false, false));    // Start
  EXPECT_FALSE(angle_is_between(0, 180, 180, false, false));  // End
  EXPECT_TRUE(angle_is_between(0, 180, 90, false, false));    // Inside
}

// ============================================================================
// angle_is_between tests - wraparound cases
// ============================================================================

TEST(AngleIsBetweenTest, WrapAroundAngleInside)
{
  // Range wraps around: 350 -> 10 (through 0)
  EXPECT_TRUE(angle_is_between(350, 10, 355));  // Between 350 and 360
  EXPECT_TRUE(angle_is_between(350, 10, 0));    // At zero
  EXPECT_TRUE(angle_is_between(350, 10, 5));    // Between 0 and 10
}

TEST(AngleIsBetweenTest, WrapAroundAngleOutside)
{
  // Range wraps around: 350 -> 10 (through 0)
  EXPECT_FALSE(angle_is_between(350, 10, 180));  // Middle of the excluded region
  EXPECT_FALSE(angle_is_between(350, 10, 20));   // Just outside end
  EXPECT_FALSE(angle_is_between(350, 10, 340));  // Just outside start
}

TEST(AngleIsBetweenTest, WrapAroundAtBoundary)
{
  EXPECT_TRUE(angle_is_between(350, 10, 350));  // At start
  EXPECT_TRUE(angle_is_between(350, 10, 10));   // At end
}

TEST(AngleIsBetweenTest, WrapAroundHalfCircle)
{
  EXPECT_TRUE(angle_is_between(270, 90, 0));
  EXPECT_TRUE(angle_is_between(270, 90, 45));
  EXPECT_TRUE(angle_is_between(270, 90, 315));
  EXPECT_FALSE(angle_is_between(270, 90, 180));
  EXPECT_FALSE(angle_is_between(270, 90, 135));
}

// ============================================================================
// normalize_angle tests
// ============================================================================

TEST(NormalizeAngleTest, AlreadyNormalized)
{
  EXPECT_DOUBLE_EQ(normalize_angle(0.0, 360.0), 0.0);
  EXPECT_DOUBLE_EQ(normalize_angle(180.0, 360.0), 180.0);
  EXPECT_DOUBLE_EQ(normalize_angle(359.0, 360.0), 359.0);
}

TEST(NormalizeAngleTest, NegativeAngle)
{
  EXPECT_DOUBLE_EQ(normalize_angle(-90.0, 360.0), 270.0);
  EXPECT_DOUBLE_EQ(normalize_angle(-180.0, 360.0), 180.0);
  EXPECT_DOUBLE_EQ(normalize_angle(-360.0, 360.0), 0.0);
  EXPECT_DOUBLE_EQ(normalize_angle(-450.0, 360.0), 270.0);  // -450 + 2*360 = 270
}

TEST(NormalizeAngleTest, LargePositiveAngle)
{
  EXPECT_DOUBLE_EQ(normalize_angle(360.0, 360.0), 0.0);
  EXPECT_DOUBLE_EQ(normalize_angle(450.0, 360.0), 90.0);
  EXPECT_DOUBLE_EQ(normalize_angle(720.0, 360.0), 0.0);
  EXPECT_DOUBLE_EQ(normalize_angle(810.0, 360.0), 90.0);  // 810 - 2*360 = 90
}

TEST(NormalizeAngleTest, Radians)
{
  double two_pi = 2.0 * M_PI;
  EXPECT_DOUBLE_EQ(normalize_angle(0.0, two_pi), 0.0);
  EXPECT_NEAR(normalize_angle(-M_PI, two_pi), M_PI, 1e-10);
  EXPECT_NEAR(normalize_angle(3.0 * M_PI, two_pi), M_PI, 1e-10);
  EXPECT_NEAR(normalize_angle(4.0 * M_PI, two_pi), 0.0, 1e-10);
}

TEST(NormalizeAngleTest, ScaledDegrees)
{
  // CentiDegrees: max = 36000
  EXPECT_EQ(normalize_angle(0, 36'000), 0);
  EXPECT_EQ(normalize_angle(36'000, 36'000), 0);
  EXPECT_EQ(normalize_angle(45'000, 36'000), 9'000);   // 45000 - 36000 = 9000
  EXPECT_EQ(normalize_angle(-9'000, 36'000), 27'000);  // -9000 + 36000 = 27000
}

// ============================================================================
// Type definitions tests
// ============================================================================

TEST(AngleTypesTest, CircleModulus)
{
  EXPECT_DOUBLE_EQ(Radians::circle_modulus, 2 * M_PI);
  EXPECT_DOUBLE_EQ(Degrees::circle_modulus, 360.0);
  EXPECT_DOUBLE_EQ(DeciDegrees::circle_modulus, 3600.0);
  EXPECT_DOUBLE_EQ(CentiDegrees::circle_modulus, 36'000.0);
  EXPECT_DOUBLE_EQ(MilliDegrees::circle_modulus, 360'000.0);
}

TEST(AngleTypesTest, AnglePairConstruction)
{
  AnglePair<double, Radians> pair{M_PI / 4, M_PI / 6};
  EXPECT_DOUBLE_EQ(pair.azimuth, M_PI / 4);
  EXPECT_DOUBLE_EQ(pair.elevation, M_PI / 6);
}

TEST(AngleTypesTest, FieldOfViewConstruction)
{
  FieldOfView<int, Degrees> fov;
  fov.azimuth = {0, 180};
  fov.elevation = {-15, 15};

  EXPECT_EQ(fov.azimuth.start, 0);
  EXPECT_EQ(fov.azimuth.end, 180);
  EXPECT_EQ(fov.elevation.start, -15);
  EXPECT_EQ(fov.elevation.end, 15);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
