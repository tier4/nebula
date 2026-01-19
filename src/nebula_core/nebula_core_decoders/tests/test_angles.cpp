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

namespace nebula::drivers
{

constexpr float two_pi = 2.0F * M_PIf;

TEST(AngleIsBetweenTest, BasicTest)
{
  // Note that [0, 0] and [0, 360] are both considered to cover the full circle.
  EXPECT_TRUE(angle_is_between(0, 0, 0));
  EXPECT_TRUE(angle_is_between(0, 0, 180));
  EXPECT_TRUE(angle_is_between(0, 360, 180));
  EXPECT_FALSE(angle_is_between(0, 180, 270));
  EXPECT_FALSE(angle_is_between(270, 90, 180));
  EXPECT_TRUE(angle_is_between(90, 270, 180));
}

TEST(AngleIsBetweenTest, BoundaryTest)
{
  // Non-360-degree sector
  EXPECT_TRUE(angle_is_between(0, 10, 0));
  EXPECT_TRUE(angle_is_between(0, 10, 10));
  EXPECT_FALSE(angle_is_between(0, 10, 0, false));
  EXPECT_FALSE(angle_is_between(0, 10, 10, true, false));
  EXPECT_TRUE(angle_is_between(0, 10, 5, false, false));
}

TEST(AngleIsBetweenTest, WrapAroundTest)
{
  EXPECT_FALSE(angle_is_between(10, 0, 5));
  EXPECT_TRUE(angle_is_between(10, 0, 15));
}

TEST(AngleIsBetweenTest, StartEqualsEndTest)
{
  EXPECT_TRUE(angle_is_between(0, 0, 10));
  EXPECT_TRUE(angle_is_between(10, 10, 10));
  EXPECT_TRUE(angle_is_between(10, 10, 10, false));
  EXPECT_TRUE(angle_is_between(10, 10, 10, true, false));
  EXPECT_FALSE(angle_is_between(10, 10, 10, false, false));
}

TEST(NormalizeAngleTest, BasicTest)
{
  EXPECT_EQ(normalize_angle(0, 10), 0);
  EXPECT_EQ(normalize_angle(1, 10), 1);
  EXPECT_EQ(normalize_angle(10, 10), 0);
  EXPECT_EQ(normalize_angle(11, 10), 1);
}

TEST(NormalizeAngleTest, MultipleOfMaxAngleTest)
{
  EXPECT_EQ(normalize_angle(20, 10), 0);
  EXPECT_EQ(normalize_angle(21, 10), 1);
}

TEST(NormalizeAngleTest, NegativeTest)
{
  EXPECT_EQ(normalize_angle(-1, 10), 9);
  EXPECT_EQ(normalize_angle(-11, 10), 9);
  EXPECT_EQ(normalize_angle(-20, 10), 0);
  EXPECT_EQ(normalize_angle(-21, 10), 9);
}

TEST(NormalizeAngleTest, FloatTest)
{
  EXPECT_FLOAT_EQ(normalize_angle(0.0F, two_pi), 0.0F);
  EXPECT_FLOAT_EQ(normalize_angle(1.0F, two_pi), 1.0F);
  EXPECT_FLOAT_EQ(normalize_angle(two_pi, two_pi), 0.0F);

  constexpr float epsilon = 1e-6F;
  EXPECT_FLOAT_EQ(normalize_angle(two_pi - epsilon, two_pi), two_pi - epsilon);
  EXPECT_FLOAT_EQ(normalize_angle(two_pi + 1.0F, two_pi), 1.0F);
}

TEST(NormalizeAngleTest, FloatMultipleOfMaxAngleTest)
{
  EXPECT_FLOAT_EQ(normalize_angle(2 * two_pi + 1.0F, two_pi), 1.0F);
}

TEST(NormalizeAngleTest, FloatNegativeTest)
{
  EXPECT_FLOAT_EQ(normalize_angle(-1.0F, two_pi), two_pi - 1.0F);
  EXPECT_FLOAT_EQ(normalize_angle(-two_pi - 1.0F, two_pi), two_pi - 1.0F);
  EXPECT_FLOAT_EQ(normalize_angle(-2 * two_pi - 1.0F, two_pi), two_pi - 1.0F);
}

}  // namespace nebula::drivers
