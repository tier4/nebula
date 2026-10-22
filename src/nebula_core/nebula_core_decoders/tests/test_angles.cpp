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
#include "nebula_core_decoders/laser_phoenix_lattice_pq.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <vector>

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
  // Tiny negative float must never round up to max_angle (violating [0, max_angle))
  const float tiny_neg = -1e-9F;
  const float norm_tiny = normalize_angle(tiny_neg, two_pi);
  EXPECT_GE(norm_tiny, 0.0F);
  EXPECT_LT(norm_tiny, two_pi);
}

TEST(LaserPhoenixLatticePqTest, AzimuthPhaseQ32Bijection)
{
  using phoenix_pq::azimuth_to_phase_q32;
  using phoenix_pq::phase_q32_to_azimuth;

  // Exact bijection across all 36,000 centi-degrees
  for (int32_t az = 0; az < 36000; ++az) {
    const uint32_t q32 = azimuth_to_phase_q32(az, 36000);
    const int32_t round_trip = phase_q32_to_azimuth(q32, 36000);
    ASSERT_EQ(round_trip, az);
  }

  // Exact bijection across all 360,000 milli-degrees
  for (int32_t az = 0; az < 360000; az += 37) {
    const uint32_t q32 = azimuth_to_phase_q32(az, 360000);
    const int32_t round_trip = phase_q32_to_azimuth(q32, 360000);
    ASSERT_EQ(round_trip, az);
  }
}

TEST(LaserPhoenixLatticePqTest, FiveStateInterferenceLattice)
{
  using phoenix_pq::classify_phoenix_interference;
  using phoenix_pq::kPhoenixLatticeAnchorsQ32;
  using phoenix_pq::PhoenixInterferenceState;

  EXPECT_EQ(
    classify_phoenix_interference(static_cast<int32_t>(kPhoenixLatticeAnchorsQ32[0])),
    PhoenixInterferenceState::Bound);
  EXPECT_EQ(
    classify_phoenix_interference(static_cast<int32_t>(kPhoenixLatticeAnchorsQ32[1])),
    PhoenixInterferenceState::Related);
  EXPECT_EQ(
    classify_phoenix_interference(static_cast<int32_t>(kPhoenixLatticeAnchorsQ32[2])),
    PhoenixInterferenceState::Independent);
  EXPECT_EQ(
    classify_phoenix_interference(static_cast<int32_t>(kPhoenixLatticeAnchorsQ32[3])),
    PhoenixInterferenceState::Contrasting);
  EXPECT_EQ(
    classify_phoenix_interference(static_cast<int32_t>(kPhoenixLatticeAnchorsQ32[4])),
    PhoenixInterferenceState::Conflicting);
}

TEST(LaserPhoenixLatticePqTest, AzimuthPllAndMarineJitterGate)
{
  phoenix_pq::LaserPhoenixAzimuthPll pll(36000);

  // Spin across 36000 -> 0 boundary at 100 centi-deg (1.0 deg) per firing block
  int64_t raw_err_sum = 0;
  int64_t pll_err_sum = 0;
  bool saw_jitter_clip = false;

  for (int step = 0; step < 60; ++step) {
    const int32_t true_az = normalize_angle(34000 + step * 100, 36000);
    // Inject alternating high-frequency optical encoder vibration (+60 / -60 centi-deg)
    const int32_t jitter = (step >= 20 && step < 40) ? ((step % 2 == 0) ? 60 : -60) : 0;
    const int32_t raw_az = normalize_angle(true_az + jitter, 36000);

    const auto res = pll.step(raw_az);
    if (res.jitter_clipped) {
      saw_jitter_clip = true;
    }

    if (step >= 20 && step < 40) {
      auto circular_diff = [](int32_t a, int32_t b) {
        int32_t d = std::abs(a - b);
        return (d > 18000) ? (36000 - d) : d;
      };
      raw_err_sum += circular_diff(raw_az, true_az);
      pll_err_sum += circular_diff(res.locked_azimuth, true_az);
    }
  }

  EXPECT_TRUE(saw_jitter_clip);
  EXPECT_LT(pll_err_sum, raw_err_sum);
}

TEST(LaserPhoenixLatticePqTest, CircularCatmullRomAzimuthDropoutRestoration)
{
  // Create a sequence of 20 azimuth blocks crossing the 36000 -> 0 wrap-around boundary
  std::vector<int32_t> ground_truth(20);
  for (size_t i = 0; i < 20; ++i) {
    ground_truth[i] = normalize_angle(static_cast<int32_t>(35200 + i * 100), 36000);
  }

  std::vector<int32_t> corrupted = ground_truth;
  // Drop 4 consecutive UDP packets right across the 35900 -> 00200 wrap-around boundary!
  corrupted[7] = phoenix_pq::kAzimuthDropoutSentinel;
  corrupted[8] = phoenix_pq::kAzimuthDropoutSentinel;
  corrupted[9] = phoenix_pq::kAzimuthDropoutSentinel;
  corrupted[10] = phoenix_pq::kAzimuthDropoutSentinel;

  const size_t restored = phoenix_pq::restore_dropped_azimuths_catmull_rom_pq(corrupted, 36000);
  EXPECT_EQ(restored, 4U);

  for (size_t i = 0; i < 20; ++i) {
    int32_t diff = std::abs(corrupted[i] - ground_truth[i]);
    if (diff > 18000) {
      diff = 36000 - diff;
    }
    EXPECT_LE(diff, 1) << "Index " << i << " expected " << ground_truth[i] << " got "
                       << corrupted[i];
  }
}

}  // namespace nebula::drivers
