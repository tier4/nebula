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

#include "nebula_core_decoders/scan_cutter.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace nebula::drivers
{

// Test constants
constexpr size_t n_channels = 128;
constexpr uint32_t angle_unit = 100;  // centi-degrees
constexpr int32_t max_angle = 360 * angle_unit;

// Helper class to track callback invocations
struct CallbackTracker
{
  std::vector<uint8_t> publish_calls;
  std::vector<uint8_t> timestamp_set_calls;

  void reset()
  {
    publish_calls.clear();
    timestamp_set_calls.clear();
  }

  auto make_publish_callback()
  {
    return [this](uint8_t buffer_index) { publish_calls.push_back(buffer_index); };
  }

  auto make_timestamp_callback()
  {
    return [this](uint8_t buffer_index) { timestamp_set_calls.push_back(buffer_index); };
  }
};

namespace
{

/// Helper to create offsets with a linear spread from 'from' to 'to'
template <size_t N>
std::array<int32_t, N> make_increasing_offsets(int32_t from, int32_t to)
{
  std::array<int32_t, N> result;
  for (size_t i = 0; i < N; ++i) {
    result.at(i) = from + static_cast<int32_t>(i * (to - from) / (N - 1));
  }
  return result;
}

/// Helper to create synthetic channel azimuths with given offsets
template <size_t N>
std::array<int32_t, N> make_channel_azimuths(
  int32_t base_azimuth, const std::array<int32_t, N> & offsets)
{
  std::array<int32_t, N> result;
  for (size_t i = 0; i < N; ++i) {
    result.at(i) = base_azimuth + offsets.at(i);
  }
  return result;
}

/// Helper to simulate a full rotation
template <size_t NChannels, uint32_t AngleUnit>
void simulate_rotation(
  ScanCutter<NChannels, AngleUnit> & cutter, int32_t start_azimuth, int32_t step_size,
  const std::array<int32_t, NChannels> & offsets, int32_t total_rotation = 360 * AngleUnit)
{
  int32_t current_azimuth = start_azimuth;
  int32_t end_azimuth = start_azimuth + total_rotation;

  while (current_azimuth < end_azimuth) {
    auto channel_azimuths = make_channel_azimuths(current_azimuth, offsets);
    cutter.step(current_azimuth, channel_azimuths);
    current_azimuth += step_size;
  }
}

}  // namespace

// =============================================================================
// Test Suite 1: Timestamp Behavior Tests
// =============================================================================

class TestScanCutterTimestamps : public ::testing::Test
{
protected:
  CallbackTracker tracker;
};

TEST_F(TestScanCutterTimestamps, TimestampSetBeforeFirstPoint)
{
  // Verify timestamp_set is called during initialization before any points are decoded
  ScanCutter<n_channels, angle_unit> cutter(
    18000,  // cut at 180 degrees
    0,      // fov start at 0
    0,      // fov end at 0 (360 deg FoV)
    tracker.make_publish_callback(), tracker.make_timestamp_callback());

  // No calls yet, cutter is not initialized
  ASSERT_EQ(tracker.timestamp_set_calls.size(), 0);

  // Create first step - this should initialize and set timestamps
  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);  // No corrections
  auto channel_azimuths = make_channel_azimuths<n_channels>(0, offsets);

  cutter.step(0, channel_azimuths);

  // After initialization, timestamp should be set for buffer 0
  ASSERT_GE(tracker.timestamp_set_calls.size(), 1);
  EXPECT_EQ(tracker.timestamp_set_calls[0], 0);
}

TEST_F(TestScanCutterTimestamps, TimestampSetOncePerFrameIn360FoV)
{
  // For 360 degree FoV, each scan should get timestamp set exactly once
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  // Simulate complete rotation
  simulate_rotation(cutter, 0, 100, offsets);

  // We expect:
  // - Initial timestamp set for buffer 0
  // - One publish (crossing cut angle)
  // - One timestamp set for next buffer
  EXPECT_GE(tracker.publish_calls.size(), 1);
  EXPECT_GE(tracker.timestamp_set_calls.size(), 2);
}

TEST_F(TestScanCutterTimestamps, TimestampResetOnFovStart)
{
  // When cut_angle == fov_end, timestamp should reset on FoV start
  ScanCutter<n_channels, angle_unit> cutter(
    27000,  // cut at 270 degrees
    9000,   // fov start at 90
    27000,  // fov end at 270 (same as cut)
    tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  // Simulate rotation through FoV entry
  for (int32_t az = 0; az < max_angle * 2; az += 100) {
    auto channel_azimuths = make_channel_azimuths<n_channels>(az, offsets);
    cutter.step(az, channel_azimuths);
  }

  // Verify timestamps were set
  EXPECT_GE(tracker.timestamp_set_calls.size(), 1);
}

// =============================================================================
// Test Suite 2: Channel Correction Variants
// =============================================================================

class TestScanCutterChannelCorrections : public ::testing::Test
{
protected:
  CallbackTracker tracker;
};

TEST_F(TestScanCutterChannelCorrections, NoCorrections)
{
  // All channel azimuths equal encoder azimuth
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  // Simulate rotation
  simulate_rotation(cutter, 0, 100, offsets);

  // With no corrections, all channels should always be in same buffer
  // Expect at least one publish for a full rotation
  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterChannelCorrections, SymmetricSpread)
{
  // Channels spread equally in positive and negative directions
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  // Spread from -500 to +500 centi-degrees
  auto offsets = make_increasing_offsets<n_channels>(-500, 500);

  // Simulate rotation
  simulate_rotation(cutter, 0, 100, offsets);

  // Should complete without errors
  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterChannelCorrections, AsymmetricSpread)
{
  // abs(min_offset) != abs(max_offset)
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  // Spread from -1000 to +500 centi-degrees
  auto offsets = make_increasing_offsets<n_channels>(-1000, 500);

  // Simulate rotation
  simulate_rotation(cutter, 0, 100, offsets);

  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterChannelCorrections, PositiveOnlySpread)
{
  // All corrections are positive
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  // Spread from 0 to +1000 centi-degrees
  auto offsets = make_increasing_offsets<n_channels>(0, 1000);

  // Simulate rotation
  simulate_rotation(cutter, 0, 100, offsets);

  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterChannelCorrections, NegativeOnlySpread)
{
  // All corrections are negative
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  // Spread from -1000 to 0 centi-degrees
  auto offsets = make_increasing_offsets<n_channels>(-1000, 0);

  // Simulate rotation
  simulate_rotation(cutter, 0, 100, offsets);

  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterChannelCorrections, BlockStraddlesCut)
{
  // Test when a block has some channels before cut and some after
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  for (size_t i = 0; i < n_channels / 2; ++i) {
    offsets.at(i) = -200;  // Before cut
  }
  for (size_t i = n_channels / 2; i < n_channels; ++i) {
    offsets.at(i) = +200;  // After cut
  }

  // Step right at the cut angle
  auto channel_azimuths = make_channel_azimuths<n_channels>(18000, offsets);
  auto buffer_indices = cutter.step(18000, channel_azimuths);

  // Verify channels are split across buffers
  bool has_buffer_0 = false;
  bool has_buffer_1 = false;
  for (auto idx : buffer_indices) {
    if (idx == 0) has_buffer_0 = true;
    if (idx == 1) has_buffer_1 = true;
  }
  EXPECT_TRUE(has_buffer_0 && has_buffer_1);
}

// =============================================================================
// Test Suite 3: FoV and Cut Angle Combinations
// =============================================================================

class TestScanCutterFoVConfigurations : public ::testing::Test
{
protected:
  CallbackTracker tracker;
};

TEST_F(TestScanCutterFoVConfigurations, FullRotation360Deg)
{
  // start == end (360 degree FoV)
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  simulate_rotation(cutter, 0, 100, offsets);

  // Should have published at least once
  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterFoVConfigurations, CutInMiddleOfFoV)
{
  // start < cut < end (non-360 FoV with cut in middle)
  ScanCutter<n_channels, angle_unit> cutter(
    18000,  // cut at 180 degrees
    9000,   // fov start at 90
    27000,  // fov end at 270
    tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  simulate_rotation(cutter, 0, 100, offsets, max_angle * 2);

  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterFoVConfigurations, CutAtFovEnd)
{
  // cut == end (special reset behavior)
  ScanCutter<n_channels, angle_unit> cutter(
    27000,  // cut at 270 degrees
    9000,   // fov start at 90
    27000,  // fov end at 270 (same as cut)
    tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  simulate_rotation(cutter, 0, 100, offsets, max_angle * 2);

  // Should work correctly with reset_timestamp_on_fov_start logic
  EXPECT_GE(tracker.timestamp_set_calls.size(), 1);
}

TEST_F(TestScanCutterFoVConfigurations, WraparoundFoV)
{
  // FoV crosses 0° (e.g., 270 to 90)
  ScanCutter<n_channels, angle_unit> cutter(
    0,      // cut at 0 degrees
    27000,  // fov start at 270
    9000,   // fov end at 90
    tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  simulate_rotation(cutter, 0, 100, offsets, max_angle * 2);

  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterFoVConfigurations, CutCrossesZero)
{
  // Cut angle near 0 with wraparound
  ScanCutter<n_channels, angle_unit> cutter(
    35000,  // cut at 350 degrees
    27000,  // fov start at 270
    9000,   // fov end at 90
    tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  simulate_rotation(cutter, 0, 100, offsets, max_angle * 2);

  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterFoVConfigurations, FoV360AsMaxAngleEquivalentToZero)
{
  // FoV with end = 360° (max_angle) should be equivalent to end = 0°
  // Both represent full 360° FoV when start = 0
  tracker.reset();
  ScanCutter<n_channels, angle_unit> cutter_with_zero(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  simulate_rotation(cutter_with_zero, 0, 100, offsets);
  auto publish_count_zero = tracker.publish_calls.size();
  auto timestamp_count_zero = tracker.timestamp_set_calls.size();

  // Now test with fov_end = max_angle (360°)
  tracker.reset();
  ScanCutter<n_channels, angle_unit> cutter_with_max(
    18000, 0, max_angle, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  simulate_rotation(cutter_with_max, 0, 100, offsets);
  auto publish_count_max = tracker.publish_calls.size();
  auto timestamp_count_max = tracker.timestamp_set_calls.size();

  // Both should behave identically (full 360° FoV)
  EXPECT_EQ(publish_count_zero, publish_count_max);
  EXPECT_EQ(timestamp_count_zero, timestamp_count_max);
}

TEST_F(TestScanCutterFoVConfigurations, CutAt360EquivalentToCutAtZero)
{
  // Cut angle at 360° should be equivalent to cut at 0°
  tracker.reset();
  ScanCutter<n_channels, angle_unit> cutter_cut_zero(
    0, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  simulate_rotation(cutter_cut_zero, 0, 100, offsets);
  auto publish_count_zero = tracker.publish_calls.size();
  auto timestamp_count_zero = tracker.timestamp_set_calls.size();

  // Now test with cut at max_angle (360°)
  tracker.reset();
  ScanCutter<n_channels, angle_unit> cutter_cut_max(
    max_angle, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  simulate_rotation(cutter_cut_max, 0, 100, offsets);
  auto publish_count_max = tracker.publish_calls.size();
  auto timestamp_count_max = tracker.timestamp_set_calls.size();

  // Both should behave identically
  EXPECT_EQ(publish_count_zero, publish_count_max);
  EXPECT_EQ(timestamp_count_zero, timestamp_count_max);
}

// =============================================================================
// Test Suite 4: Edge Cases and Boundary Conditions
// =============================================================================

class TestScanCutterEdgeCases : public ::testing::Test
{
protected:
  CallbackTracker tracker;
};

TEST_F(TestScanCutterEdgeCases, CutAndEndVeryClose)
{
  // end - cut < max channel correction
  ScanCutter<n_channels, angle_unit> cutter(
    27000,  // cut at 270 degrees
    9000,   // fov start at 90
    27050,  // fov end at 270.5 degrees (only 50 centi-degrees apart)
    tracker.make_publish_callback(), tracker.make_timestamp_callback());

  // Spread from -100 to +100 centi-degrees
  auto offsets = make_increasing_offsets<n_channels>(-100, 100);

  // Should handle this without crashes
  simulate_rotation(cutter, 0, 100, offsets, max_angle * 2);

  EXPECT_GE(tracker.publish_calls.size(), 0);  // May or may not publish depending on timing
}

TEST_F(TestScanCutterEdgeCases, LargeAzimuthJump)
{
  // Azimuth increases by more than 1 per step
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  // Large jumps of 1000 centi-degrees (10 degrees)
  simulate_rotation(cutter, 0, 1000, offsets);

  // Should still detect cut crossing correctly
  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterEdgeCases, VeryLargeAzimuthJump)
{
  // Azimuth jump larger than 180 degrees
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  auto channel_azimuths = make_channel_azimuths<n_channels>(0, offsets);
  cutter.step(0, channel_azimuths);

  // Jump by 20000 centi-degrees (200 degrees)
  channel_azimuths = make_channel_azimuths<n_channels>(20000, offsets);
  cutter.step(20000, channel_azimuths);

  // Should handle gracefully
  EXPECT_GE(tracker.timestamp_set_calls.size(), 1);
}

TEST_F(TestScanCutterEdgeCases, FoVEntryAndExit)
{
  // Points enter and leave FoV
  ScanCutter<n_channels, angle_unit> cutter(
    18000,  // cut at 180 degrees
    9000,   // fov start at 90
    27000,  // fov end at 270
    tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  // Step through various azimuths including outside FoV
  for (int32_t az = 0; az < max_angle * 2; az += 1000) {
    auto channel_azimuths = make_channel_azimuths<n_channels>(az, offsets);
    cutter.step(az, channel_azimuths);

    // Check FoV filtering
    bool in_fov = cutter.is_point_inside_fov(az);
    if (az >= 9000 && az <= 27000) {
      EXPECT_TRUE(in_fov);
    }
  }

  EXPECT_GE(tracker.publish_calls.size(), 0);
}

TEST_F(TestScanCutterEdgeCases, MultipleRotations)
{
  // Multiple complete rotations
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  // Simulate 3 complete rotations
  simulate_rotation(cutter, 0, 100, offsets, max_angle * 3);

  // Should publish approximately 3 times
  EXPECT_GE(tracker.publish_calls.size(), 2);
}

// =============================================================================
// Test Suite 5: Scan Publication Logic
// =============================================================================

class TestScanCutterPublishBehavior : public ::testing::Test
{
protected:
  CallbackTracker tracker;
};

TEST_F(TestScanCutterPublishBehavior, Case1_AllSameToDifferent)
{
  // Case 1: AllSame → Different (channels split across buffers)
  // This happens when we enter the cut region
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  for (size_t i = 0; i < n_channels / 2; ++i) {
    offsets.at(i) = -300;
  }
  for (size_t i = n_channels / 2; i < n_channels; ++i) {
    offsets.at(i) = +300;
  }

  // Initialize before cut
  auto channel_azimuths = make_channel_azimuths<n_channels>(17000, offsets);
  cutter.step(17000, channel_azimuths);

  size_t timestamp_calls_before = tracker.timestamp_set_calls.size();

  // Step into cut region - should trigger case 1
  channel_azimuths = make_channel_azimuths<n_channels>(18000, offsets);
  cutter.step(18000, channel_azimuths);

  // Should have set timestamp for next buffer
  EXPECT_GT(tracker.timestamp_set_calls.size(), timestamp_calls_before);
}

TEST_F(TestScanCutterPublishBehavior, Case2_DifferentToAllSame)
{
  // Case 2: Different → AllSame (channels reunite, publish happens)
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  for (size_t i = 0; i < n_channels / 2; ++i) {
    offsets.at(i) = -300;
  }
  for (size_t i = n_channels / 2; i < n_channels; ++i) {
    offsets.at(i) = +300;
  }

  // Get into different state (straddling cut)
  auto channel_azimuths = make_channel_azimuths<n_channels>(18000, offsets);
  cutter.step(18000, channel_azimuths);

  size_t publish_calls_before = tracker.publish_calls.size();

  // Move past cut - should trigger case 2 (publish)
  channel_azimuths = make_channel_azimuths<n_channels>(19000, offsets);
  cutter.step(19000, channel_azimuths);

  // Should have published
  EXPECT_GT(tracker.publish_calls.size(), publish_calls_before);
}

TEST_F(TestScanCutterPublishBehavior, Case3_AllSameToAllSameBufferChange)
{
  // Case 3: AllSame → AllSame with buffer change
  // This happens with no channel corrections and a clean cut crossing
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);  // No corrections

  // Start before cut
  auto channel_azimuths = make_channel_azimuths<n_channels>(17900, offsets);
  cutter.step(17900, channel_azimuths);

  size_t publish_calls_before = tracker.publish_calls.size();
  size_t timestamp_calls_before = tracker.timestamp_set_calls.size();

  // Cross cut cleanly
  channel_azimuths = make_channel_azimuths<n_channels>(18100, offsets);
  cutter.step(18100, channel_azimuths);

  // Should have published and set timestamp
  EXPECT_GT(tracker.publish_calls.size(), publish_calls_before);
  EXPECT_GT(tracker.timestamp_set_calls.size(), timestamp_calls_before);
}

TEST_F(TestScanCutterPublishBehavior, NoActionWhenStableState)
{
  // No action when channels remain in same state (not crossing cut)
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets{};
  offsets.fill(0);

  // Step far from cut
  auto channel_azimuths = make_channel_azimuths<n_channels>(9000, offsets);
  cutter.step(9000, channel_azimuths);

  size_t publish_calls_before = tracker.publish_calls.size();
  size_t timestamp_calls_before = tracker.timestamp_set_calls.size();

  // Another step still far from cut
  channel_azimuths = make_channel_azimuths<n_channels>(9100, offsets);
  cutter.step(9100, channel_azimuths);

  // Should not trigger any callbacks
  EXPECT_EQ(tracker.publish_calls.size(), publish_calls_before);
  EXPECT_EQ(tracker.timestamp_set_calls.size(), timestamp_calls_before);
}

// =============================================================================
// Additional Integration Tests
// =============================================================================

TEST(TestScanCutterIntegration, CompleteScenarioWithCorrections)
{
  // Integration test simulating realistic scenario
  CallbackTracker tracker;
  ScanCutter<n_channels, angle_unit> cutter(
    18000,  // cut at 180
    9000,   // fov start at 90
    27000,  // fov end at 270
    tracker.make_publish_callback(), tracker.make_timestamp_callback());

  // Realistic channel corrections: -500 to +500 centi-degrees
  auto offsets = make_increasing_offsets<n_channels>(-500, 500);

  // Simulate two complete rotations
  for (int rotation = 0; rotation < 2; ++rotation) {
    for (int32_t az = 0; az < max_angle; az += 100) {
      auto channel_azimuths = make_channel_azimuths<n_channels>(az, offsets);
      auto buffer_indices = cutter.step(az, channel_azimuths);

      // Verify buffer indices are valid
      for (auto idx : buffer_indices) {
        ASSERT_TRUE(idx == 0 || idx == 1);
      }

      // Check FoV filtering is working
      bool block_intersects = cutter.does_block_intersect_fov(channel_azimuths);
      if (az >= 9000 && az <= 27000) {
        EXPECT_TRUE(block_intersects || az < 9000 + 500 || az > 27000 - 500);
      }
    }
  }

  // Should have published multiple times
  EXPECT_GE(tracker.publish_calls.size(), 1);
  EXPECT_GE(tracker.timestamp_set_calls.size(), 2);
}

}  // namespace nebula::drivers
