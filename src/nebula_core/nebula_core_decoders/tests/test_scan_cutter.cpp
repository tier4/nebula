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

#include <array>
#include <cstddef>
#include <cstdint>
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
  std::array<int32_t, N> result{};
  for (size_t i = 0; i < N; ++i) {
    result.at(i) = from + static_cast<int32_t>(i * (to - from) / (N - 1));
  }

  assert(result.at(0) == from);
  assert(result.at(N - 1) == to);
  return result;
}

/// Helper to create synthetic channel azimuths with given offsets
template <size_t N>
CorrectedAzimuths<N> make_channel_azimuths(
  int32_t base_azimuth, const std::array<int32_t, N> & offsets)
{
  CorrectedAzimuths<N> result{};

  result.min_correction_index = std::min_element(offsets.begin(), offsets.end()) - offsets.begin();
  result.max_correction_index = std::max_element(offsets.begin(), offsets.end()) - offsets.begin();

  for (size_t i = 0; i < N; ++i) {
    int32_t azimuth = base_azimuth + offsets.at(i);
    azimuth = normalize_angle(azimuth, max_angle);
    result.azimuths.at(i) = azimuth;
  }

  return result;
}

/// Helper to simulate a full rotation
template <size_t NChannels, uint32_t AngleUnit>
void simulate_rotation(
  ScanCutter<NChannels, AngleUnit> & cutter, int32_t start_azimuth, int32_t step_size,
  const std::array<int32_t, NChannels> & offsets, int32_t total_rotation = 360 * AngleUnit)
{
  int32_t end_azimuth = start_azimuth + total_rotation;

  for (int32_t az = start_azimuth; az < end_azimuth; az += step_size) {
    auto channel_azimuths = make_channel_azimuths<NChannels>(az, offsets);
    cutter.step(channel_azimuths);
  }
}

}  // namespace

TEST(TestScanCutterBasic, Construction)
{
  auto make_cutter =
    [](
      int32_t cut_angle, int32_t fov_start, int32_t fov_end,
      const ScanCutter<n_channels, angle_unit>::publish_callback_t & publish_callback,
      const ScanCutter<n_channels, angle_unit>::set_timestamp_callback_t & set_timestamp_callback) {
      ScanCutter<n_channels, angle_unit> cutter(
        cut_angle, fov_start, fov_end, publish_callback, set_timestamp_callback);
      return cutter;
    };

  auto dummy_cb = [](uint8_t) {};

  // Most basic case
  EXPECT_NO_THROW(make_cutter(0, 0, 36000, dummy_cb, dummy_cb));
  // Angles are normalized on construction, so 0 == 360
  EXPECT_NO_THROW(make_cutter(0, 0, 0, dummy_cb, dummy_cb));

  // For 360deg FoV, cut angle can be anywhere
  EXPECT_NO_THROW(make_cutter(18000, 0, 36000, dummy_cb, dummy_cb));
  EXPECT_NO_THROW(make_cutter(18000, 0, 0, dummy_cb, dummy_cb));
  EXPECT_NO_THROW(make_cutter(36000, 0, 36000, dummy_cb, dummy_cb));

  // Cut angle must be within FoV
  EXPECT_THROW(make_cutter(0, 9000, 27000, dummy_cb, dummy_cb), std::invalid_argument);
  // ... even for wrapping FoVs
  EXPECT_THROW(make_cutter(18000, 27000, 9000, dummy_cb, dummy_cb), std::invalid_argument);
  // Cut angle cannot coincide with FoV start
  EXPECT_THROW(make_cutter(9000, 9000, 27000, dummy_cb, dummy_cb), std::invalid_argument);
  // Cut angle is allowed to be at FoV end
  EXPECT_NO_THROW(make_cutter(27000, 9000, 27000, dummy_cb, dummy_cb));
  // Anywhere else in the FoV is allowed
  EXPECT_NO_THROW(make_cutter(18000, 9000, 27000, dummy_cb, dummy_cb));
  // ... even for wrapping FoVs
  EXPECT_NO_THROW(make_cutter(0, 27000, 9000, dummy_cb, dummy_cb));

  // Callbacks must be valid
  EXPECT_THROW(make_cutter(18000, 0, 36000, nullptr, dummy_cb), std::invalid_argument);
  EXPECT_THROW(make_cutter(18000, 0, 36000, dummy_cb, nullptr), std::invalid_argument);
  EXPECT_THROW(make_cutter(18000, 0, 36000, nullptr, nullptr), std::invalid_argument);
}

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

  cutter.step(channel_azimuths);

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

  simulate_rotation(cutter, 0, 100, offsets, max_angle * 2);

  // Verify timestamps were set
  EXPECT_EQ(tracker.timestamp_set_calls.size(), 2);
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

  simulate_rotation(cutter, 0, 100, offsets);

  // With no corrections, all channels should always be in same buffer
  // Expect at least one publish for a full rotation
  EXPECT_EQ(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterChannelCorrections, SymmetricSpread)
{
  // Channels spread equally in positive and negative directions
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  auto offsets = make_increasing_offsets<n_channels>(-500, 500);

  simulate_rotation(cutter, 0, 100, offsets);

  EXPECT_EQ(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterChannelCorrections, AsymmetricSpread)
{
  // abs(min_offset) != abs(max_offset)
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  auto offsets = make_increasing_offsets<n_channels>(-1000, 500);

  simulate_rotation(cutter, 0, 100, offsets);

  EXPECT_EQ(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterChannelCorrections, PositiveOnlySpread)
{
  // All corrections are positive
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  auto offsets = make_increasing_offsets<n_channels>(0, 1000);

  simulate_rotation(cutter, 0, 100, offsets);

  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterChannelCorrections, NegativeOnlySpread)
{
  // All corrections are negative
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  auto offsets = make_increasing_offsets<n_channels>(-1000, 0);

  simulate_rotation(cutter, 0, 100, offsets);

  EXPECT_GE(tracker.publish_calls.size(), 1);
}

TEST_F(TestScanCutterChannelCorrections, BlockStraddlesCut)
{
  // Test when a block has some channels before cut and some after
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets = make_increasing_offsets<n_channels>(-200, 200);

  // Block intersects cut angle
  auto channel_azimuths = make_channel_azimuths<n_channels>(18000, offsets);
  auto scan_state = cutter.step(channel_azimuths);

  // Verify channels are split across buffers
  bool has_buffer_0 = false;
  bool has_buffer_1 = false;
  for (auto idx : scan_state.channel_buffer_indices) {
    if (idx == 0) has_buffer_0 = true;
    if (idx == 1) has_buffer_1 = true;
  }
  EXPECT_TRUE(has_buffer_0);
  EXPECT_TRUE(has_buffer_1);
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
  cutter.step(channel_azimuths);

  // Jump by 20000 centi-degrees (200 degrees)
  channel_azimuths = make_channel_azimuths<n_channels>(20000, offsets);
  cutter.step(channel_azimuths);

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

  std::array<int32_t, n_channels> offsets{0};

  // Step through various azimuths including outside FoV
  for (int32_t az = 0; az < max_angle * 2; az += 1000) {
    auto channel_azimuths = make_channel_azimuths<n_channels>(az, offsets);
    auto scan_state = cutter.step(channel_azimuths);

    // Check FoV filtering
    bool in_fov = scan_state.does_block_intersect_fov();
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

  // Simulate 3 complete rotations (passes cut angle 3 times)
  simulate_rotation(cutter, 0, 100, offsets, max_angle * 3);

  EXPECT_EQ(tracker.publish_calls.size(), 3);
}

// =============================================================================
// Test Suite 5: Scan Publication Logic
// =============================================================================

class TestScanCutterPublishBehavior : public ::testing::Test
{
protected:
  CallbackTracker tracker;
};

TEST_F(TestScanCutterPublishBehavior, InFrameToCrossingCut)
{
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets = make_increasing_offsets<n_channels>(-300, 300);

  // Initialize before cut
  auto channel_azimuths = make_channel_azimuths<n_channels>(17000, offsets);
  cutter.step(channel_azimuths);

  size_t timestamp_calls_before = tracker.timestamp_set_calls.size();

  // Step into cut region - should trigger case 1
  channel_azimuths = make_channel_azimuths<n_channels>(18000, offsets);
  cutter.step(channel_azimuths);

  // Should have set timestamp for next buffer
  EXPECT_GT(tracker.timestamp_set_calls.size(), timestamp_calls_before);
}

TEST_F(TestScanCutterPublishBehavior, CrossingCutToInFrame)
{
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, tracker.make_publish_callback(), tracker.make_timestamp_callback());

  std::array<int32_t, n_channels> offsets = make_increasing_offsets<n_channels>(-300, 300);

  // Get into different state (straddling cut)
  auto channel_azimuths = make_channel_azimuths<n_channels>(18000, offsets);
  cutter.step(channel_azimuths);

  size_t publish_calls_before = tracker.publish_calls.size();

  // Move past cut - should trigger case 2 (publish)
  channel_azimuths = make_channel_azimuths<n_channels>(19000, offsets);
  cutter.step(channel_azimuths);

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
  cutter.step(channel_azimuths);

  size_t publish_calls_before = tracker.publish_calls.size();
  size_t timestamp_calls_before = tracker.timestamp_set_calls.size();

  // Cross cut cleanly
  channel_azimuths = make_channel_azimuths<n_channels>(18100, offsets);
  cutter.step(channel_azimuths);

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
  cutter.step(channel_azimuths);

  size_t publish_calls_before = tracker.publish_calls.size();
  size_t timestamp_calls_before = tracker.timestamp_set_calls.size();

  // Another step still far from cut
  channel_azimuths = make_channel_azimuths<n_channels>(9100, offsets);
  cutter.step(channel_azimuths);

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
      auto scan_state = cutter.step(channel_azimuths);

      // Verify buffer indices are valid
      for (auto idx : scan_state.channel_buffer_indices) {
        ASSERT_TRUE(idx == 0 || idx == 1);
      }

      // Check FoV filtering is working
      bool block_intersects = scan_state.does_block_intersect_fov();
      if (az >= 9000 && az <= 27000) {
        EXPECT_TRUE(block_intersects || az < 9000 + 500 || az > 27000 - 500);
      }
    }
  }

  // Should have published multiple times
  EXPECT_GE(tracker.publish_calls.size(), 1);
  EXPECT_GE(tracker.timestamp_set_calls.size(), 2);
}

// =============================================================================
// Resilience Tests for Packet Loss and Azimuth Jumps
// =============================================================================

class TestScanCutterResilience : public ::testing::Test
{
protected:
  /// Dummy point cloud buffer to track scan properties
  struct DummyPointCloudBuffer
  {
    struct PacketStats
    {
      int32_t start_azimuth;
      int32_t end_azimuth;
      size_t packet_count;
      int32_t azimuth_span_accumulated;  // Track total azimuth increase
    };

    std::optional<PacketStats> stats_;
    bool timestamp_has_been_set = false;

    void reset()
    {
      stats_ = std::nullopt;
      timestamp_has_been_set = false;
    }

    void set_timestamp() { timestamp_has_been_set = true; }

    void add_packet(int32_t azimuth)
    {
      if (!stats_) {
        stats_ = PacketStats{
          normalize_angle(azimuth, max_angle),
          normalize_angle(azimuth, max_angle),
          0,
          0,
        };
      }

      stats_->end_azimuth = normalize_angle(azimuth, max_angle);
      stats_->packet_count++;

      int32_t diff = normalize_angle(azimuth - stats_->end_azimuth, max_angle);
      stats_->azimuth_span_accumulated += diff;
    }

    [[nodiscard]] bool overlaps_itself() const
    {
      if (!stats_) {
        throw std::runtime_error("No stats available");
      }
      return stats_->azimuth_span_accumulated > max_angle;
    }

    [[nodiscard]] int32_t get_angular_span() const
    {
      if (!stats_) {
        throw std::runtime_error("No stats available");
      }

      return stats_->azimuth_span_accumulated;
    }

    [[nodiscard]] bool is_active() const { return timestamp_has_been_set; }

    [[nodiscard]] bool has_points() const { return stats_ != std::nullopt; }
  };

  // Dummy decoder that tracks pointcloud properties
  struct DummyDecoder
  {
    std::array<DummyPointCloudBuffer, 2> buffers{};
    uint8_t current_buffer = 0;

    void on_publish(uint8_t buffer_index)
    {
      ASSERT_TRUE(buffers.at(buffer_index).timestamp_has_been_set)
        << "Tried to publish inactive buffer";
      buffers.at(buffer_index).reset();
    }

    void on_timestamp_set(uint8_t buffer_index)
    {
      ASSERT_FALSE(buffers.at(buffer_index).timestamp_has_been_set)
        << "Tried to set timestamp on already initialized buffer";
      buffers.at(buffer_index).set_timestamp();
    }

    void process_packet(
      int32_t block_azimuth, const typename ScanCutter<n_channels, angle_unit>::State & scan_state)
    {
      // Add packet to appropriate buffer(s)
      for (size_t ch = 0; ch < n_channels; ++ch) {
        auto buf_idx = scan_state.channel_buffer_indices.at(ch);
        buffers.at(buf_idx).add_packet(block_azimuth);
      }
    }
  };

  auto make_publish_callback()
  {
    return [this](uint8_t buffer_index) {
      decoder.on_publish(buffer_index);
      tracker.make_publish_callback()(buffer_index);
    };
  }

  auto make_timestamp_callback()
  {
    return [this](uint8_t buffer_index) {
      decoder.on_timestamp_set(buffer_index);
      tracker.make_timestamp_callback()(buffer_index);
    };
  }

  CallbackTracker tracker;
  DummyDecoder decoder;
};

// ============================================================================
// Packet Loss Tests
// ============================================================================

// Validates behavior when packets are lost around the cut angle
TEST_F(TestScanCutterResilience, PacketLossAcrossCutAngle)
{
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, make_publish_callback(), make_timestamp_callback());

  // Corrections of ± 5°
  auto offsets = make_increasing_offsets<n_channels>(-500, 500);

  const int32_t packet_loss_start = 17000;
  const int32_t packet_loss_end = 19000;

  for (int32_t az = 0; az < max_angle * 2; az += 100) {
    // Skip packets in loss range
    if (az >= packet_loss_start && az <= packet_loss_end) {
      continue;
    }

    auto channel_azimuths = make_channel_azimuths(az, offsets);
    auto scan_state = cutter.step(channel_azimuths);
    decoder.process_packet(az, scan_state);
  }

  // Should have published at least once despite packet loss
  ASSERT_GT(tracker.publish_calls.size(), 0) << "No scans were published";

  // Check that no buffer overlaps itself (no scan > 360°)
  for (const auto & buf : decoder.buffers) {
    if (buf.is_active()) {
      EXPECT_FALSE(buf.overlaps_itself()) << "Buffer overlaps itself";
      // Angular span should not exceed 360° + 30° (cut region tolerance)
      EXPECT_LE(buf.get_angular_span(), max_angle + 500)
        << "Scan width exceeds FOV + 5°: " << buf.get_angular_span() / 100.0 << "°";
    }
  }
}

// Validates behavior when packets are lost around FOV start (for non-360 FOV)
TEST_F(TestScanCutterResilience, PacketLossAcrossFovStart)
{
  ScanCutter<n_channels, angle_unit> cutter(
    27000, 9000, 27000, make_publish_callback(), make_timestamp_callback());

  // Corrections of ± 5°
  auto offsets = make_increasing_offsets<n_channels>(-500, 500);

  const int32_t packet_loss_start = 8000;
  const int32_t packet_loss_end = 10000;

  for (int32_t az = 0; az < max_angle * 2; az += 100) {
    int32_t normalized_az = normalize_angle(az, max_angle);

    // Skip packets in loss range
    if (normalized_az >= packet_loss_start && normalized_az <= packet_loss_end) {
      continue;
    }

    auto channel_azimuths = make_channel_azimuths(az, offsets);
    auto scan_state = cutter.step(channel_azimuths);
    decoder.process_packet(az, scan_state);
  }

  // Should still produce scans
  ASSERT_GT(tracker.publish_calls.size(), 0) << "No scans were published";

  // Timestamp should be set even with packet loss at FOV start
  EXPECT_GT(tracker.timestamp_set_calls.size(), 0) << "Timestamp was never set";

  // Check scan properties
  for (const auto & buf : decoder.buffers) {
    if (buf.is_active()) {
      EXPECT_FALSE(buf.overlaps_itself()) << "Buffer overlaps itself";
      EXPECT_LE(buf.get_angular_span(), 18000 + 500) << "Scan width exceeds FOV + 5°";
    }
  }
}

// Validates behavior when packets are lost around FOV end (for non-360 FOV)
TEST_F(TestScanCutterResilience, PacketLossAcrossFovEnd)
{
  ScanCutter<n_channels, angle_unit> cutter(
    27000, 9000, 27000, make_publish_callback(), make_timestamp_callback());

  // Corrections of ± 5°
  auto offsets = make_increasing_offsets<n_channels>(-500, 500);

  const int32_t packet_loss_start = 26000;
  const int32_t packet_loss_end = 28000;

  for (int32_t az = 0; az < max_angle * 2; az += 100) {
    int32_t normalized_az = normalize_angle(az, max_angle);

    // Skip packets in loss range
    if (normalized_az >= packet_loss_start && normalized_az <= packet_loss_end) {
      continue;
    }

    auto channel_azimuths = make_channel_azimuths(az, offsets);
    auto scan_state = cutter.step(channel_azimuths);
    decoder.process_packet(az, scan_state);
  }

  // Should still produce scans
  ASSERT_GT(tracker.publish_calls.size(), 0) << "No scans were published";

  // Check that scans were emitted properly
  for (const auto & buf : decoder.buffers) {
    if (buf.is_active()) {
      EXPECT_FALSE(buf.overlaps_itself()) << "Buffer overlaps itself";
      EXPECT_LE(buf.get_angular_span(), 18000 + 3000) << "Scan width exceeds FOV + 30°";
    }
  }
}

// Validates behavior with extended packet loss across cut
TEST_F(TestScanCutterResilience, MultiplePacketLossAcrossCut)
{
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, make_publish_callback(), make_timestamp_callback());

  // Corrections of ± 5°
  auto offsets = make_increasing_offsets<n_channels>(-500, 500);

  const int32_t packet_loss_start = 15000;
  const int32_t packet_loss_end = 21000;

  for (int32_t az = 0; az < max_angle * 2; az += 100) {
    // Skip packets in extended loss range
    if (az >= packet_loss_start && az <= packet_loss_end) {
      continue;
    }

    auto channel_azimuths = make_channel_azimuths(az, offsets);
    auto scan_state = cutter.step(channel_azimuths);
    decoder.process_packet(az, scan_state);
  }

  // System should still produce scans with extended packet loss
  ASSERT_GT(tracker.publish_calls.size(), 0) << "No scans were published with extended packet loss";

  // Even with extended loss, scans should not overlap
  for (const auto & buf : decoder.buffers) {
    if (buf.is_active()) {
      EXPECT_FALSE(buf.overlaps_itself()) << "Buffer overlaps itself with extended packet loss";
    }
  }
}

// ============================================================================
// Time Loop Tests (Azimuth Jump Tests)
// ============================================================================

// Validates behavior when azimuth jumps backwards at 360° boundary (loop restart)
TEST_F(TestScanCutterResilience, AzimuthJumpBackwardsAtBoundary)
{
  ScanCutter<n_channels, angle_unit> cutter(
    0, 0, 0, make_publish_callback(), make_timestamp_callback());

  // Corrections of ± 5°
  auto offsets = make_increasing_offsets<n_channels>(-500, 500);

  for (int32_t az = 0; az < 35000; az += 100) {
    auto channel_azimuths = make_channel_azimuths(az, offsets);
    auto scan_state = cutter.step(channel_azimuths);
    decoder.process_packet(az, scan_state);
  }

  size_t publish_before_jump = tracker.publish_calls.size();
  size_t timestamp_before_jump = tracker.timestamp_set_calls.size();

  // Simulate azimuth jump back to 10° (loop restart)
  for (int32_t az = 1000; az < 20000; az += 100) {
    auto channel_azimuths = make_channel_azimuths(az, offsets);
    auto scan_state = cutter.step(channel_azimuths);
    decoder.process_packet(az, scan_state);
  }

  // Should have published and reset timestamp after jump
  EXPECT_GT(tracker.publish_calls.size(), publish_before_jump) << "No publish after azimuth jump";
  EXPECT_GT(tracker.timestamp_set_calls.size(), timestamp_before_jump)
    << "No timestamp reset after azimuth jump";

  // Check no buffer overlaps
  for (const auto & buf : decoder.buffers) {
    if (buf.is_active()) {
      EXPECT_FALSE(buf.overlaps_itself()) << "Buffer overlaps after azimuth jump";
      EXPECT_LE(buf.get_angular_span(), max_angle + 500);
    }
  }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

// Validates behavior when first packets are lost (initialization with packet loss)
TEST_F(TestScanCutterResilience, PacketLossAtInitialization)
{
  ScanCutter<n_channels, angle_unit> cutter(
    0, 0, 0, make_publish_callback(), make_timestamp_callback());

  // Corrections of ± 5°
  auto offsets = make_increasing_offsets<n_channels>(-500, 500);

  // Skip first 50 packets (start at 50°) and do one complete rotation
  for (int32_t az = 5000; az <= max_angle + 5000; az += 100) {
    auto channel_azimuths = make_channel_azimuths(az, offsets);
    auto scan_state = cutter.step(channel_azimuths);
    decoder.process_packet(az, scan_state);
  }

  // Should initialize properly and produce scans despite missing initial packets
  ASSERT_EQ(tracker.publish_calls.size(), 1)
    << "No scans produced after initialization packet loss";
  EXPECT_EQ(tracker.timestamp_set_calls.size(), 1)
    << "Timestamp not set after initialization packet loss";

  // Verify scan integrity
  for (const auto & buf : decoder.buffers) {
    if (buf.is_active()) {
      EXPECT_FALSE(buf.overlaps_itself()) << "Buffer overlaps after init packet loss";
      EXPECT_LE(buf.get_angular_span(), max_angle + 500);
    }
  }
}

// Validates behavior with multiple azimuth jumps (multiple loop iterations)
TEST_F(TestScanCutterResilience, MultipleAzimuthJumps)
{
  ScanCutter<n_channels, angle_unit> cutter(
    18000, 0, 0, make_publish_callback(), make_timestamp_callback());

  // Corrections of ± 5°
  auto offsets = make_increasing_offsets<n_channels>(-500, 500);

  // Simulate three azimuth jumps
  int jump_count = 0;
  for (int iteration = 0; iteration < 3; ++iteration) {
    // Rotate to 300°
    for (int32_t az = 0; az < 30000; az += 100) {
      auto channel_azimuths = make_channel_azimuths(az, offsets);
      auto scan_state = cutter.step(channel_azimuths);
      decoder.process_packet(az, scan_state);
    }

    // Jump back to 50°
    jump_count++;
  }

  // Should handle multiple jumps correctly (should have at least as many publishes as jumps)
  EXPECT_GE(tracker.publish_calls.size(), jump_count) << "Not enough publishes for multiple jumps";

  // Verify no overlaps despite multiple jumps
  for (const auto & buf : decoder.buffers) {
    if (buf.is_active()) {
      EXPECT_FALSE(buf.overlaps_itself()) << "Buffer overlaps with multiple jumps";
      EXPECT_LE(buf.get_angular_span(), max_angle + 500);
    }
  }
}

// Validates behavior with continuous packet loss (stress test)
TEST_F(TestScanCutterResilience, ContinuousPacketLoss)
{
  ScanCutter<n_channels, angle_unit> cutter(
    0, 0, 0, make_publish_callback(), make_timestamp_callback());

  // Corrections of ± 5°
  auto offsets = make_increasing_offsets<n_channels>(-500, 500);

  // Skip every other packet (50% packet loss), do one complete rotation
  for (int32_t az = 0; az <= max_angle + 1000; az += 100) {
    if ((az % 200) == 100) {
      continue;  // Skip this packet
    }

    auto channel_azimuths = make_channel_azimuths(az, offsets);
    auto scan_state = cutter.step(channel_azimuths);
    decoder.process_packet(az, scan_state);
  }

  // Should still produce scans with 50% packet loss
  ASSERT_GT(tracker.publish_calls.size(), 0) << "No scans produced with continuous packet loss";

  // Verify scan integrity despite heavy packet loss
  for (const auto & buf : decoder.buffers) {
    if (buf.is_active()) {
      EXPECT_FALSE(buf.overlaps_itself()) << "Buffer overlaps with continuous packet loss";
      EXPECT_LE(buf.get_angular_span(), max_angle + 500);
    }
  }
}

}  // namespace nebula::drivers
