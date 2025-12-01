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

#include "nebula_core_decoders/point_filters/blockage_mask.hpp"

#include <nebula_core_common/nebula_common.hpp>

#include <gtest/gtest.h>

#include <cmath>

using nebula::drivers::AngleRange;
using nebula::drivers::deg2rad;
using nebula::drivers::MilliDegrees;
using nebula::drivers::point_filters::BlockageMask;
using nebula::drivers::point_filters::BlockageMaskPlugin;
using nebula::drivers::point_filters::BlockageState;

class BlockageMaskTest : public ::testing::Test
{
protected:
  // Create a mask covering -180 to 180 degrees with 1 degree bins and 16 channels
  static constexpr int32_t azi_start_mdeg = -180000;
  static constexpr int32_t azi_end_mdeg = 180000;
  static constexpr uint32_t bin_size_mdeg = 1000;  // 1 degree
  static constexpr uint16_t n_channels = 16;

  AngleRange<int32_t, MilliDegrees> azi_range{azi_start_mdeg, azi_end_mdeg};
};

TEST_F(BlockageMaskTest, Construction)
{
  BlockageMask mask(azi_range, bin_size_mdeg, n_channels);

  EXPECT_EQ(mask.get_width(), 360);
  EXPECT_EQ(mask.get_height(), 16);
  EXPECT_EQ(mask.get_mask().size(), mask.get_width() * mask.get_height());
}

TEST_F(BlockageMaskTest, InitialStateIsZero)
{
  BlockageMask mask(azi_range, bin_size_mdeg, n_channels);

  const auto & data = mask.get_mask();
  for (size_t i = 0; i < data.size(); ++i) {
    EXPECT_EQ(data[i], 0) << "Non-zero at index " << i;
  }
}

TEST_F(BlockageMaskTest, UpdateBlockageIncrements)
{
  BlockageMask mask(azi_range, bin_size_mdeg, n_channels);

  // Update at azimuth 0 (which maps to bin 180), channel 5
  double azi_rad = deg2rad(0.0);
  mask.update(azi_rad, 5, BlockageState::BLOCKAGE);

  // Should increment the corresponding bin
  const auto & data = mask.get_mask();

  size_t expected_index = 5 * mask.get_width() + 180;  // channel 5, bin 180
  EXPECT_EQ(data[expected_index], 1);
}

TEST_F(BlockageMaskTest, UpdateNoBlockageDoesNotIncrement)
{
  BlockageMask mask(azi_range, bin_size_mdeg, n_channels);

  double azi_rad = deg2rad(0.0);
  mask.update(azi_rad, 0, BlockageState::NO_BLOCKAGE);
  mask.update(azi_rad, 0, BlockageState::UNSURE);

  // Should not increment
  const auto & data = mask.get_mask();
  size_t expected_index = 0 * mask.get_width() + 180;
  EXPECT_EQ(data[expected_index], 0);
}

TEST_F(BlockageMaskTest, MultipleBlockagesAccumulate)
{
  BlockageMask mask(azi_range, bin_size_mdeg, n_channels);

  double azi_rad = deg2rad(45.0);
  for (int i = 0; i < 5; ++i) {
    mask.update(azi_rad, 5, BlockageState::BLOCKAGE);
  }

  const auto & data = mask.get_mask();
  // 45 degrees from -180 = bin 225
  size_t expected_index = 5 * mask.get_width() + 225;
  EXPECT_EQ(data[expected_index], 5);
}

TEST_F(BlockageMaskTest, SaturatesAtMaxUint8)
{
  BlockageMask mask(azi_range, bin_size_mdeg, n_channels);

  double azi_rad = deg2rad(0.0);

  // Try to increment more than 255 times
  for (int i = 0; i < 300; ++i) {
    mask.update(azi_rad, 0, BlockageState::BLOCKAGE);
  }

  const auto & data = mask.get_mask();
  size_t expected_index = 0 * mask.get_width() + 180;
  EXPECT_EQ(data[expected_index], 255);  // Should saturate at UINT8_MAX
}

TEST_F(BlockageMaskTest, DifferentChannelsAreIndependent)
{
  BlockageMask mask(azi_range, bin_size_mdeg, n_channels);

  double azi_rad = deg2rad(0.0);

  mask.update(azi_rad, 0, BlockageState::BLOCKAGE);
  mask.update(azi_rad, 0, BlockageState::BLOCKAGE);
  mask.update(azi_rad, 1, BlockageState::BLOCKAGE);

  const auto & data = mask.get_mask();
  size_t index_ch0 = 0 * mask.get_width() + 180;
  size_t index_ch1 = 1 * mask.get_width() + 180;

  EXPECT_EQ(data[index_ch0], 2);
  EXPECT_EQ(data[index_ch1], 1);
}

TEST_F(BlockageMaskTest, OutOfBoundsChannelIgnored)
{
  BlockageMask mask(azi_range, bin_size_mdeg, n_channels);

  double azi_rad = deg2rad(0.0);

  // Channel 16 is out of bounds (0-15 valid)
  mask.update(azi_rad, 16, BlockageState::BLOCKAGE);
  mask.update(azi_rad, 100, BlockageState::BLOCKAGE);

  // Mask should be unchanged
  const auto & data = mask.get_mask();
  for (size_t i = 0; i < data.size(); ++i) {
    EXPECT_EQ(data[i], 0);
  }
}

TEST_F(BlockageMaskTest, OutOfBoundsAzimuthIgnored)
{
  // Create a smaller range to test out-of-bounds
  AngleRange<int32_t, MilliDegrees> small_range{-45000, 45000};  // -45 to 45 degrees
  BlockageMask mask(small_range, bin_size_mdeg, n_channels);

  // Azimuth outside the range
  double azi_outside = deg2rad(90.0);
  mask.update(azi_outside, 0, BlockageState::BLOCKAGE);

  // Mask should be unchanged
  const auto & data = mask.get_mask();
  for (size_t i = 0; i < data.size(); ++i) {
    EXPECT_EQ(data[i], 0);
  }
}

TEST_F(BlockageMaskTest, NegativeAzimuthHandled)
{
  BlockageMask mask(azi_range, bin_size_mdeg, n_channels);

  // -90 degrees should be valid
  double azi_rad = deg2rad(-90.1);
  mask.update(azi_rad, 0, BlockageState::BLOCKAGE);

  const auto & data = mask.get_mask();
  // -90.1 degrees from -180 = bin 89
  size_t expected_index = 0 * mask.get_width() + 89;
  EXPECT_EQ(data[expected_index], 1);
}

TEST_F(BlockageMaskTest, BoundaryAzimuths)
{
  BlockageMask mask(azi_range, bin_size_mdeg, n_channels);

  // Test at the boundaries
  double azi_start = deg2rad(-179.95);
  double azi_end = deg2rad(179.95);  // Just before 180

  mask.update(azi_start, 0, BlockageState::BLOCKAGE);
  mask.update(azi_end, 1, BlockageState::BLOCKAGE);

  const auto & data = mask.get_mask();

  // First bin
  size_t index_start = 0 * mask.get_width() + 0;
  EXPECT_EQ(data[index_start], 1);

  // Last bin (359)
  size_t index_end = 1 * mask.get_width() + 359;
  EXPECT_EQ(data[index_end], 1);
}

// ============================================================================
// BlockageMaskPlugin tests
// ============================================================================

TEST(BlockageMaskPluginTest, Construction)
{
  BlockageMaskPlugin plugin(1000);  // 1 degree bin width
  EXPECT_EQ(plugin.get_bin_width_mdeg(), 1000);
}

TEST(BlockageMaskPluginTest, CallbackAndReset)
{
  BlockageMaskPlugin plugin(1000);

  AngleRange<int32_t, MilliDegrees> azi_range{-180000, 180000};
  BlockageMask mask(azi_range, 1000, 16);

  // Add some blockages
  mask.update(deg2rad(0.0), 0, BlockageState::BLOCKAGE);
  mask.update(deg2rad(0.0), 0, BlockageState::BLOCKAGE);

  bool callback_called = false;
  size_t received_sum = 0;
  double received_timestamp = 0.0;

  plugin.set_callback([&](const BlockageMask & m, double ts) {
    callback_called = true;
    received_timestamp = ts;
    // Sum all blockage counts
    for (auto val : m.get_mask()) {
      received_sum += val;
    }
  });

  plugin.callback_and_reset(mask, 123.456);

  EXPECT_TRUE(callback_called);
  EXPECT_EQ(received_sum, 2);
  EXPECT_DOUBLE_EQ(received_timestamp, 123.456);

  // Mask should be reset after callback
  for (auto val : mask.get_mask()) {
    EXPECT_EQ(val, 0);
  }
}

TEST(BlockageMaskPluginTest, NoCallbackSet)
{
  BlockageMaskPlugin plugin(1000);

  AngleRange<int32_t, MilliDegrees> azi_range{-180000, 180000};
  BlockageMask mask(azi_range, 1000, 16);

  mask.update(deg2rad(0.0), 0, BlockageState::BLOCKAGE);

  // Should not crash even without callback
  EXPECT_NO_THROW(plugin.callback_and_reset(mask, 0.0));

  // Mask should still be reset
  for (auto val : mask.get_mask()) {
    EXPECT_EQ(val, 0);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
