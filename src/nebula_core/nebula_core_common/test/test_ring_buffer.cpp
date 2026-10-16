// Copyright 2026 TIER IV, Inc.
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

#include "nebula_core_common/util/ring_buffer.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

TEST(RingBufferTest, EmptyBufferThrowsAndSizeStartsAtZero)
{
  nebula::util::RingBuffer<int> buffer(3);

  EXPECT_EQ(buffer.size(), 0U);
  EXPECT_FALSE(buffer.is_full());
  EXPECT_THROW(static_cast<void>(buffer.get_average()), std::runtime_error);
}

TEST(RingBufferTest, AverageTracksValuesBeforeBufferIsFull)
{
  nebula::util::RingBuffer<int> buffer(3);

  buffer.push_back(2);
  EXPECT_EQ(buffer.size(), 1U);
  EXPECT_EQ(buffer.get_average(), 2);
  EXPECT_FALSE(buffer.is_full());

  buffer.push_back(4);
  EXPECT_EQ(buffer.size(), 2U);
  EXPECT_EQ(buffer.get_average(), 3);
  EXPECT_FALSE(buffer.is_full());

  buffer.push_back(6);
  EXPECT_EQ(buffer.size(), 3U);
  EXPECT_EQ(buffer.get_average(), 4);
  EXPECT_TRUE(buffer.is_full());
}

TEST(RingBufferTest, AverageUsesNewestValuesAfterWrapAround)
{
  nebula::util::RingBuffer<int> buffer(3);

  buffer.push_back(3);
  buffer.push_back(6);
  buffer.push_back(9);
  EXPECT_EQ(buffer.get_average(), 6);

  buffer.push_back(12);
  EXPECT_EQ(buffer.size(), 3U);
  EXPECT_TRUE(buffer.is_full());
  EXPECT_EQ(buffer.get_average(), 9);

  buffer.push_back(15);
  EXPECT_EQ(buffer.get_average(), 12);

  buffer.push_back(18);
  EXPECT_EQ(buffer.get_average(), 15);
}

TEST(RingBufferTest, SupportsNonIntegralAverages)
{
  nebula::util::RingBuffer<double> buffer(2);

  buffer.push_back(0.5);
  buffer.push_back(1.5);
  EXPECT_DOUBLE_EQ(buffer.get_average(), 1.0);

  buffer.push_back(3.5);
  EXPECT_DOUBLE_EQ(buffer.get_average(), 2.5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
