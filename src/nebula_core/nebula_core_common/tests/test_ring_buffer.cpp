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

#include "nebula_core_common/util/ring_buffer.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using nebula::util::RingBuffer;

TEST(RingBufferTest, InitialState)
{
  RingBuffer<double> buffer(5);

  EXPECT_EQ(buffer.size(), 0);
  EXPECT_FALSE(buffer.is_full());
}

TEST(RingBufferTest, EmptyBufferThrowsOnGetAverage)
{
  RingBuffer<double> buffer(5);

  EXPECT_THROW(static_cast<void>(buffer.get_average()), std::runtime_error);
}

TEST(RingBufferTest, PushIncrementsSize)
{
  RingBuffer<double> buffer(5);

  buffer.push_back(1.0);
  EXPECT_EQ(buffer.size(), 1);

  buffer.push_back(2.0);
  EXPECT_EQ(buffer.size(), 2);

  buffer.push_back(3.0);
  EXPECT_EQ(buffer.size(), 3);
}

TEST(RingBufferTest, IsFullWhenCapacityReached)
{
  RingBuffer<double> buffer(3);

  buffer.push_back(1.0);
  EXPECT_FALSE(buffer.is_full());

  buffer.push_back(2.0);
  EXPECT_FALSE(buffer.is_full());

  buffer.push_back(3.0);
  EXPECT_TRUE(buffer.is_full());
  EXPECT_EQ(buffer.size(), 3);
}

TEST(RingBufferTest, AverageCalculation)
{
  RingBuffer<double> buffer(5);

  buffer.push_back(1.0);
  EXPECT_DOUBLE_EQ(buffer.get_average(), 1.0);

  buffer.push_back(2.0);
  EXPECT_DOUBLE_EQ(buffer.get_average(), 1.5);  // (1 + 2) / 2

  buffer.push_back(3.0);
  EXPECT_DOUBLE_EQ(buffer.get_average(), 2.0);  // (1 + 2 + 3) / 3
}

TEST(RingBufferTest, OverwritesOldValuesWhenFull)
{
  RingBuffer<double> buffer(3);

  // Fill the buffer
  buffer.push_back(1.0);
  buffer.push_back(2.0);
  buffer.push_back(3.0);
  EXPECT_TRUE(buffer.is_full());
  EXPECT_DOUBLE_EQ(buffer.get_average(), 2.0);  // (1 + 2 + 3) / 3

  // Push one more - should overwrite the first value (1.0)
  buffer.push_back(4.0);
  EXPECT_TRUE(buffer.is_full());
  EXPECT_EQ(buffer.size(), 3);
  EXPECT_DOUBLE_EQ(buffer.get_average(), 3.0);  // (2 + 3 + 4) / 3

  // Push another - should overwrite the second value (2.0)
  buffer.push_back(5.0);
  EXPECT_DOUBLE_EQ(buffer.get_average(), 4.0);  // (3 + 4 + 5) / 3
}

TEST(RingBufferTest, WorksWithIntegers)
{
  RingBuffer<int> buffer(4);

  buffer.push_back(10);
  buffer.push_back(20);
  buffer.push_back(30);
  buffer.push_back(40);

  EXPECT_EQ(buffer.get_average(), 25);  // (10 + 20 + 30 + 40) / 4

  buffer.push_back(50);
  EXPECT_EQ(buffer.get_average(), 35);  // (20 + 30 + 40 + 50) / 4
}

TEST(RingBufferTest, SingleElementBuffer)
{
  RingBuffer<double> buffer(1);

  buffer.push_back(5.0);
  EXPECT_TRUE(buffer.is_full());
  EXPECT_DOUBLE_EQ(buffer.get_average(), 5.0);

  buffer.push_back(10.0);
  EXPECT_DOUBLE_EQ(buffer.get_average(), 10.0);
}

TEST(RingBufferTest, LargeCapacity)
{
  RingBuffer<double> buffer(1000);

  double sum = 0.0;
  for (int i = 1; i <= 1000; ++i) {
    buffer.push_back(static_cast<double>(i));
    sum += i;
  }

  EXPECT_TRUE(buffer.is_full());
  EXPECT_DOUBLE_EQ(buffer.get_average(), sum / 1000.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
