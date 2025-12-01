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

#include "nebula_hesai_decoders/decoders/packet_loss_detector.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

using nebula::drivers::PacketLossDetector;

// Mock packet structure for testing
struct MockPacket
{
  struct
  {
    uint32_t udp_sequence;
  } tail;
};

class PacketLossDetectorTest : public ::testing::Test
{
protected:
  PacketLossDetector<MockPacket> detector;

  MockPacket make_packet(uint32_t seq)
  {
    MockPacket p;
    p.tail.udp_sequence = seq;
    return p;
  }
};

TEST_F(PacketLossDetectorTest, FirstPacketNoCallback)
{
  uint64_t lost_count = 0;
  detector.set_lost_callback([&](uint64_t n) { lost_count = n; });

  auto p = make_packet(100);
  detector.update(p);

  // First packet should not trigger callback
  EXPECT_EQ(lost_count, 0);
}

TEST_F(PacketLossDetectorTest, SequentialPacketsNoLoss)
{
  uint64_t callback_calls = 0;
  detector.set_lost_callback([&](uint64_t /*n*/) { callback_calls++; });

  auto p1 = make_packet(100);
  auto p2 = make_packet(101);
  auto p3 = make_packet(102);
  auto p4 = make_packet(103);

  detector.update(p1);
  detector.update(p2);
  detector.update(p3);
  detector.update(p4);

  // No loss - callback should not be called
  EXPECT_EQ(callback_calls, 0);
}

TEST_F(PacketLossDetectorTest, SinglePacketLoss)
{
  uint64_t lost_count = 0;
  detector.set_lost_callback([&](uint64_t n) { lost_count = n; });

  auto p1 = make_packet(100);
  auto p2 = make_packet(102);  // Skipped 101

  detector.update(p1);
  detector.update(p2);

  EXPECT_EQ(lost_count, 1);
}

TEST_F(PacketLossDetectorTest, MultiplePacketLoss)
{
  uint64_t lost_count = 0;
  detector.set_lost_callback([&](uint64_t n) { lost_count = n; });

  auto p1 = make_packet(100);
  auto p2 = make_packet(110);  // Skipped 101-109 (9 packets)

  detector.update(p1);
  detector.update(p2);

  EXPECT_EQ(lost_count, 9);
}

TEST_F(PacketLossDetectorTest, AccumulatedLoss)
{
  std::vector<uint64_t> losses;
  detector.set_lost_callback([&](uint64_t n) { losses.push_back(n); });

  auto p1 = make_packet(100);
  auto p2 = make_packet(102);  // Skip 1
  auto p3 = make_packet(103);  // Sequential
  auto p4 = make_packet(108);  // Skip 4

  detector.update(p1);
  detector.update(p2);
  detector.update(p3);
  detector.update(p4);

  EXPECT_EQ(losses.size(), 2);
  EXPECT_EQ(losses[0], 1);
  EXPECT_EQ(losses[1], 4);
}

TEST_F(PacketLossDetectorTest, NoCallbackSet)
{
  // Should not crash when no callback is set
  auto p1 = make_packet(100);
  auto p2 = make_packet(200);  // Large gap

  EXPECT_NO_THROW(detector.update(p1));
  EXPECT_NO_THROW(detector.update(p2));
}

TEST_F(PacketLossDetectorTest, LargeGap)
{
  uint64_t lost_count = 0;
  detector.set_lost_callback([&](uint64_t n) { lost_count = n; });

  auto p1 = make_packet(0);
  auto p2 = make_packet(10000);  // Large gap

  detector.update(p1);
  detector.update(p2);

  EXPECT_EQ(lost_count, 9999);
}

TEST_F(PacketLossDetectorTest, StartingFromNonZero)
{
  uint64_t lost_count = 0;
  detector.set_lost_callback([&](uint64_t n) { lost_count = n; });

  // First packet can be any sequence number
  auto p1 = make_packet(50000);
  auto p2 = make_packet(50001);
  auto p3 = make_packet(50003);  // Skip 1

  detector.update(p1);
  detector.update(p2);
  detector.update(p3);

  EXPECT_EQ(lost_count, 1);
}

TEST_F(PacketLossDetectorTest, SequenceNumberWraparoundNoLoss)
{
  // Assuming uint32_t wraparound at 0xFFFFFFFF -> 0
  std::vector<uint64_t> losses;
  detector.set_lost_callback([&](uint64_t n) { losses.push_back(n); });

  auto p1 = make_packet(0xFFFFFFFEu);
  auto p2 = make_packet(0xFFFFFFFFu);
  auto p3 = make_packet(0u);
  auto p4 = make_packet(1u);

  detector.update(p1);
  detector.update(p2);
  detector.update(p3);
  detector.update(p4);

  // No losses expected
  EXPECT_TRUE(losses.empty());
}

TEST_F(PacketLossDetectorTest, SequenceNumberWraparoundWithLoss)
{
  std::vector<uint64_t> losses;
  detector.set_lost_callback([&](uint64_t n) { losses.push_back(n); });

  auto p1 = make_packet(0xFFFFFFFEu);
  auto p2 = make_packet(1u);  // skip 0xFFFFFFFF and 0

  detector.update(p1);
  detector.update(p2);

  // One packet (0) is lost during the wrap
  // The implementation reports total lost packets in a single callback
  ASSERT_EQ(losses.size(), 1u);
  EXPECT_EQ(losses[0], 2u);
}

// ============================================================================
// Edge cases
// ============================================================================

TEST_F(PacketLossDetectorTest, CallbackCanBeChanged)
{
  uint64_t count1 = 0;
  uint64_t count2 = 0;

  detector.set_lost_callback([&](uint64_t n) { count1 = n; });

  auto p1 = make_packet(0);
  auto p2 = make_packet(5);  // Loss of 4

  detector.update(p1);
  detector.update(p2);

  EXPECT_EQ(count1, 4);
  EXPECT_EQ(count2, 0);

  // Change callback
  detector.set_lost_callback([&](uint64_t n) { count2 = n; });

  auto p3 = make_packet(10);  // Loss of 4

  detector.update(p3);

  EXPECT_EQ(count1, 4);  // Unchanged
  EXPECT_EQ(count2, 4);  // New callback used
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
