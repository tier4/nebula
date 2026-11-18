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

#include "nebula_hesai_decoders/decoders/packet_loss_detector.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>

namespace nebula::drivers
{

namespace
{

template <typename SequenceT>
struct FakePacket
{
  struct
  {
    SequenceT udp_sequence;
  } tail;
};

/// Feed packets with the given sequence numbers to a detector, and return the reported losses.
template <typename SequenceT>
uint64_t count_lost(std::initializer_list<SequenceT> sequences)
{
  uint64_t n_lost = 0;
  PacketLossDetector<FakePacket<SequenceT>> detector;
  detector.set_lost_callback([&](uint64_t n) { n_lost += n; });

  for (const auto sequence : sequences) {
    FakePacket<SequenceT> packet{};
    packet.tail.udp_sequence = sequence;
    detector.update(packet);
  }
  return n_lost;
}

}  // namespace

TEST(PacketLossDetector, ReportsNothingForConsecutivePackets)
{
  EXPECT_EQ(count_lost<uint32_t>({1, 2, 3, 4}), 0U);
}

TEST(PacketLossDetector, ReportsTheSizeOfEachGap)
{
  EXPECT_EQ(count_lost<uint32_t>({1, 2, 5, 6, 8}), 3U);
}

TEST(PacketLossDetector, ReportsGapsAcrossTheWrapAround)
{
  EXPECT_EQ(count_lost<uint32_t>({0xFFFFFFFE, 0xFFFFFFFF, 0, 1}), 0U);
  EXPECT_EQ(count_lost<uint32_t>({0xFFFFFFFE, 1}), 2U);
  EXPECT_EQ(count_lost<uint16_t>({0xFFFF, 0, 3}), 2U);
}

TEST(PacketLossDetector, DoesNotReportDuplicatesAsLost)
{
  EXPECT_EQ(count_lost<uint32_t>({1, 2, 2, 3}), 0U);
  EXPECT_EQ(count_lost<uint16_t>({1, 2, 2, 3}), 0U);
}

TEST(PacketLossDetector, DoesNotReportPacketsThatGoBackwardsAsLost)
{
  EXPECT_EQ(count_lost<uint32_t>({100, 101, 5}), 0U);
  EXPECT_EQ(count_lost<uint32_t>({0, 1, 0xFFFFFFF0}), 0U);
}

TEST(PacketLossDetector, ContinuesFromTheNewSequenceAfterGoingBackwards)
{
  // E.g. a sensor restart: losses after it are still reported
  EXPECT_EQ(count_lost<uint32_t>({1000, 1001, 0, 1, 3}), 1U);
}

}  // namespace nebula::drivers
