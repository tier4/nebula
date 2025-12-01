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

#include "nebula_hesai_decoders/decoders/hesai_packet.hpp"

#include <gtest/gtest.h>

#include <cstdint>

using nebula::drivers::hesai_packet::Block;
using nebula::drivers::hesai_packet::DateTime;
using nebula::drivers::hesai_packet::FineAzimuthBlock;
using nebula::drivers::hesai_packet::get_n_returns;
using nebula::drivers::hesai_packet::PacketBase;
namespace return_mode = nebula::drivers::hesai_packet::return_mode;
using nebula::drivers::hesai_packet::SecondsSinceEpoch;
using nebula::drivers::hesai_packet::SOBBlock;
using nebula::drivers::hesai_packet::Unit3B;

// ============================================================================
// get_n_returns tests
// ============================================================================

TEST(HesaiPacketTest, GetNReturnsSingleModes)
{
  EXPECT_EQ(get_n_returns(return_mode::SINGLE_FIRST), 1);
  EXPECT_EQ(get_n_returns(return_mode::SINGLE_SECOND), 1);
  EXPECT_EQ(get_n_returns(return_mode::SINGLE_STRONGEST), 1);
  EXPECT_EQ(get_n_returns(return_mode::SINGLE_LAST), 1);
}

TEST(HesaiPacketTest, GetNReturnsDualModes)
{
  EXPECT_EQ(get_n_returns(return_mode::DUAL_LAST_STRONGEST), 2);
  EXPECT_EQ(get_n_returns(return_mode::DUAL_FIRST_SECOND), 2);
  EXPECT_EQ(get_n_returns(return_mode::DUAL_FIRST_LAST), 2);
  EXPECT_EQ(get_n_returns(return_mode::DUAL_FIRST_STRONGEST), 2);
  EXPECT_EQ(get_n_returns(return_mode::DUAL_STRONGEST_SECONDSTRONGEST), 2);
}

TEST(HesaiPacketTest, GetNReturnsTripleMode)
{
  EXPECT_EQ(get_n_returns(return_mode::TRIPLE_FIRST_LAST_STRONGEST), 3);
}

TEST(HesaiPacketTest, GetNReturnsUnknownThrows)
{
  EXPECT_THROW(get_n_returns(0x00), std::runtime_error);
  EXPECT_THROW(get_n_returns(0xFF), std::runtime_error);
}

// ============================================================================
// DateTime tests (YearOffset = 1900)
// ============================================================================

TEST(DateTimeTest, GetSecondsBasic)
{
  DateTime<1900> dt;
  dt.year = 124;  // 2024 (124 + 1900)
  dt.month = 1;   // January
  dt.day = 1;
  dt.hour = 0;
  dt.minute = 0;
  dt.second = 0;

  // 2024-01-01 00:00:00 UTC. Unix epoch starts at 1970
  // 1704067200 = 2024-01-01 00:00:00 UTC
  uint64_t expected = 1704067200;
  EXPECT_EQ(dt.get_seconds(), expected);
}

TEST(DateTimeTest, GetSecondsWithTime)
{
  DateTime<1900> dt;
  dt.year = 124;  // 2024
  dt.month = 6;   // June
  dt.day = 15;
  dt.hour = 12;
  dt.minute = 30;
  dt.second = 45;

  // Verify it's a reasonable timestamp (mid-2024)
  uint64_t result = dt.get_seconds();
  EXPECT_GT(result, (2024 - 1970) * 365 * 24 * 60 * 60);  // After 2024-01-01
  EXPECT_LT(result, (2025 - 1970) * 365 * 24 * 60 * 60);  // Before 2025-01-01
}

TEST(DateTimeTest, GetSecondsEpoch)
{
  DateTime<1900> dt;
  dt.year = 70;  // 1970 (70 + 1900)
  dt.month = 1;
  dt.day = 1;
  dt.hour = 0;
  dt.minute = 0;
  dt.second = 0;

  // Unix epoch
  EXPECT_EQ(dt.get_seconds(), 0);
}

// ============================================================================
// DateTime tests (YearOffset = 2000)
// ============================================================================

TEST(DateTimeTest, GetSecondsYear2000Offset)
{
  DateTime<2000> dt;
  dt.year = 24;  // 2024 (24 + 2000)
  dt.month = 1;
  dt.day = 1;
  dt.hour = 0;
  dt.minute = 0;
  dt.second = 0;

  // 2024-01-01 00:00:00 UTC
  uint64_t expected = 1704067200;
  EXPECT_EQ(dt.get_seconds(), expected);
}

// ============================================================================
// SecondsSinceEpoch tests
// ============================================================================

TEST(SecondsSinceEpochTest, GetSecondsZero)
{
  SecondsSinceEpoch sse;
  sse.zero = 0;
  sse.seconds[0] = 0;
  sse.seconds[1] = 0;
  sse.seconds[2] = 0;
  sse.seconds[3] = 0;
  sse.seconds[4] = 0;

  EXPECT_EQ(sse.get_seconds(), 0);
}

TEST(SecondsSinceEpochTest, GetSecondsSmallValue)
{
  SecondsSinceEpoch sse;
  sse.zero = 0;
  // 256 (0x100) in big-endian 5-byte format
  sse.seconds[0] = 0x00;  // MSB
  sse.seconds[1] = 0x00;
  sse.seconds[2] = 0x00;
  sse.seconds[3] = 0x01;
  sse.seconds[4] = 0x00;  // LSB

  EXPECT_EQ(sse.get_seconds(), 256);
}

TEST(SecondsSinceEpochTest, GetSecondsTypicalValue)
{
  SecondsSinceEpoch sse;
  sse.zero = 0;
  // 1704067200 (2024-01-01 00:00:00) = 0x65920080
  // In big-endian 5-byte: 00 65 92 00 80
  sse.seconds[0] = 0x00;  // MSB
  sse.seconds[1] = 0x65;
  sse.seconds[2] = 0x92;
  sse.seconds[3] = 0x00;
  sse.seconds[4] = 0x80;  // LSB

  EXPECT_EQ(sse.get_seconds(), 1704067200);
}

TEST(SecondsSinceEpochTest, GetSecondsMaxValue)
{
  SecondsSinceEpoch sse;
  sse.zero = 0;
  // Maximum 5-byte value: 0xFFFFFFFFFF = 1099511627775 in big-endian
  sse.seconds[0] = 0xFF;
  sse.seconds[1] = 0xFF;
  sse.seconds[2] = 0xFF;
  sse.seconds[3] = 0xFF;
  sse.seconds[4] = 0xFF;

  EXPECT_EQ(sse.get_seconds(), 1099511627775ULL);
}

// ============================================================================
// Block tests
// ============================================================================

TEST(BlockTest, GetAzimuth)
{
  Block<Unit3B, 32> block;
  block.azimuth = 18000;  // 180.00 degrees

  EXPECT_EQ(block.get_azimuth(), 18000);
}

TEST(FineAzimuthBlockTest, GetAzimuth)
{
  FineAzimuthBlock<Unit3B, 32> block;
  block.azimuth = 180;      // Coarse azimuth
  block.fine_azimuth = 50;  // Fine azimuth

  // Combined: (180 << 8) + 50 = 46130
  EXPECT_EQ(block.get_azimuth(), 46130);
}

TEST(SOBBlockTest, GetAzimuth)
{
  SOBBlock<Unit3B, 32> block;
  block.sob = 0xFFEE;
  block.azimuth = 27000;  // 270.00 degrees

  EXPECT_EQ(block.get_azimuth(), 27000);
}

// ============================================================================
// PacketBase tests
// ============================================================================

TEST(PacketBaseTest, StaticConstants)
{
  using TestPacket = PacketBase<12, 32, 2, 100>;

  EXPECT_EQ(TestPacket::n_blocks, 12);
  EXPECT_EQ(TestPacket::n_channels, 32);
  EXPECT_EQ(TestPacket::max_returns, 2);
  EXPECT_EQ(TestPacket::degree_subdivisions, 100);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
