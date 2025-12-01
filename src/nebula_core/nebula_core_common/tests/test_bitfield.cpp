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

#include "nebula_core_common/util/bitfield.hpp"

#include <gtest/gtest.h>

#include <cstdint>

using nebula::util::get_bitfield;

TEST(BitfieldTest, SingleBitExtraction)
{
  uint8_t value = 0b10101010;

  EXPECT_EQ((get_bitfield<uint8_t, 0, 0>(value)), 0);
  EXPECT_EQ((get_bitfield<uint8_t, 1, 1>(value)), 1);
  EXPECT_EQ((get_bitfield<uint8_t, 2, 2>(value)), 0);
  EXPECT_EQ((get_bitfield<uint8_t, 3, 3>(value)), 1);
  EXPECT_EQ((get_bitfield<uint8_t, 4, 4>(value)), 0);
  EXPECT_EQ((get_bitfield<uint8_t, 5, 5>(value)), 1);
  EXPECT_EQ((get_bitfield<uint8_t, 6, 6>(value)), 0);
  EXPECT_EQ((get_bitfield<uint8_t, 7, 7>(value)), 1);
}

TEST(BitfieldTest, MultiBitExtraction)
{
  uint8_t value = 0b11010110;  // = 214

  // Extract bits [0, 3] = 0b0110 = 6
  EXPECT_EQ((get_bitfield<uint8_t, 0, 3>(value)), 6);

  // Extract bits [4, 7] = 0b1101 = 13
  EXPECT_EQ((get_bitfield<uint8_t, 4, 7>(value)), 13);

  // Extract bits [2, 5] = 0b0101 = 5
  EXPECT_EQ((get_bitfield<uint8_t, 2, 5>(value)), 5);
}

TEST(BitfieldTest, FullWidthExtraction)
{
  uint8_t value8 = 0xAB;
  EXPECT_EQ((get_bitfield<uint8_t, 0, 7>(value8)), 0xAB);

  uint16_t value16 = 0xABCD;
  EXPECT_EQ((get_bitfield<uint16_t, 0, 15>(value16)), 0xABCD);

  uint32_t value32 = 0x1234'5678;
  EXPECT_EQ((get_bitfield<uint32_t, 0, 31>(value32)), 0x1234'5678);
}

TEST(BitfieldTest, Uint16Extraction)
{
  uint16_t value = 0xABCD;  // 1010101111001101

  // Extract lower byte
  EXPECT_EQ((get_bitfield<uint8_t, 0, 7>(value)), 0xCD);

  // Extract upper byte
  EXPECT_EQ((get_bitfield<uint8_t, 8, 15>(value)), 0xAB);

  // Extract middle bits [4, 11]
  EXPECT_EQ((get_bitfield<uint8_t, 4, 11>(value)), 0xBC);
}

TEST(BitfieldTest, Uint32Extraction)
{
  uint32_t value = 0x12345678;

  // Extract each byte
  EXPECT_EQ((get_bitfield<uint8_t, 0, 7>(value)), 0x78);
  EXPECT_EQ((get_bitfield<uint8_t, 8, 15>(value)), 0x56);
  EXPECT_EQ((get_bitfield<uint8_t, 16, 23>(value)), 0x34);
  EXPECT_EQ((get_bitfield<uint8_t, 24, 31>(value)), 0x12);

  // Extract 16-bit values
  EXPECT_EQ((get_bitfield<uint16_t, 0, 15>(value)), 0x5678);
  EXPECT_EQ((get_bitfield<uint16_t, 16, 31>(value)), 0x1234);
}

enum class TestEnum : uint8_t { VALUE_0 = 0, VALUE_1 = 1, VALUE_5 = 5, VALUE_15 = 15 };

TEST(BitfieldTest, EnumOutput)
{
  uint8_t value = 0b01010001;  // bits[0,3] = 1, bits[4,7] = 5

  EXPECT_EQ((get_bitfield<TestEnum, 0, 3>(value)), TestEnum::VALUE_1);
  EXPECT_EQ((get_bitfield<TestEnum, 4, 7>(value)), TestEnum::VALUE_5);
}

TEST(BitfieldTest, ZeroValue)
{
  uint32_t value = 0;

  EXPECT_EQ((get_bitfield<uint8_t, 0, 7>(value)), 0);
  EXPECT_EQ((get_bitfield<uint16_t, 8, 23>(value)), 0);
  EXPECT_EQ((get_bitfield<uint32_t, 0, 31>(value)), 0);
}

TEST(BitfieldTest, AllOnesValue)
{
  uint8_t value8 = 0xFF;
  EXPECT_EQ((get_bitfield<uint8_t, 0, 3>(value8)), 0x0F);
  EXPECT_EQ((get_bitfield<uint8_t, 4, 7>(value8)), 0x0F);

  uint16_t value16 = 0xFFFF;
  EXPECT_EQ((get_bitfield<uint8_t, 0, 7>(value16)), 0xFF);
  EXPECT_EQ((get_bitfield<uint16_t, 0, 15>(value16)), 0xFFFF);
}

TEST(BitfieldTest, MacroAccessor)
{
  struct TestStruct
  {
    uint16_t storage;

    BITFIELD_ACCESSOR(uint8_t, low_nibble, 0, 3, storage)
    BITFIELD_ACCESSOR(uint8_t, high_nibble, 4, 7, storage)
    BITFIELD_ACCESSOR(uint8_t, upper_byte, 8, 15, storage)
  };

  TestStruct s{0xABCD};

  EXPECT_EQ(s.low_nibble(), 0x0D);
  EXPECT_EQ(s.high_nibble(), 0x0C);
  EXPECT_EQ(s.upper_byte(), 0xAB);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
