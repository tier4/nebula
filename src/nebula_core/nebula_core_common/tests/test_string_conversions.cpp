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

#include "nebula_core_common/util/string_conversions.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

using nebula::util::format_timestamp;
using nebula::util::to_string;

TEST(StringConversionsTest, ToStringInteger)
{
  EXPECT_EQ(to_string(42), "42");
  EXPECT_EQ(to_string(-123), "-123");
  EXPECT_EQ(to_string(0), "0");
}

TEST(StringConversionsTest, ToStringDouble)
{
  // Note: floating point to_string may have precision differences
  std::string result = to_string(3.14159);
  EXPECT_TRUE(result.find("3.14") != std::string::npos);
}

TEST(StringConversionsTest, ToStringCharArray)
{
  char buffer[10] = "hello";
  EXPECT_EQ(to_string<10>(buffer), "hello");

  // Test with null terminator in middle
  char buffer2[10] = {'a', 'b', 'c', '\0', 'd', 'e'};
  EXPECT_EQ(to_string<10>(buffer2), "abc");
}

TEST(StringConversionsTest, ToStringCharArrayMaxLen)
{
  // Buffer completely filled - no null terminator
  char buffer[5] = {'a', 'b', 'c', 'd', 'e'};
  EXPECT_EQ(to_string<5>(buffer), "abcde");
}

TEST(StringConversionsTest, FormatTimestampBasic)
{
  // 1234567890.123456789
  std::string result = format_timestamp(1234567890, 123456789);
  EXPECT_EQ(result, "1234567890.123456789");
}

TEST(StringConversionsTest, FormatTimestampZeroNanoseconds)
{
  std::string result = format_timestamp(1000, 0);
  EXPECT_EQ(result, "1000.000000000");
}

TEST(StringConversionsTest, FormatTimestampZeroSeconds)
{
  std::string result = format_timestamp(0, 500000000);
  EXPECT_EQ(result, "0.500000000");
}

TEST(StringConversionsTest, FormatTimestampSmallNanoseconds)
{
  // Nanoseconds with leading zeros
  std::string result = format_timestamp(100, 1);
  EXPECT_EQ(result, "100.000000001");

  result = format_timestamp(100, 100);
  EXPECT_EQ(result, "100.000000100");
}

TEST(StringConversionsTest, FormatTimestampMaxNanoseconds)
{
  std::string result = format_timestamp(0, 999999999);
  EXPECT_EQ(result, "0.999999999");
}

TEST(StringConversionsTest, ToStringJson)
{
  nlohmann::json j_string = "hello";
  EXPECT_EQ(to_string(j_string), "hello");

  nlohmann::json j_number = 42;
  EXPECT_EQ(to_string(j_number), "42");

  nlohmann::json j_object = {{"key", "value"}};
  std::string result = to_string(j_object);
  EXPECT_EQ(result, "{\"key\":\"value\"}");
}

TEST(StringConversionsTest, ToStringOrderedJson)
{
  nlohmann::ordered_json j_object = {{"key1", "value1"}, {"key2", "value2"}};
  std::string result = to_string(j_object);
  EXPECT_EQ(result, "{\"key1\":\"value1\",\"key2\":\"value2\"}");
}

// Custom type with ostream operator
struct CustomType
{
  int value;
  friend std::ostream & operator<<(std::ostream & os, const CustomType & ct)
  {
    os << "CustomFormat(" << ct.value << ")";
    return os;
  }
};

TEST(StringConversionsTest, ToStringCustomStreamableType)
{
  CustomType ct{42};
  EXPECT_EQ(to_string(ct), "CustomFormat(42)");
}

TEST(StringConversionsTest, IsStreamableTraitWorks)
{
  EXPECT_TRUE(nebula::util::IsStreamable<int>::value);
  EXPECT_TRUE(nebula::util::IsStreamable<double>::value);
  EXPECT_TRUE(nebula::util::IsStreamable<std::string>::value);
  EXPECT_TRUE(nebula::util::IsStreamable<CustomType>::value);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
