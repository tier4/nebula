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

#include "nebula_core_common/util/bitfield.hpp"
#include "nebula_core_common/util/errno.hpp"
#include "nebula_core_common/util/expected.hpp"
#include "nebula_core_common/util/rate_limiter.hpp"
#include "nebula_core_common/util/stopwatch.hpp"
#include "nebula_core_common/util/string_conversions.hpp"

#include <gtest/gtest.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

enum class BitfieldMode : std::uint8_t { Zero = 0, One = 1, Two = 2, Three = 3 };

struct BitfieldPacket
{
  std::uint16_t storage{};
  BITFIELD_ACCESSOR(BitfieldMode, mode, 0, 1, storage)
  BITFIELD_ACCESSOR(std::uint8_t, counter, 8, 11, storage)
};

TEST(UtilityHelpersTest, BitfieldExtractionSupportsRawAndAccessorUsage)
{
  constexpr std::uint16_t storage = 0x0A03U;
  const auto low_nibble = nebula::util::get_bitfield<std::uint8_t, 0, 3>(storage);
  const auto high_nibble = nebula::util::get_bitfield<std::uint8_t, 8, 11>(storage);
  const auto mode = nebula::util::get_bitfield<BitfieldMode, 0, 1>(storage);

  EXPECT_EQ(low_nibble, 3U);
  EXPECT_EQ(high_nibble, 10U);
  EXPECT_EQ(mode, BitfieldMode::Three);

  const BitfieldPacket packet{storage};
  EXPECT_EQ(packet.mode(), BitfieldMode::Three);
  EXPECT_EQ(packet.counter(), 10U);
}

TEST(UtilityHelpersTest, ExpectedProvidesValueAndErrorAccessors)
{
  const nebula::util::expected<int, std::string> value{42};
  EXPECT_TRUE(value.has_value());
  EXPECT_EQ(value.value(), 42);
  EXPECT_EQ(value.value_or(7), 42);
  EXPECT_EQ(value.value_or_throw("unused"), 42);
  EXPECT_EQ(value.error_or("fallback"), "fallback");
  EXPECT_THROW(static_cast<void>(value.error()), nebula::util::bad_expected_access);

  const nebula::util::expected<int, std::string> error{std::string("bad")};
  EXPECT_FALSE(error.has_value());
  EXPECT_EQ(error.error(), "bad");
  EXPECT_EQ(error.value_or(7), 7);
  EXPECT_EQ(error.error_or("fallback"), "bad");

  try {
    static_cast<void>(error.value());
    FAIL() << "Expected bad_expected_access";
  } catch (const nebula::util::bad_expected_access & exception) {
    EXPECT_STREQ(exception.what(), "value() called but containing error");
  }

  try {
    static_cast<void>(value.error());
    FAIL() << "Expected bad_expected_access";
  } catch (const nebula::util::bad_expected_access & exception) {
    EXPECT_STREQ(exception.what(), "error() called but containing value");
  }

  try {
    static_cast<void>(error.value_or_throw("custom failure"));
    FAIL() << "Expected runtime_error";
  } catch (const std::runtime_error & exception) {
    EXPECT_STREQ(exception.what(), "custom failure");
  }
}

TEST(UtilityHelpersTest, ExpectedCanThrowStoredErrorType)
{
  const nebula::util::expected<int, std::runtime_error> error{std::runtime_error("stored error")};

  try {
    static_cast<void>(error.value_or_throw());
    FAIL() << "Expected runtime_error";
  } catch (const std::runtime_error & exception) {
    EXPECT_STREQ(exception.what(), "stored error");
  }
}

TEST(UtilityHelpersTest, RateLimiterSuppressesCallsInsideWindow)
{
  nebula::util::RateLimiter limiter(std::chrono::milliseconds(100));
  int invocations = 0;

  auto action = [&]() { ++invocations; };

  limiter.with_rate_limit(100000000ULL, action);
  limiter.with_rate_limit(150000000ULL, action);
  limiter.with_rate_limit(199999999ULL, action);
  limiter.with_rate_limit(200000000ULL, action);

  EXPECT_EQ(invocations, 2);
}

TEST(UtilityHelpersTest, StopwatchResetRestartsElapsedTime)
{
  nebula::util::Stopwatch stopwatch;
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  const auto elapsed_before_reset = stopwatch.elapsed_ns();

  EXPECT_GT(elapsed_before_reset, 0U);

  stopwatch.reset();
  const auto elapsed_after_reset = stopwatch.elapsed_ns();

  EXPECT_LT(elapsed_after_reset, elapsed_before_reset);
}

TEST(UtilityHelpersTest, ErrnoToStringReturnsDistinctMessages)
{
  const auto success_message = nebula::util::errno_to_string(0);
  const auto invalid_argument_message = nebula::util::errno_to_string(EINVAL);

  EXPECT_FALSE(success_message.empty());
  EXPECT_FALSE(invalid_argument_message.empty());
  EXPECT_NE(success_message, invalid_argument_message);
}

TEST(UtilityHelpersTest, StringConversionsHandleStreamableTypesAndJson)
{
  EXPECT_EQ(nebula::util::to_string(123), "123");

  constexpr char truncated_string[] = {'a', 'b', '\0', 'c', '\0'};
  EXPECT_EQ(nebula::util::to_string(truncated_string), "ab");

  const nlohmann::ordered_json ordered_string = "hello";
  EXPECT_EQ(nebula::util::to_string(ordered_string), "hello");

  const nlohmann::ordered_json ordered_object = {{"answer", 42}};
  EXPECT_EQ(nebula::util::to_string(ordered_object), "{\"answer\":42}");

  const nlohmann::json array = {1, 2, 3};
  EXPECT_EQ(nebula::util::to_string(array), "[1,2,3]");
  EXPECT_EQ(nebula::util::format_timestamp(12U, 34U), "12.000000034");
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
