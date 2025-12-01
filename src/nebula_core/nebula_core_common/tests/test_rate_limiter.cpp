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

#include "nebula_core_common/util/rate_limiter.hpp"

#include <gtest/gtest.h>

#include <chrono>

using nebula::util::RateLimiter;

// Note: The RateLimiter blocks calls if (now_ns - last_passed_time_ns_) < rate_limit_ns_
// Since last_passed_time_ns_ starts at 0, the first call at time 0 is blocked for non-zero rate
// limits. This means calls only pass when now_ns >= rate_limit_ns_ (after the rate limit period has
// elapsed).

TEST(RateLimiterTest, FirstCallAtRateLimitPasses)
{
  RateLimiter limiter(std::chrono::milliseconds(100));

  int call_count = 0;
  // First call at exactly the rate limit time passes
  limiter.with_rate_limit(100'000'000, [&call_count]() { call_count++; });  // 100ms in ns

  EXPECT_EQ(call_count, 1);
}

TEST(RateLimiterTest, CallBeforeRateLimitBlocked)
{
  RateLimiter limiter(std::chrono::milliseconds(100));

  int call_count = 0;
  auto action = [&call_count]() { call_count++; };

  // Calls before the rate limit time are blocked
  limiter.with_rate_limit(0, action);
  EXPECT_EQ(call_count, 0);

  limiter.with_rate_limit(50'000'000, action);  // 50ms
  EXPECT_EQ(call_count, 0);

  limiter.with_rate_limit(99'999'999, action);  // Just under 100ms
  EXPECT_EQ(call_count, 0);
}

TEST(RateLimiterTest, CallAtRateLimitPasses)
{
  RateLimiter limiter(std::chrono::milliseconds(100));

  int call_count = 0;
  auto action = [&call_count]() { call_count++; };

  // Call at exactly 100ms passes
  limiter.with_rate_limit(100'000'000, action);
  EXPECT_EQ(call_count, 1);
}

TEST(RateLimiterTest, MultipleCallsOverTime)
{
  RateLimiter limiter(std::chrono::milliseconds(50));

  int call_count = 0;
  auto action = [&call_count]() { call_count++; };

  // t=50ms: passes (first at rate limit)
  limiter.with_rate_limit(50'000'000, action);
  EXPECT_EQ(call_count, 1);

  // t=75ms: blocked (only 25ms since last pass)
  limiter.with_rate_limit(75'000'000, action);
  EXPECT_EQ(call_count, 1);

  // t=100ms: passes (50ms since last pass)
  limiter.with_rate_limit(100'000'000, action);
  EXPECT_EQ(call_count, 2);

  // t=125ms: blocked (25ms since last pass)
  limiter.with_rate_limit(125'000'000, action);
  EXPECT_EQ(call_count, 2);

  // t=150ms: passes (50ms since last pass)
  limiter.with_rate_limit(150'000'000, action);
  EXPECT_EQ(call_count, 3);

  // t=200ms: passes (50ms since last pass)
  limiter.with_rate_limit(200'000'000, action);
  EXPECT_EQ(call_count, 4);
}

TEST(RateLimiterTest, ZeroRateLimit)
{
  RateLimiter limiter(std::chrono::milliseconds(0));

  int call_count = 0;
  auto action = [&call_count]() { call_count++; };

  // With zero rate limit, all calls pass
  limiter.with_rate_limit(0, action);
  limiter.with_rate_limit(0, action);
  limiter.with_rate_limit(0, action);

  EXPECT_EQ(call_count, 3);
}

TEST(RateLimiterTest, LargeTimeGap)
{
  RateLimiter limiter(std::chrono::milliseconds(100));

  int call_count = 0;
  auto action = [&call_count]() { call_count++; };

  // First call at rate limit
  limiter.with_rate_limit(100'000'000, action);
  EXPECT_EQ(call_count, 1);

  // Call after a very long time
  limiter.with_rate_limit(1'000'000'000'000ULL, action);  // 1000 seconds in ns
  EXPECT_EQ(call_count, 2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
