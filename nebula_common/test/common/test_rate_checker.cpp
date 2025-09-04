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

#include "nebula_common/util/rate_checker.hpp"

#include <gtest/gtest.h>

static void feed_at_rate(nebula::util::RateChecker & checker, double rate_hz, uint32_t num_updates)
{
  double interval = 1.0 / rate_hz;
  for (uint32_t i = 0; i < num_updates; ++i) {
    checker.update(i * interval);
  }
}

TEST(RateCheckerTest, BasicFunctionality)
{
  nebula::util::RateChecker checker(8.0, 12.0, 5);

  EXPECT_FALSE(checker.is_full());
  EXPECT_FALSE(checker.is_valid());
  EXPECT_THROW(static_cast<void>(checker.get_average()), std::runtime_error);

  feed_at_rate(checker, 10.0, 6);

  EXPECT_TRUE(checker.is_full());
  EXPECT_NO_THROW(static_cast<void>(checker.get_average()));
  EXPECT_NEAR(checker.get_average(), 10.0, 0.1);
  EXPECT_TRUE(checker.is_valid());
}

TEST(RateCheckerTest, InvalidRate)
{
  nebula::util::RateChecker checker(8.0, 12.0, 3);

  feed_at_rate(checker, 5.0, 4);

  EXPECT_NEAR(checker.get_average(), 5.0, 0.1);
  EXPECT_FALSE(checker.is_valid());

  checker = nebula::util::RateChecker(8.0, 12.0, 3);
  feed_at_rate(checker, 15.0, 4);

  EXPECT_NEAR(checker.get_average(), 15.0, 0.1);
  EXPECT_FALSE(checker.is_valid());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
