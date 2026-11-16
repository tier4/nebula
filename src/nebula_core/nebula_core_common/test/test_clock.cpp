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

#include "nebula_core_common/util/clock.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

namespace nebula::util
{

using std::chrono_literals::operator""ms;

TEST(SteadyClock, AdvancesWithTime)
{
  SteadyClock clock;

  const auto t0 = clock.now();
  std::this_thread::sleep_for(2ms);
  const auto t1 = clock.now();

  EXPECT_GE(t1 - t0, 2ms);
}

TEST(SteadyClock, CanBeUsedThroughTheInterface)
{
  const SteadyClock steady_clock;
  const Clock & clock = steady_clock;

  const auto t0 = clock.now();
  EXPECT_GE(clock.now(), t0);
}

}  // namespace nebula::util
