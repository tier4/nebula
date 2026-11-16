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

#include "nebula_core_common/diagnostics/hysteresis_state_machine.hpp"
#include "nebula_core_common/diagnostics/level.hpp"
#include "nebula_core_common/diagnostics/liveness_monitor.hpp"
#include "nebula_core_common/diagnostics/rate_bound_status.hpp"
#include "nebula_core_common/util/clock.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>

namespace nebula::diagnostics
{

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

namespace
{

class ManualClock final : public util::Clock
{
public:
  [[nodiscard]] std::chrono::nanoseconds now() const override { return now_; }
  void advance(std::chrono::nanoseconds duration) { now_ += duration; }

private:
  std::chrono::nanoseconds now_{1s};
};

}  // namespace

TEST(Level, HasTheValuesOfDiagnosticStatus)
{
  EXPECT_EQ(static_cast<int>(Level::OK), 0);
  EXPECT_EQ(static_cast<int>(Level::WARN), 1);
  EXPECT_EQ(static_cast<int>(Level::ERROR), 2);
  EXPECT_EQ(static_cast<int>(Level::STALE), 3);
  EXPECT_EQ(to_string(Level::WARN), "WARN");
}

TEST(LivenessMonitor, DeadOnlyAfterTimeoutWithoutTick)
{
  auto clock = std::make_shared<ManualClock>();
  LivenessMonitor monitor(clock, 100ms);

  clock->advance(99ms);
  EXPECT_TRUE(monitor.report().is_alive);
  EXPECT_EQ(monitor.report().level(), Level::OK);

  clock->advance(1ms);
  auto report = monitor.report();
  EXPECT_FALSE(report.is_alive);
  EXPECT_EQ(report.level(), Level::ERROR);
  EXPECT_EQ(report.lateness, 100ms);
  EXPECT_EQ(report.last_tick, 1s);

  monitor.tick();
  report = monitor.report();
  EXPECT_TRUE(report.is_alive);
  EXPECT_EQ(report.lateness, 0ms);
  EXPECT_EQ(report.last_tick, 1100ms);
}

TEST(HysteresisStateMachine, ChangesStateAfterEnoughObservations)
{
  HysteresisStateMachine state_machine(2, false, true);
  EXPECT_EQ(state_machine.get_current_state_level(), Level::STALE);

  state_machine.update_state(Level::OK);
  EXPECT_EQ(state_machine.get_current_state_level(), Level::OK);

  state_machine.update_state(Level::ERROR);
  EXPECT_EQ(state_machine.get_current_state_level(), Level::OK);
  EXPECT_EQ(state_machine.get_candidate_level(), Level::ERROR);

  state_machine.update_state(Level::ERROR);
  EXPECT_EQ(state_machine.get_current_state_level(), Level::ERROR);
}

TEST(RateBoundStatus, RejectsWarnRangeInsideOkRange)
{
  auto clock = std::make_shared<ManualClock>();
  EXPECT_THROW(
    RateBoundStatus(clock, RateBoundStatusParam{9, 11}, RateBoundStatusParam{9.5, 12}),
    std::runtime_error);
}

TEST(RateBoundStatus, ClassifiesTheRateOfTicks)
{
  auto clock = std::make_shared<ManualClock>();
  RateBoundStatus rate(clock, RateBoundStatusParam{9, 11}, RateBoundStatusParam{5, 20});

  auto report = rate.report();
  EXPECT_EQ(report.level, Level::STALE);
  EXPECT_FALSE(report.rate_hz.has_value());

  rate.tick();
  EXPECT_EQ(rate.report().level, Level::STALE);

  // 10 Hz
  clock->advance(100ms);
  rate.tick();
  report = rate.report();
  EXPECT_EQ(report.level, Level::OK);
  ASSERT_TRUE(report.rate_hz.has_value());
  EXPECT_NEAR(report.rate_hz.value(), 10.0, 1e-6);

  // 6.25 Hz
  clock->advance(160ms);
  rate.tick();
  EXPECT_EQ(rate.report().level, Level::WARN);

  // 50 Hz
  clock->advance(20ms);
  rate.tick();
  EXPECT_EQ(rate.report().level, Level::ERROR);

  // 10 Hz
  clock->advance(100ms);
  rate.tick();
  EXPECT_EQ(rate.report().level, Level::OK);
}

TEST(RateBoundStatus, ReportsErrorWhenTicksStop)
{
  auto clock = std::make_shared<ManualClock>();
  RateBoundStatus rate(clock, RateBoundStatusParam{9, 11}, RateBoundStatusParam{5, 20});

  rate.tick();
  clock->advance(100ms);
  rate.tick();
  EXPECT_EQ(rate.report().level, Level::OK);

  clock->advance(1s);
  const auto report = rate.report();
  EXPECT_EQ(report.level, Level::ERROR);
  EXPECT_EQ(report.assumed_skipped_frames, 5U);
}

}  // namespace nebula::diagnostics
