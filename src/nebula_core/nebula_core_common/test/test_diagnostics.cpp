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

#include "nebula_core_common/diagnostics/diagnostic_task.hpp"
#include "nebula_core_common/diagnostics/liveness_monitor.hpp"
#include "nebula_core_common/diagnostics/rate_bound_status.hpp"
#include "nebula_core_common/diagnostics/severity_latch.hpp"
#include "nebula_core_common/util/clock.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

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

class FixedTask final : public DiagnosticTask
{
public:
  FixedTask(const std::string & name, uint8_t level, std::string message)
  : DiagnosticTask(name), level_(level), message_(std::move(message))
  {
  }

  void run(DiagnosticStatus & status) override
  {
    set_summary(status, level_, message_);
    add_value(status, name(), "ran");
  }

private:
  uint8_t level_;
  std::string message_;
};

DiagnosticStatus run(DiagnosticTask & task)
{
  DiagnosticStatus status;
  task.run(status);
  return status;
}

std::string value_of(const DiagnosticStatus & status, const std::string & key)
{
  for (const auto & kv : status.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  ADD_FAILURE() << "Key not found: " << key;
  return "";
}

}  // namespace

TEST(DiagnosticTask, MergeSummaryKeepsWorstLevelAndJoinsNonOkMessages)
{
  DiagnosticStatus status;

  merge_summary(status, DiagnosticStatus::OK, "fine");
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "");

  merge_summary(status, DiagnosticStatus::WARN, "warn");
  EXPECT_EQ(status.level, DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "warn");

  merge_summary(status, DiagnosticStatus::OK, "fine");
  EXPECT_EQ(status.level, DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "warn");

  merge_summary(status, DiagnosticStatus::ERROR, "error");
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "warn; error");
}

TEST(DiagnosticTask, CompositeRunsAllChildrenAndMergesThem)
{
  FixedTask ok("a", DiagnosticStatus::OK, "fine");
  FixedTask error("b", DiagnosticStatus::ERROR, "broken");
  FixedTask warn("c", DiagnosticStatus::WARN, "odd");

  CompositeDiagnosticTask composite("composite");
  composite.add_task(ok);
  composite.add_task(error);
  composite.add_task(warn);

  const auto status = run(composite);
  EXPECT_EQ(composite.name(), "composite");
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "broken; odd");
  ASSERT_EQ(status.values.size(), 3U);
  EXPECT_EQ(value_of(status, "c"), "ran");
}

TEST(LivenessMonitor, DeadOnlyAfterTimeoutWithoutTick)
{
  auto clock = std::make_shared<ManualClock>();
  LivenessMonitor monitor("liveness", clock, 100ms);

  clock->advance(99ms);
  EXPECT_EQ(run(monitor).level, DiagnosticStatus::OK);

  clock->advance(1ms);
  auto status = run(monitor);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Dead");
  EXPECT_EQ(value_of(status, "Is alive"), "false");
  EXPECT_EQ(value_of(status, "Lateness [ms]"), std::to_string(100.0));

  monitor.tick();
  status = run(monitor);
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "Alive");
  EXPECT_EQ(value_of(status, "Last tick [s]"), std::to_string(1.1));
}

TEST(SeverityLatch, ReportsWorstStatusSinceLastRun)
{
  SeverityLatch latch("latch");
  EXPECT_EQ(run(latch).level, DiagnosticStatus::STALE);

  DiagnosticStatus error;
  set_summary(error, DiagnosticStatus::ERROR, "error");
  add_value(error, "code", 42);
  DiagnosticStatus ok;
  set_summary(ok, DiagnosticStatus::OK, "ok");

  latch.submit(error);
  latch.submit(ok);
  auto status = run(latch);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "error");
  EXPECT_EQ(value_of(status, "code"), "42");

  // Without a new submission, the last status is reported again
  EXPECT_EQ(run(latch).level, DiagnosticStatus::ERROR);

  // A status that was already reported is replaced even by a less severe one
  latch.submit(ok);
  EXPECT_EQ(run(latch).level, DiagnosticStatus::OK);
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

  EXPECT_EQ(run(rate).level, DiagnosticStatus::STALE);

  rate.tick();
  EXPECT_EQ(run(rate).level, DiagnosticStatus::STALE);

  // 10 Hz
  clock->advance(100ms);
  rate.tick();
  auto status = run(rate);
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(value_of(status, "Publish rate"), "10.00");

  // 6.25 Hz
  clock->advance(160ms);
  rate.tick();
  EXPECT_EQ(run(rate).level, DiagnosticStatus::WARN);

  // 50 Hz
  clock->advance(20ms);
  rate.tick();
  EXPECT_EQ(run(rate).level, DiagnosticStatus::ERROR);

  // 10 Hz
  clock->advance(100ms);
  rate.tick();
  EXPECT_EQ(run(rate).level, DiagnosticStatus::OK);
}

TEST(RateBoundStatus, ReportsErrorWhenTicksStop)
{
  auto clock = std::make_shared<ManualClock>();
  RateBoundStatus rate(clock, RateBoundStatusParam{9, 11}, RateBoundStatusParam{5, 20});

  rate.tick();
  clock->advance(100ms);
  rate.tick();
  EXPECT_EQ(run(rate).level, DiagnosticStatus::OK);

  clock->advance(1s);
  const auto status = run(rate);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(value_of(status, "Assumed skipped frames"), "5");
}

TEST(SteadyClock, DoesNotGoBackwards)
{
  util::SteadyClock clock;
  const auto t0 = clock.now();
  EXPECT_GE(clock.now(), t0);
}

}  // namespace nebula::diagnostics
