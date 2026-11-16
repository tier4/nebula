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

#include "nebula_core_common/diagnostics/level.hpp"
#include "nebula_core_common/diagnostics/severity_latch.hpp"

#include <gtest/gtest.h>

#include <string>

namespace nebula::diagnostics
{

TEST(SeverityLatch, ReportsNothingBeforeTheFirstSubmission)
{
  SeverityLatch<std::string> latch;

  EXPECT_FALSE(latch.report().has_value());
}

TEST(SeverityLatch, ReportsTheWorstStatusBeforeTheFirstReport)
{
  SeverityLatch<std::string> latch;

  latch.submit(Level::WARN, "warn");
  latch.submit(Level::ERROR, "error");
  latch.submit(Level::OK, "ok");

  const auto report = latch.report();
  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->level, Level::ERROR);
  EXPECT_EQ(report->status, "error");
}

TEST(SeverityLatch, ReportsTheLastOfEquallySevereStatuses)
{
  SeverityLatch<std::string> latch;

  latch.submit(Level::ERROR, "first");
  latch.submit(Level::ERROR, "second");

  const auto report = latch.report();
  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->status, "second");
}

TEST(SeverityLatch, RepeatsTheLastStatusWithoutNewSubmissions)
{
  SeverityLatch<std::string> latch;

  latch.submit(Level::ERROR, "error");

  for (int i = 0; i < 2; ++i) {
    const auto report = latch.report();
    ASSERT_TRUE(report.has_value());
    EXPECT_EQ(report->level, Level::ERROR);
    EXPECT_EQ(report->status, "error");
  }
}

TEST(SeverityLatch, ReplacesAReportedStatusWithALessSevereOne)
{
  SeverityLatch<std::string> latch;

  latch.submit(Level::ERROR, "error");
  auto report = latch.report();
  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->level, Level::ERROR);

  latch.submit(Level::OK, "ok");
  report = latch.report();
  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->level, Level::OK);
  EXPECT_EQ(report->status, "ok");
}

TEST(SeverityLatch, ReportsTheWorstStatusInEveryCycle)
{
  SeverityLatch<std::string> latch;

  // An error that is followed by less severe statuses within the same cycle must not be lost
  latch.submit(Level::WARN, "warn");
  latch.submit(Level::ERROR, "error");
  latch.submit(Level::OK, "ok");
  auto report = latch.report();
  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->level, Level::ERROR);
  EXPECT_EQ(report->status, "error");

  // The error of the previous cycle must not carry over
  latch.submit(Level::OK, "ok 1");
  latch.submit(Level::OK, "ok 2");
  report = latch.report();
  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->level, Level::OK);
  EXPECT_EQ(report->status, "ok 2");

  // Latching still works in cycles after the first one
  latch.submit(Level::OK, "ok");
  latch.submit(Level::WARN, "warn");
  latch.submit(Level::OK, "ok");
  report = latch.report();
  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->level, Level::WARN);
  EXPECT_EQ(report->status, "warn");
}

}  // namespace nebula::diagnostics
