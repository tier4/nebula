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

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <nebula_core_common/diagnostics/severity_latch.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

namespace nebula::ros
{

using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{

DiagnosticStatus make_status(uint8_t level, const std::string & message)
{
  DiagnosticStatus status;
  status.level = level;
  status.message = message;
  return status;
}

DiagnosticStatus run(SeverityLatch & latch)
{
  diagnostic_updater::DiagnosticStatusWrapper status;
  latch.run(status);
  return static_cast<const DiagnosticStatus &>(status);
}

}  // namespace

TEST(SeverityLatch, IsStaleBeforeTheFirstSubmission)
{
  SeverityLatch latch("latch");

  const auto status = run(latch);
  EXPECT_EQ(status.level, DiagnosticStatus::STALE);
  EXPECT_EQ(status.message, "No status available");
}

TEST(SeverityLatch, ReportsTheWorstStatusBeforeTheFirstRun)
{
  SeverityLatch latch("latch");

  latch.submit(make_status(DiagnosticStatus::WARN, "warn"));
  latch.submit(make_status(DiagnosticStatus::ERROR, "error"));
  latch.submit(make_status(DiagnosticStatus::OK, "ok"));

  const auto status = run(latch);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "error");
}

TEST(SeverityLatch, ReportsTheLastOfEquallySevereStatuses)
{
  SeverityLatch latch("latch");

  latch.submit(make_status(DiagnosticStatus::ERROR, "first"));
  latch.submit(make_status(DiagnosticStatus::ERROR, "second"));

  EXPECT_EQ(run(latch).message, "second");
}

TEST(SeverityLatch, ReportsKeyValuesOfTheLatchedStatus)
{
  SeverityLatch latch("latch");

  auto submitted = make_status(DiagnosticStatus::ERROR, "error");
  submitted.values.emplace_back();
  submitted.values.back().key = "code";
  submitted.values.back().value = "42";
  latch.submit(submitted);
  latch.submit(make_status(DiagnosticStatus::OK, "ok"));

  const auto status = run(latch);
  ASSERT_EQ(status.values.size(), 1U);
  EXPECT_EQ(status.values[0].key, "code");
  EXPECT_EQ(status.values[0].value, "42");
}

TEST(SeverityLatch, RepeatsTheLastStatusWithoutNewSubmissions)
{
  SeverityLatch latch("latch");

  latch.submit(make_status(DiagnosticStatus::ERROR, "error"));

  EXPECT_EQ(run(latch).level, DiagnosticStatus::ERROR);
  EXPECT_EQ(run(latch).level, DiagnosticStatus::ERROR);
}

TEST(SeverityLatch, ReplacesAReportedStatusWithALessSevereOne)
{
  SeverityLatch latch("latch");

  latch.submit(make_status(DiagnosticStatus::ERROR, "error"));
  EXPECT_EQ(run(latch).level, DiagnosticStatus::ERROR);

  latch.submit(make_status(DiagnosticStatus::OK, "ok"));
  const auto status = run(latch);
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "ok");
}

TEST(SeverityLatch, ReportsTheWorstStatusInEveryCycle)
{
  SeverityLatch latch("latch");

  // An error that is followed by less severe statuses within the same cycle must not be lost
  latch.submit(make_status(DiagnosticStatus::WARN, "warn"));
  latch.submit(make_status(DiagnosticStatus::ERROR, "error"));
  latch.submit(make_status(DiagnosticStatus::OK, "ok"));
  auto status = run(latch);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "error");

  // The error of the previous cycle must not carry over
  latch.submit(make_status(DiagnosticStatus::OK, "ok 1"));
  latch.submit(make_status(DiagnosticStatus::OK, "ok 2"));
  status = run(latch);
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "ok 2");

  // Latching still works in cycles after the first one
  latch.submit(make_status(DiagnosticStatus::OK, "ok"));
  latch.submit(make_status(DiagnosticStatus::WARN, "warn"));
  latch.submit(make_status(DiagnosticStatus::OK, "ok"));
  status = run(latch);
  EXPECT_EQ(status.level, DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "warn");
}

}  // namespace nebula::ros
