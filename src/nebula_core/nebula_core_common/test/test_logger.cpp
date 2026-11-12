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

#include "nebula_core_common/loggers/logger.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers::loggers
{

namespace
{

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

/// A logger that records its messages and whose throttling clock is set by the test.
class RecordingLogger : public Logger
{
public:
  void debug(const std::string & message) override { messages.push_back("DEBUG " + message); }
  void info(const std::string & message) override { messages.push_back("INFO " + message); }
  void warn(const std::string & message) override { messages.push_back("WARN " + message); }
  void error(const std::string & message) override { messages.push_back("ERROR " + message); }

  std::shared_ptr<Logger> child(const std::string &) override
  {
    return std::make_shared<RecordingLogger>();
  }

  void advance(std::chrono::nanoseconds duration) { now_ns_ += duration.count(); }

  std::vector<std::string> messages;

protected:
  uint64_t steady_now_ns() const override { return now_ns_; }

private:
  uint64_t now_ns_{0};
};

}  // namespace

TEST(Logger, ThrottledLetsFirstMessageThrough)
{
  RecordingLogger logger;

  logger.throttled(1s).warn("a");

  EXPECT_EQ(logger.messages, std::vector<std::string>{"WARN a"});
}

TEST(Logger, ThrottledDropsMessagesWithinPeriod)
{
  RecordingLogger logger;

  for (int i = 0; i < 10; ++i) {
    logger.throttled(1s).warn(std::to_string(i));
    logger.advance(300ms);
  }

  // Calls happen at 0, 300, ..., 2700 ms. The ones at 0, 1200 and 2400 ms pass.
  EXPECT_EQ(logger.messages, (std::vector<std::string>{"WARN 0", "WARN 4", "WARN 8"}));
}

TEST(Logger, ThrottledLetsMessageThroughExactlyAfterPeriod)
{
  RecordingLogger logger;

  for (int i = 0; i < 3; ++i) {
    logger.throttled(1s).info(std::to_string(i));
    logger.advance(1s);
  }

  EXPECT_EQ(logger.messages.size(), 3U);
}

TEST(Logger, ThrottledKeepsStatePerCallSite)
{
  RecordingLogger logger;

  for (int i = 0; i < 3; ++i) {
    logger.throttled(1s).warn("first site");
    logger.throttled(1s).error("second site");
  }

  EXPECT_EQ(logger.messages, (std::vector<std::string>{"WARN first site", "ERROR second site"}));
}

TEST(Logger, ThrottledKeepsStatePerLogger)
{
  RecordingLogger logger_a;
  RecordingLogger logger_b;

  for (auto * logger : {&logger_a, &logger_b, &logger_a, &logger_b}) {
    logger->throttled(1s).warn("a");
  }

  EXPECT_EQ(logger_a.messages.size(), 1U);
  EXPECT_EQ(logger_b.messages.size(), 1U);
}

TEST(Logger, OnceLetsOnlyFirstMessageThrough)
{
  RecordingLogger logger;

  for (int i = 0; i < 5; ++i) {
    logger.once().error(std::to_string(i));
    logger.advance(10s);
  }

  EXPECT_EQ(logger.messages, std::vector<std::string>{"ERROR 0"});
}

TEST(Logger, OnceKeepsStatePerCallSiteAndLogger)
{
  RecordingLogger logger_a;
  RecordingLogger logger_b;

  for (auto * logger : {&logger_a, &logger_b, &logger_a, &logger_b}) {
    logger->once().info("first site");
    logger->once().debug("second site");
  }

  const std::vector<std::string> expected{"INFO first site", "DEBUG second site"};
  EXPECT_EQ(logger_a.messages, expected);
  EXPECT_EQ(logger_b.messages, expected);
}

TEST(Logger, FilteredViewTellsWhetherItForwards)
{
  RecordingLogger logger;
  int n_formatted = 0;

  for (int i = 0; i < 3; ++i) {
    if (auto log = logger.once()) {
      ++n_formatted;
      NEBULA_LOG_STREAM(log.warn, "value " << i);
    }
  }

  EXPECT_EQ(n_formatted, 1);
  EXPECT_EQ(logger.messages, std::vector<std::string>{"WARN value 0"});
}

TEST(Logger, FilteredViewWorksWithStreamMacro)
{
  RecordingLogger logger;

  for (int i = 0; i < 3; ++i) {
    NEBULA_LOG_STREAM(logger.throttled(1s).error, "value " << i);
  }

  EXPECT_EQ(logger.messages, std::vector<std::string>{"ERROR value 0"});
}

}  // namespace nebula::drivers::loggers
