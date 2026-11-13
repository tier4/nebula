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

#include "args.hpp"

#include <gtest/gtest.h>

#include <initializer_list>
#include <string>
#include <vector>

using nebula::cli::Args;

namespace
{
Args parse(std::initializer_list<const char *> flags)
{
  std::vector<std::string> store{"nebula_cli", "convert"};
  for (const auto * f : flags) store.emplace_back(f);
  std::vector<char *> argv;
  for (auto & s : store) argv.push_back(s.data());
  return nebula::cli::parse_args(static_cast<int>(argv.size()), argv.data());
}
}  // namespace

TEST(ParseArgs, ImplicitSensorWithDefaults)
{
  const auto a = parse({"--input", "in", "--output", "out", "--vendor", "hesai", "--topic", "/p"});
  EXPECT_EQ(a.input, "in");
  EXPECT_EQ(a.output, "out");
  EXPECT_EQ(a.output_storage, "sqlite3");
  EXPECT_TRUE(a.copy_other);
  EXPECT_EQ(a.max_clouds, 0u);
  ASSERT_EQ(a.sensors.size(), 1u);
  EXPECT_EQ(a.sensors[0].output_topic, "/points");
  EXPECT_EQ(a.sensors[0].point_format, "full");
}

TEST(ParseArgs, SensorFlagsAndGlobals)
{
  const auto a = parse(
    {"--input", "in", "--output", "out", "--output-storage", "mcap", "--max-clouds", "5",
     "--drop-other-topics", "--sensor", "--vendor", "velodyne", "--topic", "/a", "--model", "VLP16",
     "--return-mode", "Dual", "--calibration-file", "c.yaml", "--frame-id", "f", "--params-file",
     "p.yaml", "-p", "cut_angle:=1", "--point-format", "xyzir", "--output-topic", "/o1", "--sensor",
     "--vendor", "robosense", "--topic", "/b", "--info-topic", "/i", "--output-topic", "/o2"});
  EXPECT_EQ(a.output_storage, "mcap");
  EXPECT_EQ(a.max_clouds, 5u);
  EXPECT_FALSE(a.copy_other);
  ASSERT_EQ(a.sensors.size(), 2u);
  const auto & s = a.sensors[0];
  EXPECT_EQ(s.vendor, "velodyne");
  EXPECT_EQ(s.params_file, "p.yaml");
  EXPECT_EQ(s.point_format, "xyzir");
  ASSERT_EQ(s.assignments.size(), 1u);
  EXPECT_EQ(s.assignments[0], "cut_angle:=1");
  ASSERT_EQ(s.flag_params.size(), 4u);
  EXPECT_EQ(s.flag_params[0].first, "sensor_model");
  EXPECT_EQ(s.flag_params[0].second, "VLP16");
  EXPECT_EQ(a.sensors[1].info_topic, "/i");
}

TEST(ParseArgs, Errors)
{
  EXPECT_THROW(parse({"--input"}), std::runtime_error);
  EXPECT_THROW(parse({"--bogus", "x"}), std::runtime_error);
  EXPECT_THROW(parse({"--output", "o", "--vendor", "hesai", "--topic", "/t"}), std::runtime_error);
  EXPECT_THROW(parse({"--input", "i", "--vendor", "hesai", "--topic", "/t"}), std::runtime_error);
  EXPECT_THROW(parse({"--input", "i", "--output", "o"}), std::runtime_error);
  EXPECT_THROW(parse({"--input", "i", "--output", "o", "--topic", "/t"}), std::runtime_error);
  EXPECT_THROW(parse({"--input", "i", "--output", "o", "--vendor", "hesai"}), std::runtime_error);
  EXPECT_THROW(
    parse({"--input", "i", "--output", "o", "--vendor", "hesai", "--topic", "/t", "--info-topic",
           "/i"}),
    std::runtime_error);
  EXPECT_THROW(
    parse({"--input", "i", "--output", "o", "--vendor", "hesai", "--topic", "/t", "--point-format",
           "bad"}),
    std::runtime_error);
  EXPECT_THROW(
    parse({"--input", "i", "--output", "o", "--vendor", "hesai", "--topic", "/t", "--max-clouds",
           "x"}),
    std::exception);
}

TEST(ParseArgs, MultiSensorRules)
{
  // several sensors need explicit output topics
  EXPECT_THROW(
    parse({"--input", "i", "--output", "o", "--sensor", "--vendor", "hesai", "--topic", "/a",
           "--sensor", "--vendor", "hesai", "--topic", "/b"}),
    std::runtime_error);
  // a topic feeds one sensor only
  EXPECT_THROW(
    parse({"--input", "i", "--output", "o", "--sensor", "--vendor", "hesai", "--topic", "/a",
           "--output-topic", "/x", "--sensor", "--vendor", "hesai", "--topic", "/a",
           "--output-topic", "/y"}),
    std::runtime_error);
  // output must not collide with an input topic or another output
  EXPECT_THROW(
    parse({"--input", "i", "--output", "o", "--vendor", "hesai", "--topic", "/a", "--output-topic",
           "/a"}),
    std::runtime_error);
  EXPECT_THROW(
    parse({"--input", "i", "--output", "o", "--sensor", "--vendor", "hesai", "--topic", "/a",
           "--output-topic", "/x", "--sensor", "--vendor", "hesai", "--topic", "/b",
           "--output-topic", "/x"}),
    std::runtime_error);
}

TEST(ParseArgs, ThreadsDefaultAndOverride)
{
  EXPECT_EQ(
    parse({"--input", "in", "--output", "out", "--vendor", "hesai", "--topic", "/p"}).threads, 4u);
  EXPECT_EQ(
    parse({"--input", "in", "--output", "out", "--vendor", "hesai", "--topic", "/p", "--threads",
           "2"})
      .threads,
    2u);
  EXPECT_THROW(
    parse({"--input", "in", "--output", "out", "--vendor", "hesai", "--topic", "/p", "--threads",
           "0"}),
    std::runtime_error);
}
