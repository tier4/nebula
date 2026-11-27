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

#include "converters.hpp"

#include <gtest/gtest.h>

#include <string>

using nebula::cli::make_converter;
using nebula::cli::Params;

namespace
{
const nebula::cli::Converter::CloudCallback kNoop = [](const auto &, double) {};

Params make(const std::string & model, const std::string & mode)
{
  Params p;
  p.set("sensor_model", model);
  if (!mode.empty()) p.set("return_mode", mode);
  return p;
}
}  // namespace

TEST(MakeConverter, UnknownVendor)
{
  EXPECT_THROW(make_converter("acme", make("VLP16", "Strongest"), kNoop), std::runtime_error);
}

TEST(MakeConverter, BadModelOrMode)
{
  EXPECT_THROW(make_converter("hesai", make("Nope", "Strongest"), kNoop), std::runtime_error);
  EXPECT_THROW(make_converter("hesai", make("Pandar64", "Nope"), kNoop), std::runtime_error);
  EXPECT_THROW(make_converter("velodyne", make("VLP16", "Nope"), kNoop), std::runtime_error);
  EXPECT_THROW(make_converter("velodyne", make("", "Strongest"), kNoop), std::runtime_error);
}

TEST(MakeConverter, MissingCalibrationFile)
{
  auto p = make("VLP16", "Strongest");
  p.set("calibration_file", "/nonexistent/calib.yaml");
  EXPECT_THROW(make_converter("velodyne", p, kNoop), std::runtime_error);
  p = make("Pandar64", "Strongest");
  p.set("calibration_file", "/nonexistent/calib.csv");
  EXPECT_THROW(make_converter("hesai", p, kNoop), std::runtime_error);
  p = make("PandarAT128", "Strongest");
  p.set("correction_file", "/nonexistent/corr.dat");
  EXPECT_THROW(make_converter("hesai", p, kNoop), std::runtime_error);
}

TEST(MakeConverter, ShippedCalibrationLoads)
{
  auto velodyne = make_converter("velodyne", make("VLP16", "Strongest"), kNoop);
  ASSERT_NE(velodyne, nullptr);
  EXPECT_EQ(velodyne->dropped_packets(), 0u);
  // garbage is not a scan: must not throw or emit
  velodyne->feed_scan({0, std::vector<uint8_t>(10)});

  auto hesai = make_converter("hesai", make("Pandar64", "Strongest"), kNoop);
  ASSERT_NE(hesai, nullptr);
  hesai->feed_scan({0, std::vector<uint8_t>(10)});
  EXPECT_EQ(hesai->dropped_packets(), 1u);
}

TEST(MakeConverter, RobosenseDropsScansWithoutInfo)
{
  auto c = make_converter("robosense", make("Helios", ""), kNoop);
  ASSERT_NE(c, nullptr);
  c->feed_scan({0, std::vector<uint8_t>(1248)});
  c->feed_scan({0, std::vector<uint8_t>(1248)});
  EXPECT_EQ(c->dropped_packets(), 2u);
  c->feed_info({0, std::vector<uint8_t>(10)});  // invalid info is ignored
  c->feed_scan({0, std::vector<uint8_t>(1248)});
  EXPECT_EQ(c->dropped_packets(), 3u);
}
