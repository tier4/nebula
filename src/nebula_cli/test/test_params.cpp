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

#include "params.hpp"

#include <gtest/gtest.h>

using nebula::cli::Params;

TEST(Params, AssignmentAndTypedGetters)
{
  Params p;
  p.set_assignment("rotation_speed:=600.0");
  p.set_assignment("frame_id:=lidar");
  EXPECT_EQ(p.get_int("rotation_speed", 0), 600);
  EXPECT_DOUBLE_EQ(p.get_double("rotation_speed", 0), 600.0);
  EXPECT_EQ(p.get_string("frame_id", ""), "lidar");
  EXPECT_EQ(p.get_string("missing", "dflt"), "dflt");
  EXPECT_THROW(p.set_assignment("novalue"), std::runtime_error);
  EXPECT_THROW(p.get_int("frame_id", 0), std::runtime_error);
}

TEST(Params, Substitution)
{
  Params p;
  p.set("sensor_model", "Pandar64");
  p.set("file", "a/$(var sensor_model).csv");
  EXPECT_EQ(p.get_string("file", ""), "a/Pandar64.csv");
  EXPECT_THROW(p.substitute("$(var nope)"), std::runtime_error);
  EXPECT_EQ(p.substitute("plain"), "plain");
}
