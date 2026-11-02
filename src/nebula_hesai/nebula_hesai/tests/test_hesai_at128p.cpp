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

#include "nebula_hesai_decoders/decoders/pandar_at128.hpp"

#include <gtest/gtest.h>

#include <vector>

using nebula::drivers::PandarAT128P;
using nebula::drivers::ReturnType;
namespace return_mode = nebula::drivers::hesai_packet::return_mode;
using Unit = PandarAT128P::packet_t::body_t::block_t::unit_t;

TEST(TestPandarAT128P, StrongestFirstStoresStrongestInFirstBlock)
{
  PandarAT128P sensor;
  Unit strongest{1000, 200, 0};
  Unit first{500, 50, 0};
  const std::vector<const Unit *> units{&strongest, &first};

  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 0, units), ReturnType::STRONGEST);
  EXPECT_EQ(sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 1, units), ReturnType::FIRST);
}

TEST(TestPandarAT128P, StrongestFirstWhenFirstIsAlsoStrongest)
{
  // If the first return is also the strongest, the first block holds the second strongest return
  PandarAT128P sensor;
  Unit second_strongest{1000, 40, 0};
  Unit first_strongest{500, 90, 0};
  const std::vector<const Unit *> units{&second_strongest, &first_strongest};

  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 0, units),
    ReturnType::SECONDSTRONGEST);
  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 1, units),
    ReturnType::FIRST_STRONGEST);
}

TEST(TestPandarAT128P, LastStrongestIsUnchanged)
{
  PandarAT128P sensor;
  Unit last{1000, 10, 0};
  Unit strongest{500, 90, 0};
  const std::vector<const Unit *> units{&last, &strongest};

  EXPECT_EQ(sensor.get_return_type(return_mode::DUAL_LAST_STRONGEST, 0, units), ReturnType::LAST);
  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_LAST_STRONGEST, 1, units), ReturnType::STRONGEST);
}

TEST(TestPandarAT128P, IdenticalReturnsAreReportedAsIdentical)
{
  PandarAT128P sensor;
  Unit a{1000, 90, 0};
  Unit b{1000, 90, 0};
  const std::vector<const Unit *> units{&a, &b};

  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 0, units), ReturnType::IDENTICAL);
  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 1, units), ReturnType::IDENTICAL);
}
