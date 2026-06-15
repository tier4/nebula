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

#include <nebula_core_common/nebula_common.hpp>

#include <gtest/gtest.h>

#include <sstream>
#include <string>
#include <utility>
#include <vector>

using nebula::drivers::return_mode_from_string;
using nebula::drivers::ReturnMode;

TEST(TestCoreTypes, ReturnModeRoundTrip)
{
  const std::vector<std::pair<ReturnMode, std::string>> cases = {
    {ReturnMode::FIRST, "First"},
    {ReturnMode::LAST, "Last"},
    {ReturnMode::STRONGEST, "Strongest"},
    {ReturnMode::SINGLE_FIRST, "SingleFirst"},
    {ReturnMode::SINGLE_STRONGEST, "SingleStrongest"},
    {ReturnMode::SINGLE_LAST, "SingleLast"},
    {ReturnMode::DUAL, "Dual"},
    {ReturnMode::DUAL_LAST_STRONGEST, "LastStrongest"},
    {ReturnMode::DUAL_LAST_FIRST, "LastFirst"},
    {ReturnMode::DUAL_FIRST_STRONGEST, "FirstStrongest"},
  };

  for (const auto & test_case : cases) {
    std::stringstream stream;
    stream << test_case.first;
    EXPECT_EQ(stream.str(), test_case.second);
    EXPECT_EQ(return_mode_from_string(test_case.second), test_case.first);
  }

  EXPECT_EQ(return_mode_from_string("DualOnly"), ReturnMode::UNKNOWN);
  EXPECT_EQ(return_mode_from_string("DualFirst"), ReturnMode::UNKNOWN);
  EXPECT_EQ(return_mode_from_string("DualLast"), ReturnMode::UNKNOWN);
  EXPECT_EQ(return_mode_from_string("Triple"), ReturnMode::UNKNOWN);
  EXPECT_EQ(return_mode_from_string("Unknown"), ReturnMode::UNKNOWN);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
