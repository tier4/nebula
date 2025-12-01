// Copyright 2025 TIER IV, Inc.
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

#include "nebula_robosense_common/robosense_common.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <sstream>
#include <string>

using nebula::drivers::ChannelCorrection;
using nebula::drivers::return_mode_from_string_robosense;
using nebula::drivers::ReturnMode;
using nebula::drivers::RobosenseCalibrationConfiguration;

// ============================================================================
// return_mode_from_string_robosense tests
// ============================================================================

TEST(RobosenseCommonTest, ReturnModeFromStringDual)
{
  EXPECT_EQ(return_mode_from_string_robosense("Dual"), ReturnMode::DUAL);
}

TEST(RobosenseCommonTest, ReturnModeFromStringStrongest)
{
  EXPECT_EQ(return_mode_from_string_robosense("Strongest"), ReturnMode::SINGLE_STRONGEST);
}

TEST(RobosenseCommonTest, ReturnModeFromStringLast)
{
  EXPECT_EQ(return_mode_from_string_robosense("Last"), ReturnMode::SINGLE_LAST);
}

TEST(RobosenseCommonTest, ReturnModeFromStringFirst)
{
  EXPECT_EQ(return_mode_from_string_robosense("First"), ReturnMode::SINGLE_FIRST);
}

TEST(RobosenseCommonTest, ReturnModeFromStringUnknown)
{
  EXPECT_EQ(return_mode_from_string_robosense("Invalid"), ReturnMode::UNKNOWN);
  EXPECT_EQ(return_mode_from_string_robosense(""), ReturnMode::UNKNOWN);
  EXPECT_EQ(return_mode_from_string_robosense("dual"), ReturnMode::UNKNOWN);  // Case sensitive
}

// ============================================================================
// ChannelCorrection tests
// ============================================================================

TEST(ChannelCorrectionTest, DefaultConstruction)
{
  ChannelCorrection correction{};

  EXPECT_TRUE(std::isnan(correction.azimuth));
  EXPECT_TRUE(std::isnan(correction.elevation));
  EXPECT_EQ(correction.channel, 0);
}

TEST(ChannelCorrectionTest, HasValueWhenSet)
{
  ChannelCorrection correction{1.5f, 2.5f, 3};

  EXPECT_TRUE(correction.has_value());
  EXPECT_FLOAT_EQ(correction.azimuth, 1.5f);
  EXPECT_FLOAT_EQ(correction.elevation, 2.5f);
  EXPECT_EQ(correction.channel, 3);
}

TEST(ChannelCorrectionTest, HasValueFalseWhenNaN)
{
  ChannelCorrection correction{};
  EXPECT_FALSE(correction.has_value());

  ChannelCorrection partial_nan{1.5f, NAN, 0};
  EXPECT_FALSE(partial_nan.has_value());

  ChannelCorrection partial_nan2{NAN, 2.5f, 0};
  EXPECT_FALSE(partial_nan2.has_value());
}

// ============================================================================
// RobosenseCalibrationConfiguration tests
// ============================================================================

TEST(RobosenseCalibrationConfigTest, DefaultConstruction)
{
  RobosenseCalibrationConfiguration config;

  EXPECT_TRUE(config.calibration.empty());
}

TEST(RobosenseCalibrationConfigTest, SetChannelSize)
{
  RobosenseCalibrationConfiguration config;

  config.set_channel_size(32);
  EXPECT_EQ(config.calibration.size(), 32);

  config.set_channel_size(64);
  EXPECT_EQ(config.calibration.size(), 64);
}

TEST(RobosenseCalibrationConfigTest, LoadFromValidStream)
{
  RobosenseCalibrationConfiguration config;
  config.set_channel_size(3);

  std::stringstream ss;
  ss << "Laser id,Elevation,Azimuth\n";
  ss << "1,10.5,0.5\n";
  ss << "2,5.5,0.3\n";
  ss << "3,-10.0,0.1\n";

  auto status = config.load_from_stream(ss);

  EXPECT_EQ(status, nebula::Status::OK);
  EXPECT_TRUE(config.calibration[0].has_value());
  EXPECT_FLOAT_EQ(config.calibration[0].elevation, 10.5f);
  EXPECT_FLOAT_EQ(config.calibration[0].azimuth, 0.5f);
}

TEST(RobosenseCalibrationConfigTest, LoadFromString)
{
  RobosenseCalibrationConfiguration config;
  config.set_channel_size(3);

  std::string content =
    "Laser id,Elevation,Azimuth\n"
    "1,10.5,0.5\n"
    "2,5.5,0.3\n"
    "3,-10.0,0.1\n";

  auto status = config.load_from_string(content);

  EXPECT_EQ(status, nebula::Status::OK);
  EXPECT_EQ(config.calibration.size(), 3);
}

TEST(RobosenseCalibrationConfigTest, GetCorrection)
{
  RobosenseCalibrationConfiguration config;
  config.set_channel_size(3);

  std::string content =
    "Laser id,Elevation,Azimuth\n"
    "1,10.5,0.5\n"
    "2,5.5,0.3\n"
    "3,-10.0,0.1\n";

  config.load_from_string(content);

  auto c0 = config.get_correction(0);
  EXPECT_FLOAT_EQ(c0.elevation, 10.5f);
  EXPECT_FLOAT_EQ(c0.azimuth, 0.5f);

  auto c2 = config.get_correction(2);
  EXPECT_FLOAT_EQ(c2.elevation, -10.0f);
  EXPECT_FLOAT_EQ(c2.azimuth, 0.1f);
}

TEST(RobosenseCalibrationConfigTest, CreateCorrectedChannels)
{
  RobosenseCalibrationConfiguration config;
  config.set_channel_size(3);

  std::string content =
    "Laser id,Elevation,Azimuth\n"
    "1,10.5,0.5\n"    // Highest elevation -> channel 2
    "2,5.5,0.3\n"     // Middle elevation -> channel 1
    "3,-10.0,0.1\n";  // Lowest elevation -> channel 0

  config.load_from_string(content);
  config.create_corrected_channels();

  // Channels should be ordered by elevation (lowest = 0)
  EXPECT_EQ(config.calibration[0].channel, 2);  // 10.5 is highest
  EXPECT_EQ(config.calibration[1].channel, 1);  // 5.5 is middle
  EXPECT_EQ(config.calibration[2].channel, 0);  // -10.0 is lowest
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
