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

#include "nebula_robosense_common/robosense_common.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <initializer_list>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

namespace
{

using nebula::Status;
using nebula::drivers::ChannelCorrection;
using nebula::drivers::ReturnMode;
using nebula::drivers::RobosenseCalibrationConfiguration;
using nebula::drivers::RobosenseSensorConfiguration;
using nebula::drivers::SensorModel;

template <typename T>
std::string stream_to_string(const T & value)
{
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

void expect_contains_all(
  const std::string & output, std::initializer_list<std::string_view> expected_substrings)
{
  for (const auto expected_substring : expected_substrings) {
    EXPECT_NE(output.find(expected_substring), std::string::npos) << output;
  }
}

std::filesystem::path make_temp_file_path(std::string_view stem)
{
  return std::filesystem::temp_directory_path() /
         (std::string(stem) + "_" + std::to_string(getpid()) + ".csv");
}

RobosenseSensorConfiguration make_sensor_configuration()
{
  RobosenseSensorConfiguration configuration{};
  configuration.sensor_model = SensorModel::ROBOSENSE_HELIOS;
  configuration.frame_id = "robosense_frame";
  configuration.host_ip = "192.168.1.10";
  configuration.sensor_ip = "192.168.1.200";
  configuration.data_port = 6699;
  configuration.return_mode = ReturnMode::SINGLE_LAST;
  configuration.packet_mtu_size = 1200;
  configuration.use_sensor_time = true;
  configuration.gnss_port = 7788;
  configuration.scan_phase = 45.5;
  configuration.dual_return_distance_threshold = 0.8;
  return configuration;
}

TEST(RobosenseCommonTest, RobosenseReturnModeParsingCoversKnownAndUnknownValues)
{
  EXPECT_EQ(nebula::drivers::return_mode_from_string_robosense("Dual"), ReturnMode::DUAL);
  EXPECT_EQ(
    nebula::drivers::return_mode_from_string_robosense("Strongest"), ReturnMode::SINGLE_STRONGEST);
  EXPECT_EQ(nebula::drivers::return_mode_from_string_robosense("Last"), ReturnMode::SINGLE_LAST);
  EXPECT_EQ(nebula::drivers::return_mode_from_string_robosense("First"), ReturnMode::SINGLE_FIRST);
  EXPECT_EQ(nebula::drivers::return_mode_from_string_robosense("unexpected"), ReturnMode::UNKNOWN);
}

TEST(RobosenseCommonTest, ChannelCorrectionHasValueTracksNanState)
{
  ChannelCorrection empty_correction{};
  EXPECT_FALSE(empty_correction.has_value());

  ChannelCorrection populated_correction{};
  populated_correction.azimuth = 1.25F;
  populated_correction.elevation = -2.5F;
  EXPECT_TRUE(populated_correction.has_value());
}

TEST(RobosenseCommonTest, SensorConfigurationStreamingReflectsConfiguredValues)
{
  const auto configuration = make_sensor_configuration();
  const std::string output = stream_to_string(configuration);

  expect_contains_all(
    output, {"Robosense Sensor Configuration:", "Sensor Model: HELIOS", "Frame ID: robosense_frame",
             "Host IP: 192.168.1.10", "Sensor IP: 192.168.1.200", "Data Port: 6699",
             "Return Mode: SingleLast", "MTU: 1200", "Use Sensor Time: 1", "GNSS Port: 7788",
             "Scan Phase: 45.5"});
}

TEST(RobosenseCommonTest, CalibrationConfigurationLoadsValidCalibrationAndCreatesChannels)
{
  RobosenseCalibrationConfiguration configuration{};
  configuration.set_channel_size(3);

  EXPECT_EQ(
    configuration.load_from_string(
      "Laser id,Elevation,Azimuth\n"
      "1,2.5,10.0\n"
      "2,-1.0,20.0\n"
      "3,0.5,30.0\n"),
    Status::OK);

  EXPECT_FLOAT_EQ(configuration.get_correction(0).elevation, 2.5F);
  EXPECT_FLOAT_EQ(configuration.get_correction(0).azimuth, 10.0F);
  EXPECT_FLOAT_EQ(configuration.get_correction(1).elevation, -1.0F);
  EXPECT_FLOAT_EQ(configuration.get_correction(1).azimuth, 20.0F);
  EXPECT_FLOAT_EQ(configuration.get_correction(2).elevation, 0.5F);
  EXPECT_FLOAT_EQ(configuration.get_correction(2).azimuth, 30.0F);

  configuration.create_corrected_channels();
  EXPECT_EQ(configuration.get_correction(0).channel, 2);
  EXPECT_EQ(configuration.get_correction(1).channel, 0);
  EXPECT_EQ(configuration.get_correction(2).channel, 1);
}

TEST(RobosenseCommonTest, CalibrationConfigurationRejectsDuplicateAndMissingData)
{
  RobosenseCalibrationConfiguration configuration{};
  configuration.set_channel_size(2);

  EXPECT_EQ(
    configuration.load_from_string(
      "Laser id,Elevation,Azimuth\n"
      "1,1.0,2.0\n"
      "1,3.0,4.0\n"),
    Status::INVALID_CALIBRATION_FILE);

  EXPECT_FALSE(configuration.get_correction(0).has_value());
  EXPECT_FALSE(configuration.get_correction(1).has_value());
  EXPECT_TRUE(std::isnan(configuration.get_correction(0).elevation));
  EXPECT_TRUE(std::isnan(configuration.get_correction(0).azimuth));
}

TEST(RobosenseCommonTest, CalibrationConfigurationLoadsAndSavesFiles)
{
  const auto file_path = make_temp_file_path("nebula_robosense_calibration");
  std::filesystem::remove(file_path);

  RobosenseCalibrationConfiguration saved_configuration{};
  saved_configuration.set_channel_size(2);
  saved_configuration.calibration[0] = ChannelCorrection{10.0F, 1.25F};
  saved_configuration.calibration[1] = ChannelCorrection{20.0F, -0.5F};

  ASSERT_EQ(saved_configuration.save_file(file_path.string()), Status::OK);

  RobosenseCalibrationConfiguration loaded_configuration{};
  loaded_configuration.set_channel_size(2);
  ASSERT_EQ(loaded_configuration.load_from_file(file_path.string()), Status::OK);

  EXPECT_FLOAT_EQ(loaded_configuration.get_correction(0).azimuth, 10.0F);
  EXPECT_FLOAT_EQ(loaded_configuration.get_correction(0).elevation, 1.25F);
  EXPECT_FLOAT_EQ(loaded_configuration.get_correction(1).azimuth, 20.0F);
  EXPECT_FLOAT_EQ(loaded_configuration.get_correction(1).elevation, -0.5F);

  std::filesystem::remove(file_path);
}

TEST(RobosenseCommonTest, CalibrationConfigurationReportsFileErrors)
{
  const auto missing_directory_file = std::filesystem::temp_directory_path() /
                                      ("nebula_robosense_missing_" + std::to_string(getpid())) /
                                      "calibration.csv";

  RobosenseCalibrationConfiguration configuration{};
  configuration.set_channel_size(1);
  configuration.calibration[0] = ChannelCorrection{10.0F, 1.25F};

  EXPECT_EQ(
    configuration.load_from_file(missing_directory_file.string()),
    Status::INVALID_CALIBRATION_FILE);
  EXPECT_EQ(configuration.save_file(missing_directory_file.string()), Status::CANNOT_SAVE_FILE);
}

}  // namespace
