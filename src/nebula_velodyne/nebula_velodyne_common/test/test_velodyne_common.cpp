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

#include "nebula_velodyne_common/velodyne_common.hpp"
#include "nebula_velodyne_common/velodyne_status.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <sstream>
#include <string>
#include <string_view>

namespace
{

using nebula::Status;
using nebula::VelodyneStatus;
using nebula::drivers::ReturnMode;
using nebula::drivers::SensorModel;
using nebula::drivers::VelodyneCalibration;
using nebula::drivers::VelodyneCalibrationConfiguration;
using nebula::drivers::VelodyneSensorConfiguration;

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
         (std::string(stem) + "_" + std::to_string(getpid()) + ".yaml");
}

void write_text_file(const std::filesystem::path & path, const std::string & content)
{
  std::ofstream output(path);
  output << content;
}

TEST(VelodyneCommonTest, VelodyneReturnModeParsingCoversKnownAndUnknownValues)
{
  EXPECT_EQ(
    nebula::drivers::return_mode_from_string_velodyne("Strongest"), ReturnMode::SINGLE_STRONGEST);
  EXPECT_EQ(nebula::drivers::return_mode_from_string_velodyne("Last"), ReturnMode::SINGLE_LAST);
  EXPECT_EQ(nebula::drivers::return_mode_from_string_velodyne("Dual"), ReturnMode::DUAL_ONLY);
  EXPECT_EQ(nebula::drivers::return_mode_from_string_velodyne("unexpected"), ReturnMode::UNKNOWN);
}

TEST(VelodyneCommonTest, DefaultInitializedSensorConfigurationStreamsWithoutCrashing)
{
  const VelodyneSensorConfiguration configuration{};
  EXPECT_NO_THROW({
    const auto output = stream_to_string(configuration);
    EXPECT_FALSE(output.empty());
  });
}

TEST(VelodyneCommonTest, SensorConfigurationStreamingReflectsConfiguredValues)
{
  VelodyneSensorConfiguration configuration{};
  configuration.sensor_model = SensorModel::VELODYNE_VLS128;
  configuration.frame_id = "velodyne_frame";
  configuration.host_ip = "192.168.1.10";
  configuration.sensor_ip = "192.168.1.201";
  configuration.data_port = 2368;
  configuration.return_mode = ReturnMode::DUAL_ONLY;
  configuration.packet_mtu_size = 1500;
  configuration.use_sensor_time = true;
  configuration.gnss_port = 8308;
  configuration.scan_phase = 90.5;
  configuration.rotation_speed = 600;
  configuration.cloud_min_angle = 100;
  configuration.cloud_max_angle = 300;

  const auto output = stream_to_string(configuration);

  expect_contains_all(
    output, {"Velodyne Sensor Configuration:", "Sensor Model: VLS128", "Frame ID: velodyne_frame",
             "Host IP: 192.168.1.10", "Sensor IP: 192.168.1.201", "Data Port: 2368",
             "Return Mode: Dual", "MTU: 1500", "Use Sensor Time: 1", "GNSS Port: 8308",
             "Scan Phase: 90.5", "Rotation Speed: 600", "FoV Start: 100", "FoV End: 300"});
}

TEST(VelodyneCommonTest, VelodyneStatusStreamingCoversCustomAndBaseStatuses)
{
  EXPECT_EQ(stream_to_string(VelodyneStatus{}), "OK");
  EXPECT_EQ(
    stream_to_string(VelodyneStatus{VelodyneStatus::INVALID_RPM_ERROR}),
    "Invalid rotation speed value(range from 300 to 1200, in increments of 60)");
  EXPECT_EQ(
    stream_to_string(VelodyneStatus{VelodyneStatus::INVALID_FOV_ERROR}),
    "Invalid fov value(0 to 359)");
  EXPECT_EQ(
    stream_to_string(VelodyneStatus{VelodyneStatus::INVALID_RETURN_MODE_ERROR}),
    "Invalid return mode(only SINGLE_STRONGEST, SINGLE_LAST, DUAL_ONLY)");
  EXPECT_EQ(
    stream_to_string(VelodyneStatus{Status::INVALID_CALIBRATION_FILE}), "Invalid Calibration File");

  EXPECT_EQ(
    VelodyneStatus{VelodyneStatus::INVALID_FOV_ERROR},
    VelodyneStatus{VelodyneStatus::INVALID_FOV_ERROR});
  EXPECT_NE(
    VelodyneStatus{VelodyneStatus::INVALID_FOV_ERROR},
    VelodyneStatus{VelodyneStatus::INVALID_RPM_ERROR});
}

TEST(VelodyneCommonTest, CalibrationDecoderReadsYAMLAndPopulatesDerivedFields)
{
  const auto calibration_file = make_temp_file_path("nebula_velodyne_calibration_input");
  const auto calibration_yaml = R"(num_lasers: 2
distance_resolution: 0.004
lasers:
  - laser_id: 0
    rot_correction: 0.1
    vert_correction: -0.2
    dist_correction: 0.3
    dist_correction_x: 0.4
    dist_correction_y: 0.5
    vert_offset_correction: 0.6
    focal_distance: 0.7
    focal_slope: 0.8
  - laser_id: 1
    rot_correction: 0.2
    vert_correction: 0.1
    dist_correction: 0.4
    two_pt_correction_available: true
    dist_correction_x: 0.6
    dist_correction_y: 0.7
    vert_offset_correction: 0.8
    horiz_offset_correction: 0.9
    max_intensity: 123.9
    min_intensity: 4.2
    focal_distance: 1.1
    focal_slope: 1.2
)";
  write_text_file(calibration_file, calibration_yaml);

  const VelodyneCalibration calibration(calibration_file.string());

  ASSERT_TRUE(calibration.initialized);
  ASSERT_EQ(calibration.num_lasers, 2);
  ASSERT_EQ(calibration.laser_corrections.size(), 2U);
  ASSERT_EQ(calibration.laser_corrections_map.size(), 2U);
  EXPECT_FLOAT_EQ(calibration.distance_resolution_m, 0.004F);

  const auto & first_correction = calibration.laser_corrections[0];
  EXPECT_FLOAT_EQ(first_correction.rot_correction, 0.1F);
  EXPECT_FLOAT_EQ(first_correction.vert_correction, -0.2F);
  EXPECT_FLOAT_EQ(first_correction.dist_correction, 0.3F);
  EXPECT_FALSE(first_correction.two_pt_correction_available);
  EXPECT_FLOAT_EQ(first_correction.dist_correction_x, 0.4F);
  EXPECT_FLOAT_EQ(first_correction.dist_correction_y, 0.5F);
  EXPECT_FLOAT_EQ(first_correction.vert_offset_correction, 0.6F);
  EXPECT_FLOAT_EQ(first_correction.horiz_offset_correction, 0.0F);
  EXPECT_EQ(first_correction.max_intensity, 255);
  EXPECT_EQ(first_correction.min_intensity, 0);
  EXPECT_FLOAT_EQ(first_correction.focal_distance, 0.7F);
  EXPECT_FLOAT_EQ(first_correction.focal_slope, 0.8F);
  EXPECT_NEAR(first_correction.cos_rot_correction, std::cos(0.1F), 1e-6F);
  EXPECT_NEAR(first_correction.sin_rot_correction, std::sin(0.1F), 1e-6F);
  EXPECT_NEAR(first_correction.cos_vert_correction, std::cos(-0.2F), 1e-6F);
  EXPECT_NEAR(first_correction.sin_vert_correction, std::sin(-0.2F), 1e-6F);
  EXPECT_EQ(first_correction.laser_ring, 0);

  const auto & second_correction = calibration.laser_corrections[1];
  EXPECT_TRUE(second_correction.two_pt_correction_available);
  EXPECT_FLOAT_EQ(second_correction.horiz_offset_correction, 0.9F);
  EXPECT_EQ(second_correction.max_intensity, 123);
  EXPECT_EQ(second_correction.min_intensity, 4);
  EXPECT_EQ(second_correction.laser_ring, 1);

  EXPECT_FLOAT_EQ(calibration.laser_corrections_map.at(1).vert_correction, 0.1F);

  std::filesystem::remove(calibration_file);
}

TEST(VelodyneCommonTest, CalibrationConfigurationLoadsWritesAndReloadsCalibrationFiles)
{
  const auto input_file = make_temp_file_path("nebula_velodyne_calibration_config_input");
  const auto output_file = make_temp_file_path("nebula_velodyne_calibration_config_output");
  const auto calibration_yaml = R"(num_lasers: 2
distance_resolution: 0.004
lasers:
  - laser_id: 0
    rot_correction: 0.1
    vert_correction: -0.2
    dist_correction: 0.3
    dist_correction_x: 0.4
    dist_correction_y: 0.5
    vert_offset_correction: 0.6
    focal_distance: 0.7
    focal_slope: 0.8
  - laser_id: 1
    rot_correction: 0.2
    vert_correction: 0.1
    dist_correction: 0.4
    two_pt_correction_available: true
    dist_correction_x: 0.6
    dist_correction_y: 0.7
    vert_offset_correction: 0.8
    horiz_offset_correction: 0.9
    max_intensity: 123.9
    min_intensity: 4.2
    focal_distance: 1.1
    focal_slope: 1.2
)";
  write_text_file(input_file, calibration_yaml);
  std::filesystem::remove(output_file);

  VelodyneCalibrationConfiguration configuration{};
  ASSERT_EQ(configuration.load_from_file(input_file.string()), Status::OK);
  ASSERT_EQ(configuration.save_file(output_file.string()), Status::OK);

  const VelodyneCalibration reloaded(output_file.string());
  ASSERT_TRUE(reloaded.initialized);
  EXPECT_EQ(reloaded.num_lasers, 2);
  EXPECT_EQ(reloaded.laser_corrections_map.size(), 2U);
  EXPECT_FLOAT_EQ(reloaded.laser_corrections_map.at(0).dist_correction_x, 0.4F);
  EXPECT_TRUE(reloaded.laser_corrections_map.at(1).two_pt_correction_available);

  std::filesystem::remove(input_file);
  std::filesystem::remove(output_file);
}

TEST(VelodyneCommonTest, CalibrationConfigurationReportsInvalidCalibrationFiles)
{
  const auto missing_file = make_temp_file_path("nebula_velodyne_missing");

  VelodyneCalibrationConfiguration configuration{};
  EXPECT_EQ(configuration.load_from_file(missing_file.string()), Status::INVALID_CALIBRATION_FILE);
}

}  // namespace
