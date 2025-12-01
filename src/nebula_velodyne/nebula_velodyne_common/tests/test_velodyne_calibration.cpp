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

#include "nebula_velodyne_common/velodyne_calibration_decoder.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using nebula::drivers::VelodyneCalibration;
using nebula::drivers::VelodyneLaserCorrection;

#ifndef _CALIBRATION_DIR_PATH
#error "_CALIBRATION_DIR_PATH is not defined"
#endif

class VelodyneCalibrationTest : public ::testing::Test
{
protected:
  std::filesystem::path calibration_dir{_CALIBRATION_DIR_PATH};
};

// ============================================================================
// VelodyneLaserCorrection tests
// ============================================================================

TEST(VelodyneLaserCorrectionTest, DefaultValues)
{
  VelodyneLaserCorrection correction{};

  // Struct should be zero-initialized by default
  EXPECT_EQ(correction.laser_ring, 0);
  EXPECT_FALSE(correction.two_pt_correction_available);
}

// ============================================================================
// VelodyneCalibration basic tests
// ============================================================================

TEST(VelodyneCalibrationBasicTest, DefaultConstruction)
{
  VelodyneCalibration calib;

  EXPECT_FALSE(calib.initialized);
  EXPECT_FLOAT_EQ(calib.distance_resolution_m, 0.002f);
  EXPECT_EQ(calib.num_lasers, 0);
  EXPECT_TRUE(calib.laser_corrections.empty());
  EXPECT_TRUE(calib.laser_corrections_map.empty());
}

TEST(VelodyneCalibrationBasicTest, NonExistentFile)
{
  VelodyneCalibration calib("/nonexistent/path/to/calibration.yaml");

  EXPECT_FALSE(calib.initialized);
}

// ============================================================================
// Read existing calibration files
// ============================================================================

TEST_F(VelodyneCalibrationTest, ReadVLP16Calibration)
{
  auto calib_file = calibration_dir / "VLP16.yaml";
  ASSERT_TRUE(std::filesystem::exists(calib_file)) << "VLP16.yaml not found";

  VelodyneCalibration calib(calib_file.string());

  EXPECT_TRUE(calib.initialized);
  EXPECT_EQ(calib.num_lasers, 16);
  EXPECT_EQ(calib.laser_corrections.size(), 16);
}

TEST_F(VelodyneCalibrationTest, ReadVLP32Calibration)
{
  auto calib_file = calibration_dir / "VLP32.yaml";
  ASSERT_TRUE(std::filesystem::exists(calib_file)) << "VLP32.yaml not found";

  VelodyneCalibration calib(calib_file.string());

  EXPECT_TRUE(calib.initialized);
  EXPECT_EQ(calib.num_lasers, 32);
  EXPECT_EQ(calib.laser_corrections.size(), 32);
}

TEST_F(VelodyneCalibrationTest, ReadVLS128Calibration)
{
  auto calib_file = calibration_dir / "VLS128.yaml";
  ASSERT_TRUE(std::filesystem::exists(calib_file)) << "VLS128.yaml not found";

  VelodyneCalibration calib(calib_file.string());

  EXPECT_TRUE(calib.initialized);
  EXPECT_EQ(calib.num_lasers, 128);
  EXPECT_EQ(calib.laser_corrections.size(), 128);
}

// ============================================================================
// Calibration data validation
// ============================================================================

TEST_F(VelodyneCalibrationTest, LaserCorrectionsHaveCachedValues)
{
  auto calib_file = calibration_dir / "VLP16.yaml";
  ASSERT_TRUE(std::filesystem::exists(calib_file));

  VelodyneCalibration calib(calib_file.string());
  ASSERT_TRUE(calib.initialized);

  for (const auto & correction : calib.laser_corrections) {
    // Verify that cached sin/cos values match the angle values
    EXPECT_NEAR(correction.sin_rot_correction, std::sin(correction.rot_correction), 1e-6);
    EXPECT_NEAR(correction.cos_rot_correction, std::cos(correction.rot_correction), 1e-6);
    EXPECT_NEAR(correction.sin_vert_correction, std::sin(correction.vert_correction), 1e-6);
    EXPECT_NEAR(correction.cos_vert_correction, std::cos(correction.vert_correction), 1e-6);
  }
}

TEST_F(VelodyneCalibrationTest, LaserRingsAreAssigned)
{
  auto calib_file = calibration_dir / "VLP16.yaml";
  ASSERT_TRUE(std::filesystem::exists(calib_file));

  VelodyneCalibration calib(calib_file.string());
  ASSERT_TRUE(calib.initialized);

  // Verify that laser rings are assigned (0 to num_lasers-1)
  std::vector<bool> ring_used(calib.num_lasers, false);

  for (const auto & correction : calib.laser_corrections) {
    EXPECT_GE(correction.laser_ring, 0);
    EXPECT_LT(correction.laser_ring, calib.num_lasers);
    ring_used[correction.laser_ring] = true;
  }

  // All rings should be used exactly once
  for (int i = 0; i < calib.num_lasers; ++i) {
    EXPECT_TRUE(ring_used[i]) << "Ring " << i << " not used";
  }
}

// ============================================================================
// Write and read roundtrip
// ============================================================================

TEST_F(VelodyneCalibrationTest, WriteReadRoundtrip)
{
  auto calib_file = calibration_dir / "VLP16.yaml";
  ASSERT_TRUE(std::filesystem::exists(calib_file));

  // Read original
  VelodyneCalibration original(calib_file.string());
  ASSERT_TRUE(original.initialized);

  // Write to temp file
  auto temp_file = std::filesystem::temp_directory_path() / "test_velodyne_calib.yaml";
  original.write(temp_file.string());

  ASSERT_TRUE(std::filesystem::exists(temp_file));

  // Read back
  VelodyneCalibration loaded(temp_file.string());
  EXPECT_TRUE(loaded.initialized);

  // Compare
  EXPECT_EQ(loaded.num_lasers, original.num_lasers);
  EXPECT_FLOAT_EQ(loaded.distance_resolution_m, original.distance_resolution_m);
  EXPECT_EQ(loaded.laser_corrections.size(), original.laser_corrections.size());

  for (size_t i = 0; i < original.laser_corrections.size(); ++i) {
    const auto & orig = original.laser_corrections[i];
    const auto & load = loaded.laser_corrections[i];

    EXPECT_FLOAT_EQ(load.rot_correction, orig.rot_correction);
    EXPECT_FLOAT_EQ(load.vert_correction, orig.vert_correction);
    EXPECT_FLOAT_EQ(load.dist_correction, orig.dist_correction);
    EXPECT_EQ(load.two_pt_correction_available, orig.two_pt_correction_available);
    EXPECT_FLOAT_EQ(load.dist_correction_x, orig.dist_correction_x);
    EXPECT_FLOAT_EQ(load.dist_correction_y, orig.dist_correction_y);
    EXPECT_FLOAT_EQ(load.vert_offset_correction, orig.vert_offset_correction);
    EXPECT_FLOAT_EQ(load.horiz_offset_correction, orig.horiz_offset_correction);
    EXPECT_EQ(load.max_intensity, orig.max_intensity);
    EXPECT_EQ(load.min_intensity, orig.min_intensity);
    EXPECT_FLOAT_EQ(load.focal_distance, orig.focal_distance);
    EXPECT_FLOAT_EQ(load.focal_slope, orig.focal_slope);
  }

  // Cleanup
  std::filesystem::remove(temp_file);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
