// Copyright 2026 TIER IV, Inc.

#include "nebula_ouster_common/ouster_calibration_data.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

namespace nebula::drivers
{

class TestOusterCalibrationData : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Generate a valid metadata JSON from the SDK's default sensor info
    auto info = ouster::sdk::core::default_sensor_info(ouster::sdk::core::LidarMode::_512x10);
    valid_metadata_json_ = info.to_json_string();
  }

  std::string valid_metadata_json_;
};

TEST_F(TestOusterCalibrationData, TestLoadFromStringSuccess)
{
  auto result = OusterCalibrationData::load_from_string(valid_metadata_json_);
  ASSERT_TRUE(result.has_value());

  auto calibration = result.value();
  EXPECT_FALSE(calibration.metadata_json.empty());
  EXPECT_NE(calibration.sensor_info, nullptr);
  EXPECT_NE(calibration.packet_format, nullptr);
}

TEST_F(TestOusterCalibrationData, TestLoadFromStringEmpty)
{
  auto result = OusterCalibrationData::load_from_string("");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OusterCalibrationData::ErrorCode::EMPTY_METADATA);
}

TEST_F(TestOusterCalibrationData, TestLoadFromFileSuccess)
{
  // Write valid metadata to a temporary file
  std::string tmp_path = "/tmp/test_ouster_metadata.json";
  {
    std::ofstream f(tmp_path);
    f << valid_metadata_json_;
  }

  auto result = OusterCalibrationData::load_from_file(tmp_path);
  ASSERT_TRUE(result.has_value());

  auto calibration = result.value();
  EXPECT_EQ(calibration.calibration_file, tmp_path);
  EXPECT_FALSE(calibration.metadata_json.empty());
  EXPECT_NE(calibration.sensor_info, nullptr);
  EXPECT_NE(calibration.packet_format, nullptr);

  std::remove(tmp_path.c_str());
}

TEST_F(TestOusterCalibrationData, TestLoadFromFileNotFound)
{
  auto result = OusterCalibrationData::load_from_file("/nonexistent/path/metadata.json");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OusterCalibrationData::ErrorCode::OPEN_FOR_READ_FAILED);
}

TEST_F(TestOusterCalibrationData, TestLoadFromFileEmpty)
{
  std::string tmp_path = "/tmp/test_ouster_empty_metadata.json";
  {
    std::ofstream f(tmp_path);
    // Write empty file
  }

  auto result = OusterCalibrationData::load_from_file(tmp_path);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OusterCalibrationData::ErrorCode::EMPTY_METADATA);

  std::remove(tmp_path.c_str());
}

TEST_F(TestOusterCalibrationData, TestSaveToFileSuccess)
{
  auto load_result = OusterCalibrationData::load_from_string(valid_metadata_json_);
  ASSERT_TRUE(load_result.has_value());

  auto calibration = load_result.value();
  std::string tmp_path = "/tmp/test_ouster_save_metadata.json";
  auto save_result = calibration.save_to_file(tmp_path);
  ASSERT_TRUE(save_result.has_value());

  // Verify file content
  std::ifstream f(tmp_path);
  std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  EXPECT_EQ(content, valid_metadata_json_);

  std::remove(tmp_path.c_str());
}

TEST_F(TestOusterCalibrationData, TestSaveToFileInvalidPath)
{
  auto load_result = OusterCalibrationData::load_from_string(valid_metadata_json_);
  ASSERT_TRUE(load_result.has_value());

  auto calibration = load_result.value();
  auto save_result = calibration.save_to_file("/nonexistent/directory/metadata.json");
  ASSERT_FALSE(save_result.has_value());
  EXPECT_EQ(save_result.error().code, OusterCalibrationData::ErrorCode::OPEN_FOR_WRITE_FAILED);
}

}  // namespace nebula::drivers

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
