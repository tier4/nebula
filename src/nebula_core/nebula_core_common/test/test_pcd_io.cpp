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

#include "nebula_core_common/io/pcd.hpp"
#include "nebula_core_common/point_types.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#ifndef NEBULA_CORE_COMMON_TEST_RESOURCES_DIR
#error "NEBULA_CORE_COMMON_TEST_RESOURCES_DIR must be defined"
#endif

namespace
{

namespace fs = std::filesystem;
using nebula::drivers::PointCloud;
using nebula::drivers::PointXYZ;
using nebula::drivers::PointXYZIR;
using nebula::drivers::io::PcdReader;
using nebula::drivers::io::PcdWriter;

class PcdIoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    temp_dir_ =
      fs::temp_directory_path() / fs::path("nebula_pcd_test_" + std::to_string(::getpid()));
    fs::create_directories(temp_dir_);
  }

  void TearDown() override { fs::remove_all(temp_dir_); }

  static void write_text_file(const fs::path & path, const std::string & content)
  {
    std::ofstream file(path, std::ios::binary);
    ASSERT_TRUE(file.is_open());
    file << content;
    ASSERT_TRUE(file.good());
  }

  static fs::path fixture_path(const fs::path & relative_path)
  {
    return fs::path(NEBULA_CORE_COMMON_TEST_RESOURCES_DIR) / relative_path;
  }

  fs::path temp_dir_;
};

TEST_F(PcdIoTest, ReadHandCraftedAsciiPointXYZ)
{
  const auto path = fixture_path("pcd/ascii_xyz.pcd");
  ASSERT_TRUE(fs::exists(path));

  const auto cloud = PcdReader::read<PointXYZ>(path.string());
  ASSERT_EQ(cloud.size(), 2U);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.5F);
  EXPECT_FLOAT_EQ(cloud[0].y, -2.25F);
  EXPECT_FLOAT_EQ(cloud[0].z, 3.0F);
  EXPECT_FLOAT_EQ(cloud[1].x, -4.5F);
  EXPECT_FLOAT_EQ(cloud[1].y, 5.125F);
  EXPECT_FLOAT_EQ(cloud[1].z, 6.75F);
}

TEST_F(PcdIoTest, ReadHandCraftedBinaryPointXYZ)
{
  const auto path = fixture_path("pcd/binary_xyz_pcl.pcd");
  ASSERT_TRUE(fs::exists(path));

  const auto cloud = PcdReader::read<PointXYZ>(path.string());
  ASSERT_EQ(cloud.size(), 2U);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.25F);
  EXPECT_FLOAT_EQ(cloud[0].y, 2.5F);
  EXPECT_FLOAT_EQ(cloud[0].z, -3.75F);
  EXPECT_FLOAT_EQ(cloud[1].x, -10.0F);
  EXPECT_FLOAT_EQ(cloud[1].y, 20.0F);
  EXPECT_FLOAT_EQ(cloud[1].z, 30.0F);
}

TEST_F(PcdIoTest, RoundTripBinaryPointXYZ)
{
  PointCloud<PointXYZ> input_cloud;
  input_cloud.push_back(PointXYZ{1.0F, 2.0F, 3.0F});
  input_cloud.push_back(PointXYZ{-4.5F, 5.5F, -6.5F});
  input_cloud.push_back(PointXYZ{0.0F, -0.125F, 42.25F});

  const auto path = temp_dir_ / "round_trip_xyz.pcd";
  PcdWriter::write_binary(path.string(), input_cloud);

  const auto loaded_cloud = PcdReader::read<PointXYZ>(path.string());
  ASSERT_EQ(loaded_cloud.size(), input_cloud.size());
  for (size_t i = 0; i < input_cloud.size(); ++i) {
    EXPECT_FLOAT_EQ(loaded_cloud[i].x, input_cloud[i].x);
    EXPECT_FLOAT_EQ(loaded_cloud[i].y, input_cloud[i].y);
    EXPECT_FLOAT_EQ(loaded_cloud[i].z, input_cloud[i].z);
  }
}

TEST_F(PcdIoTest, RoundTripBinaryPointXYZIRWithPadding)
{
  PointCloud<PointXYZIR> input_cloud;
  PointXYZIR p0{};
  p0.x = 1.0F;
  p0.y = 2.0F;
  p0.z = 3.0F;
  p0.intensity = 10.5F;
  p0.ring = 3U;
  input_cloud.push_back(p0);

  PointXYZIR p1{};
  p1.x = -4.5F;
  p1.y = 5.5F;
  p1.z = -6.5F;
  p1.intensity = 42.0F;
  p1.ring = 511U;
  input_cloud.push_back(p1);

  const auto path = temp_dir_ / "round_trip_xyzir.pcd";
  PcdWriter::write_binary(path.string(), input_cloud);

  const auto loaded_cloud = PcdReader::read<PointXYZIR>(path.string());
  ASSERT_EQ(loaded_cloud.size(), input_cloud.size());
  for (size_t i = 0; i < input_cloud.size(); ++i) {
    EXPECT_FLOAT_EQ(loaded_cloud[i].x, input_cloud[i].x);
    EXPECT_FLOAT_EQ(loaded_cloud[i].y, input_cloud[i].y);
    EXPECT_FLOAT_EQ(loaded_cloud[i].z, input_cloud[i].z);
    EXPECT_FLOAT_EQ(loaded_cloud[i].intensity, input_cloud[i].intensity);
    EXPECT_EQ(loaded_cloud[i].ring, input_cloud[i].ring);
  }
}

TEST_F(PcdIoTest, BinaryCompressedFormatThrows)
{
  const auto path = temp_dir_ / "compressed.pcd";
  write_text_file(
    path,
    "# .PCD v0.7 - Point Cloud Data file format\n"
    "VERSION 0.7\n"
    "FIELDS x y z\n"
    "SIZE 4 4 4\n"
    "TYPE F F F\n"
    "COUNT 1 1 1\n"
    "WIDTH 1\n"
    "HEIGHT 1\n"
    "POINTS 1\n"
    "DATA binary_compressed\n");

  EXPECT_THROW(static_cast<void>(PcdReader::read<PointXYZ>(path.string())), std::runtime_error);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
