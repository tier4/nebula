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

#include "nebula_hesai_decoders/decoders/hesai_ftx140_decoder.hpp"

#include <nebula_hesai_common/hesai_common.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#ifndef _SRC_CALIBRATION_DIR_PATH
static_assert(false, "_SRC_CALIBRATION_DIR_PATH is not defined");
#endif

using nebula::Status;
using nebula::drivers::DecodeError;
using nebula::drivers::HesaiFtx140Decoder;
using nebula::drivers::HesaiFtxCalibration;
using nebula::drivers::HesaiSensorConfiguration;
using nebula::drivers::NebulaPointCloud;
using nebula::drivers::NebulaPointCloudPtr;
using nebula::drivers::PacketLossDetector;
using nebula::drivers::hesai_packet::PacketFtx;

namespace
{

/// @brief The sample correction file of the Hesai LiDAR SDK 2.0, installed as the default
const char g_calibration_path[] = _SRC_CALIBRATION_DIR_PATH "FTX140.dat";

class NullLogger : public nebula::drivers::loggers::Logger
{
public:
  void debug(const std::string &) override {}
  void info(const std::string &) override {}
  void warn(const std::string &) override {}
  void error(const std::string &) override {}
  std::shared_ptr<Logger> child(const std::string &) override
  {
    return std::make_shared<NullLogger>();
  }
};

struct PacketParams
{
  uint8_t frame_id{0};
  uint8_t row_id{0};
  uint8_t column_id{0};
  uint32_t udp_sequence{0};
  uint32_t timestamp_us{0};
  uint8_t configuration_number{0};
  bool zoom{false};
};

std::vector<uint8_t> make_packet(const PacketParams & params)
{
  PacketFtx packet{};
  packet.pre_header = {0xFFEE, 7, 3, 1, 0};
  packet.header.total_columns = params.zoom ? 160 : 256;
  packet.header.total_rows = params.zoom ? 96 : 192;
  packet.header.dis_unit = 4;
  packet.header.channel_num = PacketFtx::n_channels;
  packet.header.configuration_number = params.configuration_number;
  for (size_t i = 0; i < PacketFtx::n_channels; ++i) {
    packet.units[i].distance = static_cast<uint16_t>(1000 + i);
    packet.units[i].reflectivity = static_cast<uint8_t>(i);
  }
  packet.tail.column_id = params.column_id;
  packet.tail.row_id = params.row_id;
  packet.tail.frame_id = params.frame_id;
  packet.tail.return_mode = nebula::drivers::hesai_packet::return_mode::SINGLE_FIRST;
  uint64_t seconds = 1'700'000'000;
  for (int b = 4; b >= 0; --b) {
    packet.tail.date_time.seconds[b] = seconds & 0xFF;
    seconds >>= 8;
  }
  packet.tail.timestamp = params.timestamp_us;
  packet.tail.udp_sequence = params.udp_sequence;

  std::vector<uint8_t> bytes(sizeof(packet));
  std::memcpy(bytes.data(), &packet, sizeof(packet));
  return bytes;
}

std::shared_ptr<HesaiSensorConfiguration> make_config()
{
  auto config = std::make_shared<HesaiSensorConfiguration>();
  config->sensor_model = nebula::drivers::SensorModel::HESAI_FTX140;
  config->min_range = 0.1;
  config->max_range = 100;
  config->cloud_min_angle = 0;
  config->cloud_max_angle = 360;
  return config;
}

class TestHesaiFtx140 : public ::testing::Test
{
protected:
  void SetUp() override
  {
    calibration_ = std::make_shared<HesaiFtxCalibration>();
    ASSERT_EQ(calibration_->load_from_file(g_calibration_path), Status::OK);
  }

  /// @brief Feed one frame (packet per subzone) of a fully populated frame, then the first packet
  /// of the next frame, and return the completed frame
  NebulaPointCloud decode_one_frame(
    const std::shared_ptr<HesaiSensorConfiguration> & config, bool zoom = false,
    uint8_t configuration_number = 0)
  {
    HesaiFtx140Decoder decoder(config, calibration_, std::make_shared<NullLogger>(), nullptr);
    NebulaPointCloud cloud;
    decoder.set_pointcloud_callback(
      [&](const NebulaPointCloudPtr & pointcloud, double) { cloud = *pointcloud; });
    const uint8_t n_subzone_rows = zoom ? 16 : 32;
    const uint8_t n_subzone_columns = zoom ? 10 : 16;
    for (uint8_t row = 0; row < n_subzone_rows; ++row) {
      for (uint8_t column = 0; column < n_subzone_columns; ++column) {
        decoder.unpack(make_packet({1, row, column, 0, 0, configuration_number, zoom}));
      }
    }
    decoder.unpack(make_packet({2, 0, 0, 0, 100'000, configuration_number, zoom}));
    return cloud;
  }

  std::shared_ptr<HesaiFtxCalibration> calibration_;
};

}  // namespace

TEST_F(TestHesaiFtx140, CalibrationMatchesSdk)
{
  EXPECT_EQ(calibration_->rows, 192);
  EXPECT_EQ(calibration_->columns, 256);

  // Reference directions computed with the Hesai SDK's generate_pixel_vectors()
  struct Reference
  {
    size_t row;
    size_t column;
    std::array<float, 3> direction;
  };
  const std::vector<Reference> references{
    {0, 0, {-0.003009f, 0.782732f, -0.622351f}},     {96, 128, {0.998468f, -0.053784f, -0.012964f}},
    {191, 255, {-0.077906f, -0.814262f, 0.575247f}}, {10, 200, {0.416265f, -0.603778f, -0.679835f}},
    {150, 30, {0.510356f, 0.747211f, 0.425690f}},
  };
  for (const auto & reference : references) {
    const auto & direction = calibration_->pixel_direction(reference.row, reference.column);
    for (size_t axis = 0; axis < 3; ++axis) {
      EXPECT_NEAR(direction[axis], reference.direction[axis], 1e-5)
        << "pixel (" << reference.row << ", " << reference.column << "), axis " << axis;
    }
  }
}

TEST_F(TestHesaiFtx140, CalibrationRejectsInvalidData)
{
  HesaiFtxCalibration calibration;
  EXPECT_EQ(
    calibration.load_from_bytes(std::vector<uint8_t>(10, 0)), Status::INVALID_CALIBRATION_FILE);

  std::vector<uint8_t> wrong_version(HesaiFtxCalibration::correction_size, 0);
  wrong_version[0] = 0xEE;
  wrong_version[1] = 0xFF;
  wrong_version[2] = 0x04;
  wrong_version[3] = 0x03;
  EXPECT_EQ(calibration.load_from_bytes(wrong_version), Status::INVALID_CALIBRATION_FILE);
}

TEST_F(TestHesaiFtx140, DecodesFullFrame)
{
  auto config = make_config();
  uint64_t n_lost = 0;
  auto packet_loss_detector = std::make_shared<PacketLossDetector<PacketFtx>>();
  packet_loss_detector->set_lost_callback([&](uint64_t n) { n_lost += n; });

  HesaiFtx140Decoder decoder(
    config, calibration_, std::make_shared<NullLogger>(), packet_loss_detector);
  int n_clouds = 0;
  NebulaPointCloud cloud;
  double cloud_timestamp_s = 0;
  decoder.set_pointcloud_callback([&](const NebulaPointCloudPtr & pointcloud, double timestamp_s) {
    ++n_clouds;
    cloud = *pointcloud;
    cloud_timestamp_s = timestamp_s;
  });

  uint32_t sequence = 100;
  for (uint8_t row = 0; row < 32; ++row) {
    for (uint8_t column = 0; column < 16; ++column) {
      auto result = decoder.unpack(
        make_packet({5, row, column, sequence++, static_cast<uint32_t>(1000 + row * 16 + column)}));
      ASSERT_TRUE(result.metadata_or_error.has_value());
      EXPECT_FALSE(result.metadata_or_error.value().did_scan_complete);
    }
  }
  EXPECT_EQ(n_clouds, 0);

  sequence += 3;  // lose three packets
  auto result = decoder.unpack(make_packet({6, 0, 0, sequence, 50'000}));
  ASSERT_TRUE(result.metadata_or_error.has_value());
  EXPECT_TRUE(result.metadata_or_error.value().did_scan_complete);

  EXPECT_EQ(n_clouds, 1);
  EXPECT_EQ(n_lost, 3U);
  EXPECT_EQ(cloud.size(), 192U * 256U);
  EXPECT_NEAR(cloud_timestamp_s, 1'700'000'000.001, 1e-6);

  // The last packet of the frame came 511 us after the first one
  uint32_t max_point_time = 0;
  for (const auto & point : cloud) max_point_time = std::max(max_point_time, point.time_stamp);
  EXPECT_EQ(max_point_time, 511'000U);

  // Unit 7 of subzone (row 2, column 3) is pixel (2 * 6 + 7 % 6, 3 * 16 + 7 / 6) = (13, 49)
  const float expected_distance = (1000 + 7) * 0.004f;
  const auto & direction = calibration_->pixel_direction(13, 49);
  auto it = std::find_if(cloud.begin(), cloud.end(), [&](const auto & point) {
    return point.channel == 13 && std::abs(point.distance - expected_distance) < 1e-4 &&
           std::abs(point.x - expected_distance * direction[0]) < 1e-3 &&
           std::abs(point.y - expected_distance * direction[1]) < 1e-3 &&
           std::abs(point.z - expected_distance * direction[2]) < 1e-3;
  });
  EXPECT_NE(it, cloud.end());
}

TEST_F(TestHesaiFtx140, IgnoresImuAndFaultMessagePackets)
{
  HesaiFtx140Decoder decoder(make_config(), calibration_, std::make_shared<NullLogger>(), nullptr);

  std::vector<uint8_t> imu_packet(34, 0);
  imu_packet[0] = 0xEE;
  imu_packet[1] = 0xFF;
  imu_packet[2] = 7;
  imu_packet[3] = 3;
  imu_packet[5] = 1;
  auto imu_result = decoder.unpack(imu_packet);
  ASSERT_FALSE(imu_result.metadata_or_error.has_value());
  EXPECT_EQ(imu_result.metadata_or_error.error(), DecodeError::PACKET_IGNORED);

  std::vector<uint8_t> fault_message(99, 0);
  fault_message[0] = 1;
  auto fault_result = decoder.unpack(fault_message);
  ASSERT_FALSE(fault_result.metadata_or_error.has_value());
  EXPECT_EQ(fault_result.metadata_or_error.error(), DecodeError::PACKET_IGNORED);
}

TEST_F(TestHesaiFtx140, FlipsRowsForBottomUpConfiguration)
{
  HesaiFtx140Decoder decoder(make_config(), calibration_, std::make_shared<NullLogger>(), nullptr);
  NebulaPointCloud cloud;
  decoder.set_pointcloud_callback(
    [&](const NebulaPointCloudPtr & pointcloud, double) { cloud = *pointcloud; });
  decoder.unpack(make_packet({1, 0, 0, 0, 0, 1}));
  decoder.unpack(make_packet({2, 0, 0, 0, 100'000, 1}));

  ASSERT_EQ(cloud.size(), PacketFtx::n_channels);
  for (const auto & point : cloud) {
    EXPECT_GE(point.channel, 186);
  }
}

TEST_F(TestHesaiFtx140, MapsZoomModeToCentralPixels)
{
  auto cloud = decode_one_frame(make_config(), true);
  ASSERT_EQ(cloud.size(), 96U * 160U);
  auto [min_it, max_it] = std::minmax_element(
    cloud.begin(), cloud.end(),
    [](const auto & a, const auto & b) { return a.channel < b.channel; });
  EXPECT_EQ(min_it->channel, 48);
  EXPECT_EQ(max_it->channel, 48 + 96 - 1);
}

TEST_F(TestHesaiFtx140, CropsToFieldOfView)
{
  auto config = make_config();
  config->cloud_min_angle = 80;
  config->cloud_max_angle = 100;
  auto cloud = decode_one_frame(config);

  ASSERT_GT(cloud.size(), 0U);
  EXPECT_LT(cloud.size(), 192U * 256U);
  for (const auto & point : cloud) {
    EXPECT_GE(point.azimuth, nebula::drivers::deg2rad(80.f) - 1e-5);
    EXPECT_LE(point.azimuth, nebula::drivers::deg2rad(100.f) + 1e-5);
  }
}
