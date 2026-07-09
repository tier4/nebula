// Copyright 2026 TIER IV, Inc.

#include "nebula_ouster_decoders/ouster_decoder.hpp"

#include <nebula_ouster_common/ouster_calibration_data.hpp>

#include <gtest/gtest.h>
#include <ouster/types.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{

class TestOusterDecoder : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Use the SDK's default_sensor_info to get a valid SensorInfo for testing
    auto info = ouster::sdk::core::default_sensor_info(ouster::sdk::core::LidarMode::_512x10);
    auto sensor_info = std::make_shared<ouster::sdk::core::SensorInfo>(info);
    auto packet_format = std::make_shared<ouster::sdk::core::PacketFormat>(*sensor_info);

    calibration_.sensor_info = sensor_info;
    calibration_.packet_format = packet_format;
    calibration_.metadata_json = "{}";  // not used by decoder directly
  }

  OusterCalibrationData calibration_;
};

TEST_F(TestOusterDecoder, TestEmptyPacketReturnsError)
{
  auto decoder = OusterDecoder(
    FieldOfView<float, Degrees>{}, calibration_, [](const NebulaPointCloudPtr &, double) {});

  std::vector<uint8_t> empty_packet;
  auto result = decoder.unpack(empty_packet);
  ASSERT_FALSE(result.metadata_or_error.has_value());
  EXPECT_EQ(result.metadata_or_error.error(), DecodeError::EMPTY_PACKET);
}

TEST_F(TestOusterDecoder, TestCallbackNotSetReturnsError)
{
  auto decoder = OusterDecoder(FieldOfView<float, Degrees>{}, calibration_, nullptr);

  std::vector<uint8_t> packet(100, 0x00);
  auto result = decoder.unpack(packet);
  ASSERT_FALSE(result.metadata_or_error.has_value());
  EXPECT_EQ(result.metadata_or_error.error(), DecodeError::CALLBACK_NOT_SET);
}

TEST_F(TestOusterDecoder, TestInvalidPacketSizeReturnsError)
{
  auto decoder = OusterDecoder(
    FieldOfView<float, Degrees>{}, calibration_, [](const NebulaPointCloudPtr &, double) {});

  // A packet whose size does not match lidar, imu, or zone packet size
  std::vector<uint8_t> bad_packet(42, 0xAA);
  auto result = decoder.unpack(bad_packet);
  ASSERT_FALSE(result.metadata_or_error.has_value());
  EXPECT_EQ(result.metadata_or_error.error(), DecodeError::PACKET_FORMAT_INVALID);
}

TEST_F(TestOusterDecoder, TestValidLidarPacketDoesNotError)
{
  std::atomic_int callback_count{0};
  auto decoder = OusterDecoder(
    FieldOfView<float, Degrees>{}, calibration_,
    [&](const NebulaPointCloudPtr &, double) { callback_count++; });

  // Create a packet with the correct lidar packet size (all zeros is fine for format check)
  const size_t lidar_size = decoder.lidar_packet_size();
  std::vector<uint8_t> lidar_packet(lidar_size, 0x00);
  auto result = decoder.unpack(lidar_packet);

  // Should succeed (metadata returned)
  ASSERT_TRUE(result.metadata_or_error.has_value());
  // Single packet typically does not complete a full scan
  EXPECT_FALSE(result.metadata_or_error.value().did_scan_complete);
  EXPECT_GT(result.performance_counters.decode_time_ns, 0U);
}

TEST_F(TestOusterDecoder, TestSetPointcloudCallback)
{
  std::atomic_int callback_count{0};
  auto decoder = OusterDecoder(FieldOfView<float, Degrees>{}, calibration_, nullptr);

  // Should fail with callback not set
  std::vector<uint8_t> packet(42, 0x00);
  auto result = decoder.unpack(packet);
  EXPECT_EQ(result.metadata_or_error.error(), DecodeError::CALLBACK_NOT_SET);

  // Now set a valid callback
  decoder.set_pointcloud_callback([&](const NebulaPointCloudPtr &, double) { callback_count++; });

  // Still invalid size, but we should get FORMAT_INVALID, not CALLBACK_NOT_SET
  result = decoder.unpack(packet);
  EXPECT_EQ(result.metadata_or_error.error(), DecodeError::PACKET_FORMAT_INVALID);
}

TEST_F(TestOusterDecoder, TestLidarPacketSizeIsNonZero)
{
  auto decoder = OusterDecoder(
    FieldOfView<float, Degrees>{}, calibration_, [](const NebulaPointCloudPtr &, double) {});

  EXPECT_GT(decoder.lidar_packet_size(), 0U);
}

TEST_F(TestOusterDecoder, TestDecodeErrorToCstr)
{
  EXPECT_STREQ(to_cstr(DecodeError::PACKET_FORMAT_INVALID), "packet format invalid");
  EXPECT_STREQ(to_cstr(DecodeError::CALLBACK_NOT_SET), "pointcloud callback is not set");
  EXPECT_STREQ(to_cstr(DecodeError::EMPTY_PACKET), "packet is empty");
}

TEST_F(TestOusterDecoder, TestPerformanceCountersRecorded)
{
  auto decoder = OusterDecoder(
    FieldOfView<float, Degrees>{}, calibration_, [](const NebulaPointCloudPtr &, double) {});

  // Even for error paths, decode_time_ns should be recorded
  std::vector<uint8_t> empty_packet;
  auto result = decoder.unpack(empty_packet);
  EXPECT_GT(result.performance_counters.decode_time_ns, 0U);
  EXPECT_EQ(result.performance_counters.callback_time_ns, 0U);
}

TEST_F(TestOusterDecoder, TestMoveConstruction)
{
  auto decoder = OusterDecoder(
    FieldOfView<float, Degrees>{}, calibration_, [](const NebulaPointCloudPtr &, double) {});

  size_t original_size = decoder.lidar_packet_size();

  // Move construction
  auto moved_decoder = std::move(decoder);
  EXPECT_EQ(moved_decoder.lidar_packet_size(), original_size);
}

}  // namespace nebula::drivers

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
