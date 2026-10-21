// Copyright 2024 TIER IV, Inc.

#include "velodyne_ros_decoder_test_vlp32.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

std::shared_ptr<nebula::ros::VelodyneRosDecoderTest> velodyne_driver;

TEST(TestDecoder, TestPcd)
{
  std::cout << "TEST(TestDecoder, TestPcd)" << std::endl;
  velodyne_driver->read_bag();
}

namespace
{

constexpr size_t g_velodyne_packet_size = 1206;
constexpr size_t g_block_size = 100;
constexpr size_t g_last_block_azimuth_offset = 1102;

/// @brief Build a well-formed single-return packet whose 12 blocks all carry the given azimuth
/// and plausible returns, so that only the size (or azimuth) under test is wrong.
std::vector<uint8_t> make_packet(uint16_t azimuth_centideg)
{
  std::vector<uint8_t> packet(g_velodyne_packet_size, 0);
  for (size_t block = 0; block < 12; ++block) {
    uint8_t * b = packet.data() + block * g_block_size;
    b[0] = 0xFF;  // upper bank flag 0xEEFF (little endian)
    b[1] = 0xEE;
    b[2] = azimuth_centideg & 0xFF;
    b[3] = azimuth_centideg >> 8;
    for (size_t ch = 0; ch < 32; ++ch) {
      b[4 + ch * 3 + 0] =
        0x00;  // distance = 0x0800 raw (4.1 m for 2 mm, 8.2 m for 4 mm resolution)
      b[4 + ch * 3 + 1] = 0x08;
      b[4 + ch * 3 + 2] = 100;  // intensity
    }
  }
  packet[1204] = 55;    // return mode: strongest
  packet[1205] = 0x22;  // product id
  return packet;
}

}  // namespace

TEST(TestDecoder, TestValidPacketStreamProducesPointcloud)
{
  // Sanity check for the tests below: a stream of well-formed packets completes a scan.
  bool got_pointcloud = false;
  for (const uint16_t azimuth : {0, 9000, 18000, 27000, 0, 9000, 18000, 27000, 0}) {
    const auto pointcloud =
      std::get<0>(velodyne_driver->parse_cloud_packet(make_packet(azimuth), 0.0));
    got_pointcloud |= (pointcloud != nullptr);
  }
  EXPECT_TRUE(got_pointcloud);
}

TEST(TestDecoder, TestWrongSizedPacketIsRejected)
{
  // Velodyne data packets are exactly 1206 bytes. Anything else must be dropped before any
  // fixed-offset access happens (see CVE-2026-74238). The packets are truncated or padded copies
  // of a well-formed packet, so every byte that is present looks valid.
  const auto full = make_packet(18000);
  for (const size_t size : {0u,    1u,    2u,    3u,    4u,    8u,    64u,   100u,  1101u, 1102u,
                            1103u, 1104u, 1200u, 1203u, 1204u, 1205u, 1207u, 1208u, 1500u, 9000u}) {
    std::vector<uint8_t> packet(full);
    packet.resize(size, 0xAB);
    // Alternate azimuths so that the scan-cutting logic would trigger if the packet were decoded
    if (size > g_last_block_azimuth_offset + 1) {
      packet[2] = 0;
      packet[3] = 0;
    }
    const auto pointcloud = std::get<0>(velodyne_driver->parse_cloud_packet(packet, 0.0));
    EXPECT_EQ(pointcloud, nullptr) << "packet of " << size << " bytes was not rejected";
  }
}

TEST(TestDecoder, TestOutOfRangeAzimuthDoesNotCrash)
{
  // The wire azimuth is a free 16-bit value. Values >= 36000 must not index past the
  // 36000-entry azimuth lookup table.
  for (const uint16_t azimuth : {35999, 36000, 36001, 40000, 65535, 0, 65535}) {
    EXPECT_NO_THROW(velodyne_driver->parse_cloud_packet(make_packet(azimuth), 0.0));
  }
}

TEST(TestDecoder, TestScanOfOnlyOverflowPointsDoesNotCrash)
{
  // A scan cut whose points all lie within a quarter turn after the scan phase moves every point
  // to the overflow buffer. That must not read past the beginning of the emptied scan buffer.
  for (const uint16_t azimuth : {1000, 2000, 0, 1000, 2000, 0, 1000}) {
    EXPECT_NO_THROW(velodyne_driver->parse_cloud_packet(make_packet(azimuth), 0.0));
  }
}

int main(int argc, char * argv[])
{
  std::cout << "velodyne_ros_decoder_test_main_vlp32.cpp" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_velodyne_decoder_test";

  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  velodyne_driver = std::make_shared<nebula::ros::VelodyneRosDecoderTest>(options, node_name);
  exec.add_node(velodyne_driver->get_node_base_interface());

  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Get Status");
  nebula::Status driver_status = velodyne_driver->get_status();
  int result = 0;
  if (driver_status == nebula::Status::OK) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Reading Started");
    result = RUN_ALL_TESTS();
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(node_name), driver_status);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Ending");
  velodyne_driver.reset();
  rclcpp::shutdown();

  return result;
}
