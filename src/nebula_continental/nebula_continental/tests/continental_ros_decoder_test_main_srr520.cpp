// Copyright 2024 TIER IV, Inc.
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

#include "continental_ros_decoder_test_srr520.hpp"

#include <nebula_continental_common/continental_srr520.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

std::shared_ptr<nebula::ros::ContinentalRosDecoderTest> continental_driver;

TEST(TestDecoder, TestBag)
{
  std::cout << "TEST(TestDecoder, TestBag)" << std::endl;
  continental_driver->read_bag();
}

namespace
{

using namespace nebula::drivers::continental_srr520;  // NOLINT(build/namespaces)

/// @brief Build a packet as the SRR520 HW interface does: 4-byte big-endian CAN message ID
/// followed by the CAN payload.
std::vector<uint8_t> make_packet(uint32_t can_message_id, size_t payload_size)
{
  std::vector<uint8_t> packet(4 + payload_size, 0);
  packet[0] = can_message_id >> 24;
  packet[1] = (can_message_id >> 16) & 0xFF;
  packet[2] = (can_message_id >> 8) & 0xFF;
  packet[3] = can_message_id & 0xFF;
  return packet;
}

}  // namespace

TEST(TestDecoder, TestPacketShorterThanCanIdIsRejected)
{
  for (const size_t size : {0u, 1u, 2u, 3u}) {
    std::vector<uint8_t> packet(size, 0xFF);
    EXPECT_FALSE(continental_driver->process_packet(packet)) << "size " << size;
  }
}

TEST(TestDecoder, TestWrongSizedPayloadIsRejected)
{
  const std::vector<std::pair<uint32_t, size_t>> expected_sizes = {
    {rdi_near_header_can_message_id, rdi_near_header_packet_size},
    {rdi_near_element_can_message_id, rdi_near_element_packet_size},
    {rdi_hrr_header_can_message_id, rdi_hrr_header_packet_size},
    {rdi_hrr_element_can_message_id, rdi_hrr_element_packet_size},
    {object_header_can_message_id, object_header_packet_size},
    {object_can_message_id, object_packet_size},
    {crc_list_can_message_id, crc_list_packet_size},
    {status_can_message_id, status_packet_size},
    {sync_follow_up_can_message_id, sync_follow_up_can_packet_size},
  };
  for (const auto & [can_message_id, payload_size] : expected_sizes) {
    for (const size_t size : {size_t{0}, payload_size - 1, payload_size + 1}) {
      EXPECT_FALSE(continental_driver->process_packet(make_packet(can_message_id, size)))
        << "CAN ID " << can_message_id << ", payload size " << size;
    }
  }

  EXPECT_FALSE(continental_driver->process_packet(make_packet(0xFFFFFFFF, 8)));
}

TEST(TestDecoder, TestIgnoredMessageIdsAreAccepted)
{
  // Vehicle dynamics and sensor configuration messages are sent by the host and ignored.
  EXPECT_TRUE(continental_driver->process_packet(make_packet(veh_dyn_can_message_id, 0)));
  EXPECT_TRUE(continental_driver->process_packet(make_packet(sensor_config_can_message_id, 8)));
}

int main(int argc, char * argv[])
{
  std::cout << "continental_ros_decoder_test_main_srr520.cpp" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_continental_decoder_test";

  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  continental_driver = std::make_shared<nebula::ros::ContinentalRosDecoderTest>(options, node_name);
  exec.add_node(continental_driver->get_node_base_interface());

  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Get Status");
  nebula::Status driver_status = continental_driver->get_status();
  int result = 0;
  if (driver_status == nebula::Status::OK) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Reading Started");
    result = RUN_ALL_TESTS();
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(node_name), driver_status);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Ending");
  continental_driver.reset();
  rclcpp::shutdown();

  return result;
}
