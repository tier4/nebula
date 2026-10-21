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

#include "continental_ros_decoder_test_ars548.hpp"

#include <nebula_continental_common/continental_ars548.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
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

using namespace nebula::drivers::continental_ars548;  // NOLINT(build/namespaces)

/// @brief Build an ARS548 SOME/IP packet of the given total size, with service ID 0, the given
/// method ID and advertised PDU length, and a valid timestamp sync status.
std::vector<uint8_t> make_packet(uint16_t method_id, uint32_t pdu_length, size_t total_size)
{
  std::vector<uint8_t> packet(total_size, 0);
  const std::vector<uint8_t> header = {
    0x00,
    0x00,
    static_cast<uint8_t>(method_id >> 8),
    static_cast<uint8_t>(method_id & 0xFF),
    static_cast<uint8_t>(pdu_length >> 24),
    static_cast<uint8_t>((pdu_length >> 16) & 0xFF),
    static_cast<uint8_t>((pdu_length >> 8) & 0xFF),
    static_cast<uint8_t>(pdu_length & 0xFF)};
  std::copy_n(header.begin(), std::min(header.size(), total_size), packet.begin());

  const size_t sync_status_offset =
    offsetof(ObjectListPacket, stamp) + offsetof(StampSyncStatusPacket, timestamp_sync_status);
  if (total_size > sync_status_offset) {
    packet[sync_status_offset] = 1;
  }
  return packet;
}

}  // namespace

TEST(TestDecoder, TestValidObjectListsAreDecoded)
{
  // Sanity check for the tests below: correctly sized packets of both formats are decoded.
  continental_driver->set_compare_with_ground_truth(false);
  for (const auto & [pdu_length, udp_payload] :
       {std::pair{object_list_pdu_length_common, object_list_udp_payload_common},
        std::pair{object_list_pdu_length_fw40, object_list_udp_payload_fw40}}) {
    const size_t callbacks_before = continental_driver->get_object_list_callback_count();
    EXPECT_TRUE(continental_driver->process_packet(
      make_packet(object_list_method_id, pdu_length, udp_payload)));
    EXPECT_EQ(continental_driver->get_object_list_callback_count(), callbacks_before + 1)
      << "PDU length " << pdu_length;
  }
  continental_driver->set_compare_with_ground_truth(true);
}

TEST(TestDecoder, TestMalformedObjectListIsRejected)
{
  // See GHSA-vfxx-jpg9-gqcp: a packet must be rejected unless both its actual size and its
  // advertised PDU length match the same supported object list format.
  continental_driver->set_compare_with_ground_truth(false);
  const size_t callbacks_before = continental_driver->get_object_list_callback_count();

  struct Case
  {
    const char * name;
    uint32_t pdu_length;
    size_t total_size;
  };
  const std::vector<Case> cases = {
    {"empty", object_list_pdu_length_common, 0},
    {"half header", object_list_pdu_length_common, 4},
    {"header only, common PDU length", object_list_pdu_length_common, sizeof(HeaderPacket)},
    {"header only, fw40 PDU length", object_list_pdu_length_fw40, sizeof(HeaderPacket)},
    {"common list header, no objects", object_list_pdu_length_common,
     offsetof(ObjectListPacket, objects)},
    {"common one byte short", object_list_pdu_length_common, object_list_udp_payload_common - 1},
    {"common one byte long", object_list_pdu_length_common, object_list_udp_payload_common + 1},
    {"fw40 one byte short", object_list_pdu_length_fw40, object_list_udp_payload_fw40 - 1},
    {"fw40 one byte long", object_list_pdu_length_fw40, object_list_udp_payload_fw40 + 1},
    {"common size, fw40 PDU length", object_list_pdu_length_fw40, object_list_udp_payload_common},
    {"fw40 size, common PDU length", object_list_pdu_length_common, object_list_udp_payload_fw40},
    {"common size, PDU length off by one", object_list_pdu_length_common - 1,
     object_list_udp_payload_common},
    {"common size, PDU length off by one", object_list_pdu_length_common + 1,
     object_list_udp_payload_common},
    {"common size, PDU length 0", 0, object_list_udp_payload_common},
    {"common size, PDU length max", 0xFFFFFFFF, object_list_udp_payload_common},
  };
  for (const auto & c : cases) {
    EXPECT_FALSE(continental_driver->process_packet(
      make_packet(object_list_method_id, c.pdu_length, c.total_size)))
      << c.name;
  }
  EXPECT_EQ(continental_driver->get_object_list_callback_count(), callbacks_before);
  continental_driver->set_compare_with_ground_truth(true);
}

TEST(TestDecoder, TestMalformedDetectionListAndStatusAreRejected)
{
  for (const auto & [method_id, pdu_length, udp_payload] :
       {std::tuple{detection_list_method_id, detection_list_pdu_length, detection_list_udp_payload},
        std::tuple{sensor_status_method_id, sensor_status_pdu_length, sensor_status_udp_payload}}) {
    EXPECT_FALSE(
      continental_driver->process_packet(make_packet(method_id, pdu_length, sizeof(HeaderPacket))))
      << "method " << method_id;
    EXPECT_FALSE(
      continental_driver->process_packet(make_packet(method_id, pdu_length, udp_payload - 1)))
      << "method " << method_id;
    EXPECT_FALSE(
      continental_driver->process_packet(make_packet(method_id, pdu_length + 1, udp_payload)))
      << "method " << method_id;
  }
}

int main(int argc, char * argv[])
{
  std::cout << "continental_ros_decoder_test_main_ars548.cpp" << std::endl;
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
