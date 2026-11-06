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

#include "unpack.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <robosense_msgs/msg/robosense_info_packet.hpp>
#include <robosense_msgs/msg/robosense_scan.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using nebula::cli::Packet;
using nebula::cli::unpack;

namespace
{
template <typename Msg>
rosbag2_storage::SerializedBagMessage serialize(const Msg & msg)
{
  auto s = std::make_shared<rclcpp::SerializedMessage>();
  rclcpp::Serialization<Msg>().serialize_message(&msg, s.get());
  rosbag2_storage::SerializedBagMessage bag;
  // aliasing pointer: the buffer lives as long as the owning SerializedMessage
  bag.serialized_data =
    std::shared_ptr<rcutils_uint8_array_t>(s, &s->get_rcl_serialized_message());
  return bag;
}
}  // namespace

TEST(Unpack, Pandar)
{
  pandar_msgs::msg::PandarScan scan;
  pandar_msgs::msg::PandarPacket p;
  p.stamp.sec = 2;
  p.stamp.nanosec = 5;
  p.data[0] = 7;
  p.size = 3;
  scan.packets.push_back(p);
  p.size = 60000;  // lies: clamped to the array size
  scan.packets.push_back(p);
  std::vector<Packet> out;
  ASSERT_TRUE(unpack("pandar_msgs/msg/PandarScan", serialize(scan), out));
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].stamp_ns, 2'000'000'005);
  EXPECT_EQ(out[0].data.size(), 3u);
  EXPECT_EQ(out[0].data[0], 7);
  EXPECT_EQ(out[1].data.size(), p.data.size());
}

TEST(Unpack, Velodyne)
{
  velodyne_msgs::msg::VelodyneScan scan;
  velodyne_msgs::msg::VelodynePacket p;
  p.stamp.sec = 1;
  scan.packets.push_back(p);
  std::vector<Packet> out;
  ASSERT_TRUE(unpack("velodyne_msgs/msg/VelodyneScan", serialize(scan), out));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].stamp_ns, 1'000'000'000);
  EXPECT_EQ(out[0].data.size(), p.data.size());
}

TEST(Unpack, Robosense)
{
  robosense_msgs::msg::RobosenseScan scan;
  scan.packets.emplace_back();
  scan.packets.emplace_back();
  std::vector<Packet> out;
  ASSERT_TRUE(unpack("robosense_msgs/msg/RobosenseScan", serialize(scan), out));
  EXPECT_EQ(out.size(), 2u);

  robosense_msgs::msg::RobosenseInfoPacket info;
  info.packet.stamp.sec = 3;
  out.clear();
  ASSERT_TRUE(unpack("robosense_msgs/msg/RobosenseInfoPacket", serialize(info), out));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].stamp_ns, 3'000'000'000);
}

TEST(Unpack, NebulaPackets)
{
  nebula_msgs::msg::NebulaPackets msgs;
  nebula_msgs::msg::NebulaPacket p;
  p.data = {1, 2, 3};
  msgs.packets.push_back(p);
  std::vector<Packet> out;
  ASSERT_TRUE(unpack("nebula_msgs/msg/NebulaPackets", serialize(msgs), out));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].data, (std::vector<uint8_t>{1, 2, 3}));
}

TEST(Unpack, UnsupportedTypeLeavesOutputUntouched)
{
  std::vector<Packet> out;
  EXPECT_FALSE(unpack("std_msgs/msg/String", rosbag2_storage::SerializedBagMessage{}, out));
  EXPECT_TRUE(out.empty());
}
