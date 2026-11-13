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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <robosense_msgs/msg/robosense_info_packet.hpp>
#include <robosense_msgs/msg/robosense_scan.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace nebula::cli
{
namespace
{
/// Stamp conversion shared by all packet message types
int64_t to_ns(const builtin_interfaces::msg::Time & t)
{
  return rclcpp::Time(t).nanoseconds();
}

template <typename Msg>
Msg deserialize(const rosbag2_storage::SerializedBagMessage & bag_msg)
{
  Msg msg;
  rclcpp::SerializedMessage serialized(*bag_msg.serialized_data);
  rclcpp::Serialization<Msg>().deserialize_message(&serialized, &msg);
  return msg;
}

}  // namespace

bool unpack(
  const std::string & type, const rosbag2_storage::SerializedBagMessage & m,
  std::vector<Packet> & out)
{
  if (type == "pandar_msgs/msg/PandarScan") {
    for (const auto & p : deserialize<pandar_msgs::msg::PandarScan>(m).packets) {
      // The array is fixed-size; `size` is the valid length (clamped: a corrupt message may lie)
      const auto size = std::min<size_t>(p.size, p.data.size());
      out.push_back({to_ns(p.stamp), {p.data.begin(), p.data.begin() + size}});
    }
  } else if (type == "velodyne_msgs/msg/VelodyneScan") {
    for (const auto & p : deserialize<velodyne_msgs::msg::VelodyneScan>(m).packets) {
      out.push_back({to_ns(p.stamp), {p.data.begin(), p.data.end()}});
    }
  } else if (type == "robosense_msgs/msg/RobosenseScan") {
    for (const auto & p : deserialize<robosense_msgs::msg::RobosenseScan>(m).packets) {
      out.push_back({to_ns(p.stamp), {p.data.begin(), p.data.end()}});
    }
  } else if (type == "robosense_msgs/msg/RobosenseInfoPacket") {
    const auto p = deserialize<robosense_msgs::msg::RobosenseInfoPacket>(m).packet;
    out.push_back({to_ns(p.stamp), {p.data.begin(), p.data.end()}});
  } else if (type == "nebula_msgs/msg/NebulaPackets") {
    for (const auto & p : deserialize<nebula_msgs::msg::NebulaPackets>(m).packets) {
      out.push_back({to_ns(p.stamp), p.data});
    }
  } else {
    return false;
  }
  return true;
}

}  // namespace nebula::cli
