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

#include "convert.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using nebula::cli::Args;
using nebula::cli::SensorArgs;

namespace
{
/// VLP16 single-strongest packet: 12 blocks, azimuth advancing by `step` centi-degrees per block
velodyne_msgs::msg::VelodynePacket vlp16_packet(uint32_t start_azimuth, uint32_t step)
{
  velodyne_msgs::msg::VelodynePacket p;
  p.data.fill(0);
  for (uint32_t b = 0; b < 12; ++b) {
    auto * block = &p.data[b * 100];
    block[0] = 0xFF;
    block[1] = 0xEE;
    const uint16_t az = static_cast<uint16_t>((start_azimuth + b * step) % 36000);
    block[2] = az & 0xFF;
    block[3] = az >> 8;
    for (int i = 0; i < 32; ++i) {
      const uint16_t dist = 2000 + 10 * i;  // 2 mm units
      block[4 + i * 3] = dist & 0xFF;
      block[5 + i * 3] = dist >> 8;
      block[6 + i * 3] = 100;
    }
  }
  p.data[1204] = 55;    // strongest
  p.data[1205] = 0x22;  // VLP16
  return p;
}

/// Bag with `scans` full rotations on /packets and one std_msgs/String on /other
std::string make_bag(const fs::path & dir, int scans)
{
  fs::remove_all(dir);
  rosbag2_storage::StorageOptions o;
  o.uri = dir.string();
  o.storage_id = "sqlite3";
  rosbag2_cpp::Writer w;
  w.open(o, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  for (const auto & [name, type] : {std::pair<const char *, const char *>{
                                      "/packets", "velodyne_msgs/msg/VelodyneScan"},
                                    {"/other", "std_msgs/msg/String"}}) {
    rosbag2_storage::TopicMetadata topic;
    topic.name = name;
    topic.type = type;
    topic.serialization_format = "cdr";
    w.create_topic(topic);
  }
  int64_t t = 1'000'000'000;
  for (int s = 0; s < scans; ++s) {
    velodyne_msgs::msg::VelodyneScan scan;
    // 12 blocks * 300 = 3600 centi-deg per packet -> 10 packets per rotation
    for (uint32_t k = 0; k < 10; ++k) scan.packets.push_back(vlp16_packet(k * 3600, 300));
    for (auto & p : scan.packets) {
      p.stamp = rclcpp::Time(t);
      t += 1'000'000;
    }
    w.write(scan, "/packets", rclcpp::Time(t));
  }
  std_msgs::msg::String str;
  str.data = "hello";
  w.write(str, "/other", rclcpp::Time(t));
  return dir.string();
}

std::map<std::string, size_t> count_topics(const std::string & path)
{
  rosbag2_storage::StorageOptions o;
  o.uri = path;
  rosbag2_cpp::Reader r;
  r.open(o);
  std::map<std::string, size_t> counts;
  while (r.has_next()) ++counts[r.read_next()->topic_name];
  return counts;
}

Args make_args(const std::string & in, const std::string & out)
{
  Args a;
  a.input = in;
  a.output = out;
  SensorArgs s;
  s.vendor = "velodyne";
  s.topic = "/packets";
  s.output_topic = "/points";
  a.sensors.push_back(s);
  return a;
}

class Convert : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = fs::temp_directory_path() / ("nebula_cli_convert_" + std::to_string(::getpid()));
    fs::remove_all(root_);
    fs::create_directories(root_);
    in_ = make_bag(root_ / "in", 3);
    out_ = (root_ / "out").string();
  }
  void TearDown() override { fs::remove_all(root_); }
  fs::path root_;
  std::string in_, out_;
};
}  // namespace

TEST_F(Convert, DecodesAndCopiesOtherTopics)
{
  EXPECT_EQ(nebula::cli::convert(make_args(in_, out_)), 0);
  const auto counts = count_topics(out_);
  EXPECT_GE(counts.at("/points"), 1u);
  EXPECT_EQ(counts.at("/packets"), 3u);
  EXPECT_EQ(counts.at("/other"), 1u);
}

TEST_F(Convert, DropOtherTopicsAndPointFormats)
{
  for (const std::string fmt : {"xyzir", "xyziradt", "full"}) {
    fs::remove_all(out_);
    auto a = make_args(in_, out_);
    a.copy_other = false;
    a.sensors[0].point_format = fmt;
    EXPECT_EQ(nebula::cli::convert(a), 0) << fmt;
    const auto counts = count_topics(out_);
    EXPECT_GE(counts.at("/points"), 1u) << fmt;
    EXPECT_EQ(counts.count("/packets"), 0u);
  }
}

TEST_F(Convert, MaxCloudsStopsEarly)
{
  auto a = make_args(in_, out_);
  a.max_clouds = 1;
  EXPECT_EQ(nebula::cli::convert(a), 0);
  EXPECT_EQ(count_topics(out_).at("/points"), 1u);
}

TEST_F(Convert, ExplicitModelAndAssignments)
{
  auto a = make_args(in_, out_);
  a.sensors[0].flag_params = {{"sensor_model", "VLP16"}, {"return_mode", "Strongest"}};
  a.sensors[0].assignments = {"frame_id:=lidar_top"};
  EXPECT_EQ(nebula::cli::convert(a), 0);
}

TEST_F(Convert, ParamsFile)
{
  const auto yaml = root_ / "p.yaml";
  std::ofstream(yaml) << "/**:\n  ros__parameters:\n    sensor_model: VLP16\n    return_mode: Strongest\n";
  auto a = make_args(in_, out_);
  a.sensors[0].params_file = yaml.string();
  EXPECT_EQ(nebula::cli::convert(a), 0);
}

TEST_F(Convert, WrongTopicTypeThrows)
{
  auto a = make_args(in_, out_);
  a.sensors[0].topic = "/other";  // wrong type: String is not a packet message
  EXPECT_THROW(nebula::cli::convert(a), std::runtime_error);
}

TEST_F(Convert, Errors)
{
  auto a = make_args(in_, out_);
  a.sensors[0].topic = "/missing";
  EXPECT_THROW(nebula::cli::convert(a), std::runtime_error);
  EXPECT_FALSE(fs::exists(out_)) << "no output bag may be left behind";

  a = make_args(in_, out_);
  a.sensors[0].output_topic = "/packets";  // exists in the input and other topics are copied
  EXPECT_THROW(nebula::cli::convert(a), std::runtime_error);

  a = make_args(in_, out_);
  a.sensors[0].vendor = "robosense";  // needs --info-topic
  EXPECT_THROW(nebula::cli::convert(a), std::runtime_error);

  a = make_args(in_, out_);
  a.sensors[0].vendor = "robosense";
  a.sensors[0].info_topic = "/other";
  EXPECT_THROW(nebula::cli::convert(a), std::runtime_error);  // model cannot be detected

  a = make_args(in_, out_);
  a.sensors[0].flag_params = {{"sensor_model", "Nope"}, {"return_mode", "Strongest"}};
  EXPECT_THROW(nebula::cli::convert(a), std::runtime_error);

  a = make_args((root_ / "nonexistent").string(), out_);
  EXPECT_THROW(nebula::cli::convert(a), std::exception);

  // an existing output path must fail cleanly
  fs::create_directories(out_);
  EXPECT_ANY_THROW(nebula::cli::convert(make_args(in_, out_)));
}
