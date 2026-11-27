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

#include "nebula_ros2_control/driver_config.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace nebula::ros2_control
{

TEST(DriverConfig, MinimalConfigUsesDefaults)
{
  const auto config = parse_driver_config("front_lidar", {{"plugin", "HesaiRosWrapper"}});
  EXPECT_EQ(config.plugin, "HesaiRosWrapper");
  EXPECT_EQ(config.node_name, "front_lidar_driver");
  EXPECT_TRUE(config.node_namespace.empty());
  EXPECT_TRUE(config.package.empty());
  EXPECT_TRUE(config.parameters.empty());
  EXPECT_EQ(config.executor_threads, 1U);
  EXPECT_FALSE(config.use_intra_process_comms);
  EXPECT_TRUE(config.monitor_topic.empty());
  EXPECT_TRUE(config.allow_substs);
}

TEST(DriverConfig, MissingPluginIsRejected)
{
  EXPECT_THROW(parse_driver_config("front_lidar", {}), ConfigError);
}

TEST(DriverConfig, UnknownHardwareParameterIsRejected)
{
  EXPECT_THROW(
    parse_driver_config("front_lidar", {{"plugin", "X"}, {"sensor_model", "Pandar64"}}),
    ConfigError);
}

TEST(DriverConfig, FullConfig)
{
  const auto config = parse_driver_config(
    "front_lidar",
    {
      {"plugin", "HesaiRosWrapper"},
      {"package", "nebula_hesai"},
      {"node_name", "hesai_front"},
      {"namespace", "sensing/lidar/front"},
      {"params_file", "/a.yaml; /b.yaml"},
      {"remappings", "pandar_points:=pointcloud_raw_ex; pandar_packets:=packets"},
      {"use_intra_process_comms", "True"},
      {"executor_threads", "4"},
      {"monitor_topic", "pointcloud_raw_ex"},
      {"ros.sensor_model", "Pandar64"},
      {"ros.frame_id", "overridden_by_sensor"},
      {"ros.fov.azimuth.min_deg", "0.0"},
    },
    {{"frame_id", "lidar_front"}, {"ros.return_mode", "Dual"}});

  EXPECT_EQ(config.package, "nebula_hesai");
  EXPECT_EQ(config.node_name, "hesai_front");
  EXPECT_EQ(config.node_namespace, "/sensing/lidar/front");
  EXPECT_EQ(config.params_files, (std::vector<std::string>{"/a.yaml", "/b.yaml"}));
  ASSERT_EQ(config.remappings.size(), 2U);
  EXPECT_EQ(
    config.remappings[0],
    (std::pair<std::string, std::string>{"pandar_points", "pointcloud_raw_ex"}));
  EXPECT_TRUE(config.use_intra_process_comms);
  EXPECT_EQ(config.executor_threads, 4U);
  EXPECT_EQ(config.monitor_topic, "pointcloud_raw_ex");
  EXPECT_EQ(config.monitor_topic_type, "sensor_msgs/msg/PointCloud2");

  const std::vector<std::pair<std::string, std::string>> expected_parameters{
    {"fov.azimuth.min_deg", "0.0"},
    {"frame_id", "lidar_front"},
    {"return_mode", "Dual"},
    {"sensor_model", "Pandar64"},
  };
  EXPECT_EQ(config.parameters, expected_parameters);
}

TEST(DriverConfig, InvalidValuesAreRejected)
{
  EXPECT_THROW(parse_driver_config("l", {{"plugin", "X"}, {"executor_threads", "0"}}), ConfigError);
  EXPECT_THROW(
    parse_driver_config("l", {{"plugin", "X"}, {"executor_threads", "two"}}), ConfigError);
  EXPECT_THROW(
    parse_driver_config("l", {{"plugin", "X"}, {"use_intra_process_comms", "maybe"}}), ConfigError);
  EXPECT_THROW(parse_driver_config("l", {{"plugin", "X"}, {"remappings", "a=b"}}), ConfigError);
  EXPECT_THROW(parse_driver_config("l", {{"plugin", "X"}, {"ros.", "1"}}), ConfigError);
  EXPECT_THROW(parse_driver_config("l", {{"plugin", "X"}}, {{"min_range", "1"}}), ConfigError);
}

TEST(DriverConfig, RosArguments)
{
  DriverConfig config;
  config.plugin = "HesaiRosWrapper";
  config.node_name = "hesai";
  config.node_namespace = "/lidar";
  config.params_files = {"/a.yaml"};
  config.parameters = {{"sensor_model", "Pandar64"}, {"host_ip", ""}};
  config.remappings = {{"pandar_points", "points"}};

  const std::vector<std::string> expected{
    "--ros-args",
    "-r",
    "__node:=hesai",
    "-r",
    "__ns:=/lidar",
    "--params-file",
    "/a.yaml",
    "-p",
    "sensor_model:=Pandar64",
    "-p",
    "host_ip:=''",
    "-r",
    "pandar_points:=points"};
  EXPECT_EQ(build_ros_arguments(config), expected);
}

TEST(DriverConfig, StateInterfaceKinds)
{
  EXPECT_EQ(parse_state_interface_kind("driver_active"), StateInterfaceKind::driver_active);
  EXPECT_EQ(parse_state_interface_kind("message_rate"), StateInterfaceKind::message_rate);
  EXPECT_FALSE(requires_monitor_topic(StateInterfaceKind::driver_active));
  EXPECT_TRUE(requires_monitor_topic(StateInterfaceKind::header_stamp));
  EXPECT_THROW(parse_state_interface_kind("temperature"), ConfigError);
}

TEST(DriverConfig, FactoryClassName)
{
  EXPECT_EQ(
    component_factory_class_name("nebula::ros::SampleRosWrapper"),
    "rclcpp_components::NodeFactoryTemplate<nebula::ros::SampleRosWrapper>");
}

TEST(HeaderStamp, LittleEndianCdr)
{
  // sec = 1700000000 (0x6553F100), nanosec = 250000000 (0x0EE6B280)
  const std::vector<std::uint8_t> data{0x00, 0x01, 0x00, 0x00, 0x00, 0xF1, 0x53,
                                       0x65, 0x80, 0xB2, 0xE6, 0x0E, 0x05, 0x00};
  const auto stamp = parse_header_stamp(data.data(), data.size());
  ASSERT_TRUE(stamp.has_value());
  EXPECT_DOUBLE_EQ(*stamp, 1700000000.25);
}

TEST(HeaderStamp, BigEndianCdr)
{
  const std::vector<std::uint8_t> data{0x00, 0x00, 0x00, 0x00, 0x65, 0x53,
                                       0xF1, 0x00, 0x0E, 0xE6, 0xB2, 0x80};
  const auto stamp = parse_header_stamp(data.data(), data.size());
  ASSERT_TRUE(stamp.has_value());
  EXPECT_DOUBLE_EQ(*stamp, 1700000000.25);
}

TEST(HeaderStamp, DelimitedCdr2SkipsDheader)
{
  const std::vector<std::uint8_t> data{0x00, 0x09, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
                                       0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const auto stamp = parse_header_stamp(data.data(), data.size());
  ASSERT_TRUE(stamp.has_value());
  EXPECT_DOUBLE_EQ(*stamp, 2.0);
}

TEST(HeaderStamp, InvalidInput)
{
  const std::vector<std::uint8_t> too_short{0x00, 0x01, 0x00, 0x00, 0x01};
  EXPECT_FALSE(parse_header_stamp(too_short.data(), too_short.size()).has_value());
  const std::vector<std::uint8_t> parameter_list{0x00, 0x03, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_FALSE(parse_header_stamp(parameter_list.data(), parameter_list.size()).has_value());
  EXPECT_FALSE(parse_header_stamp(nullptr, 0).has_value());
}

TEST(Substitutions, ResolvesAllOccurrences)
{
  std::vector<std::pair<std::string, std::string>> calls;
  const auto resolver = [&](const std::string & kind, const std::string & argument) {
    calls.emplace_back(kind, argument);
    return "<" + kind + ":" + argument + ">";
  };
  EXPECT_EQ(
    apply_substitutions(
      "calibration_file: $(find-pkg-share nebula_hesai_decoders)/calibration/$( var  "
      "sensor_model ).csv",
      resolver),
    "calibration_file: <find-pkg-share:nebula_hesai_decoders>/calibration/<var:sensor_model>.csv");
  EXPECT_EQ(calls.size(), 2U);
  EXPECT_EQ(apply_substitutions("no substitutions: $ (x)", resolver), "no substitutions: $ (x)");
  EXPECT_THROW(apply_substitutions("broken: $(var x", resolver), ConfigError);
  EXPECT_THROW(apply_substitutions("empty: $()", resolver), ConfigError);
}

TEST(Substitutions, VarComesFromUrdfParameters)
{
  const auto config = parse_driver_config(
    "l", {{"plugin", "X"}, {"ros.sensor_model", "Pandar64"}, {"ros.quoted", "'abc'"}});
  EXPECT_EQ(resolve_var_substitution(config, "sensor_model"), "Pandar64");
  EXPECT_EQ(resolve_var_substitution(config, "quoted"), "abc");
  EXPECT_THROW(resolve_var_substitution(config, "launch_hw"), ConfigError);
}

TEST(SplitList, TrimsAndDropsEmpty)
{
  EXPECT_EQ(split_list(" a ;; b;"), (std::vector<std::string>{"a", "b"}));
  EXPECT_TRUE(split_list("").empty());
}

}  // namespace nebula::ros2_control
