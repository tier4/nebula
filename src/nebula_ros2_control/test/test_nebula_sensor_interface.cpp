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

#include "nebula_ros2_control/nebula_sensor_interface.hpp"

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace nebula::ros2_control
{
namespace
{

using hardware_interface::CallbackReturn;
using std::chrono_literals::operator""s;
using std::chrono_literals::operator""ms;

const rclcpp_lifecycle::State k_unconfigured{
  lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured"};
const rclcpp_lifecycle::State k_inactive{
  lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"};
const rclcpp_lifecycle::State k_active{lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"};

hardware_interface::HardwareInfo make_info(
  const std::vector<std::string> & state_interfaces,
  std::unordered_map<std::string, std::string> extra_hardware_parameters = {})
{
  hardware_interface::HardwareInfo info;
  info.name = "test_lidar";
  info.type = "sensor";
  info.hardware_parameters = {
    {"plugin", "nebula::ros2_control::test::TestDriver"},
    {"library", TEST_COMPONENT_LIBRARY},
    {"namespace", "nebula_ros2_control_test"},
    {"monitor_topic", "points"},
    {"ros.rate_hz", "50.0"},
  };
  for (const auto & [name, value] : extra_hardware_parameters) {
    info.hardware_parameters[name] = value;
  }

  hardware_interface::ComponentInfo sensor;
  sensor.name = "test_lidar";
  sensor.type = "sensor";
  sensor.parameters = {{"frame_id", "test_lidar_frame"}};
  for (const auto & name : state_interfaces) {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = name;
    sensor.state_interfaces.push_back(interface_info);
  }
  info.sensors.push_back(sensor);
  return info;
}

class NebulaSensorInterfaceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(NebulaSensorInterfaceTest, RejectsInvalidUrdf)
{
  NebulaSensorInterface hardware;
  EXPECT_EQ(hardware.on_init(make_info({"temperature"})), CallbackReturn::ERROR);

  NebulaSensorInterface hardware_without_monitor;
  EXPECT_EQ(
    hardware_without_monitor.on_init(make_info({"message_rate"}, {{"monitor_topic", ""}})),
    CallbackReturn::ERROR);

  NebulaSensorInterface hardware_with_unknown_param;
  EXPECT_EQ(
    hardware_with_unknown_param.on_init(make_info({}, {{"sensor_model", "Pandar64"}})),
    CallbackReturn::ERROR);
}

TEST_F(NebulaSensorInterfaceTest, FailsToConfigureUnknownComponent)
{
  NebulaSensorInterface hardware;
  ASSERT_EQ(
    hardware.on_init(make_info({"driver_active"}, {{"plugin", "DoesNotExist"}})),
    CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.on_configure(k_unconfigured), CallbackReturn::ERROR);
}

TEST_F(NebulaSensorInterfaceTest, ReportsDriverStartupFailure)
{
  NebulaSensorInterface hardware;
  ASSERT_EQ(
    hardware.on_init(make_info({"driver_active"}, {{"ros.fail_on_start", "true"}})),
    CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(k_unconfigured), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.on_activate(k_inactive), CallbackReturn::ERROR);
}

TEST_F(NebulaSensorInterfaceTest, RunsDriverAndExportsState)
{
  // A parameter file with launch-style substitutions, as shipped by the Nebula vendor packages.
  const auto params_file =
    (std::filesystem::temp_directory_path() / "nebula_ros2_control_test.param.yaml").string();
  {
    std::ofstream file(params_file);
    file << "/**:\n  ros__parameters:\n    stamp_sec: $(env NEBULA_ROS2_CONTROL_TEST_STAMP)\n";
  }
  setenv("NEBULA_ROS2_CONTROL_TEST_STAMP", "42", 1);

  NebulaSensorInterface hardware;
  ASSERT_EQ(
    hardware.on_init(make_info(
      {"driver_active", "message_count", "message_rate", "message_age", "header_stamp"},
      {{"params_file", params_file}})),
    CallbackReturn::SUCCESS);

  auto interfaces = hardware.export_state_interfaces();
  ASSERT_EQ(interfaces.size(), 5U);
  std::map<std::string, hardware_interface::StateInterface *> by_name;
  for (auto & interface : interfaces) {
    EXPECT_EQ(interface.get_prefix_name(), "test_lidar");
    by_name[interface.get_interface_name()] = &interface;
  }
  const auto value = [&](const std::string & name) {
  // get_value() is deprecated on newer ros2_control releases, but available everywhere.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    return by_name.at(name)->get_value();
#pragma GCC diagnostic pop
  };

  ASSERT_EQ(hardware.on_configure(k_unconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
  EXPECT_EQ(value("driver_active"), 0.0);
  EXPECT_TRUE(std::isnan(value("header_stamp")));

  // An independent subscriber sees the driver under the configured name, with the URDF parameters
  // (frame_id from <sensor>, stamp_sec from the substituted parameter file) applied.
  auto observer = std::make_shared<rclcpp::Node>("observer");
  std::string received_frame_id;
  auto subscription = observer->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/nebula_ros2_control_test/points", rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      received_frame_id = msg->header.frame_id;
    });

  ASSERT_EQ(hardware.on_activate(k_inactive), CallbackReturn::SUCCESS);

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(observer);
    hardware.read(rclcpp::Time(), rclcpp::Duration(0, 0));
    if (!received_frame_id.empty() && value("message_rate") > 0.0) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  EXPECT_EQ(received_frame_id, "test_lidar_frame");
  EXPECT_EQ(value("driver_active"), 1.0);
  EXPECT_GT(value("message_count"), 0.0);
  EXPECT_GT(value("message_rate"), 10.0);
  EXPECT_LT(value("message_rate"), 100.0);
  EXPECT_GE(value("message_age"), 0.0);
  EXPECT_LT(value("message_age"), 1.0);
  EXPECT_DOUBLE_EQ(value("header_stamp"), 42.5);

  const auto graph_names = observer->get_node_names();
  EXPECT_NE(
    std::find(
      graph_names.begin(), graph_names.end(), "/nebula_ros2_control_test/test_lidar_driver"),
    graph_names.end());

  ASSERT_EQ(hardware.on_deactivate(k_active), CallbackReturn::SUCCESS);
  hardware.read(rclcpp::Time(), rclcpp::Duration(0, 0));
  EXPECT_EQ(value("driver_active"), 0.0);
  EXPECT_EQ(value("message_rate"), 0.0);

  // Can be reactivated after deactivation.
  ASSERT_EQ(hardware.on_activate(k_inactive), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_deactivate(k_active), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_cleanup(k_inactive), CallbackReturn::SUCCESS);
  std::filesystem::remove(params_file);
}

}  // namespace
}  // namespace nebula::ros2_control
