// Copyright 2026 TIER IV, Inc.

#include <nebula_seyond/seyond_ros_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace
{

rclcpp::NodeOptions make_options(const std::string & sensor_model)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("launch_hw", false);
  options.append_parameter_override("sensor_model", sensor_model);
  options.append_parameter_override("host_ip", "172.168.1.100");
  options.append_parameter_override("sensor_ip", "172.168.1.10");
  options.append_parameter_override("netmask", "255.255.255.0");
  options.append_parameter_override("gateway", "0.0.0.0");
  options.append_parameter_override("udp_port", 8010);
  options.append_parameter_override("udp_message_port", 8011);
  options.append_parameter_override("udp_status_port", 8012);
  options.append_parameter_override("setup_sensor", false);
  options.append_parameter_override("return_mode", "StrongestFurthest");
  options.append_parameter_override("reflectance_mode", "Reflectivity");
  options.append_parameter_override("time_sync", "PTP");
  options.append_parameter_override("frame_rate", 12.5);
  options.append_parameter_override("horizontal_roi", 30.0);
  options.append_parameter_override("vertical_roi", 5.0);
  return options;
}

}  // namespace

TEST(SeyondRosWrapper, ConstructsWithoutHardwareFalconK)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  auto node = std::make_shared<nebula::ros::SeyondRosWrapper>(make_options("FalconK"));
  ASSERT_NE(node, nullptr);

  rclcpp::shutdown();
}

TEST(SeyondRosWrapper, ConstructsWithoutHardwareRobinE1X)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  auto node = std::make_shared<nebula::ros::SeyondRosWrapper>(make_options("RobinE1X"));
  ASSERT_NE(node, nullptr);

  rclcpp::shutdown();
}
