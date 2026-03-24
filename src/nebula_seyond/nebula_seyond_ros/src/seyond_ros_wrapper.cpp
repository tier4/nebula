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

#include <nebula_core_ros/point_cloud_conversions.hpp>
#include <nebula_seyond_ros/seyond_ros_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/format.hpp>

#include <memory>
#include <string>
#include <vector>

namespace nebula::ros
{

SeyondRosWrapper::SeyondRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("seyond_ros_wrapper", options)
{
  declare_parameters();
  get_parameters();

  cloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("seyond_points", rclcpp::SensorDataQoS());

  hw_interface_ = std::make_unique<nebula::drivers::SeyondHwInterface>(config_);
  hw_interface_->register_scan_callback(
    std::bind(
      &SeyondRosWrapper::receive_packet_callback, this, std::placeholders::_1,
      std::placeholders::_2));

  if (config_.setup_sensor) {
    auto status = hw_interface_->setup_sensor(config_);
    if (status != nebula::Status::OK) {
      RCLCPP_ERROR(
        get_logger(), "Failed to setup Seyond sensor: %s",
        (boost::format("%1%") % status).str().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Seyond sensor setup successfully");
    }
  }

  // Fetch calibration
  auto calibration_or_error = hw_interface_->get_calibration();
  if (!calibration_or_error.has_value()) {
    RCLCPP_WARN(get_logger(), "Failed to fetch calibration from sensor. Using defaults.");
  }

  decoder_ = std::make_unique<nebula::drivers::SeyondDecoder>(
    config_,
    std::bind(&SeyondRosWrapper::publish_cloud, this, std::placeholders::_1, std::placeholders::_2),
    calibration_or_error.has_value() ? calibration_or_error.value()
                                     : nebula::drivers::SeyondCalibrationData{});

  auto status = hw_interface_->sensor_interface_start();
  if (status != nebula::Status::OK) {
    RCLCPP_ERROR(
      get_logger(), "Failed to start Seyond hardware interface: %s",
      (boost::format("%1%") % status).str().c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Seyond hardware interface started successfully");
  }
}

void SeyondRosWrapper::receive_packet_callback(
  std::vector<uint8_t> & packet,
  const nebula::drivers::connections::UdpSocket::RxMetadata & metadata)
{
  (void)metadata;
  decoder_->unpack(packet);
}

void SeyondRosWrapper::publish_cloud(
  nebula::drivers::NebulaPointCloudPtr cloud, uint64_t base_timestamp_ns)
{
  auto ros_msg = nebula::ros::to_ros_msg(cloud);

  // Set header
  ros_msg.header.frame_id = config_.frame_id;

  if (config_.use_sensor_time && !cloud->empty()) {
    // use the timestamp of the first point
    uint64_t ts_ns = base_timestamp_ns + cloud->front().time_stamp;
    ros_msg.header.stamp.sec = ts_ns / 1000000000;
    ros_msg.header.stamp.nanosec = ts_ns % 1000000000;
  } else {
    ros_msg.header.stamp = now();
  }

  cloud_pub_->publish(ros_msg);
}

void SeyondRosWrapper::declare_parameters()
{
  declare_parameter<std::string>("sensor_model", "FalconK");
  declare_parameter<std::string>("host_ip", "192.168.1.100");
  declare_parameter<std::string>("sensor_ip", "192.168.1.10");
  declare_parameter<int>("udp_port", 8010);
  declare_parameter<bool>("use_sensor_time", false);
  declare_parameter<std::string>("frame_id", "seyond");
  declare_parameter<bool>("setup_sensor", true);
  declare_parameter<std::string>("return_mode", "Single");
  declare_parameter<std::string>("reflectance_mode", "Reflectivity");
  declare_parameter<std::string>("time_sync", "Host");
}

void SeyondRosWrapper::get_parameters()
{
  std::string model_str = get_parameter("sensor_model").as_string();
  config_.sensor_model = nebula::drivers::seyond_sensor_model_from_string(model_str);
  config_.connection.seyond_host_ip = get_parameter("host_ip").as_string();
  config_.connection.seyond_sensor_ip = get_parameter("sensor_ip").as_string();
  config_.connection.seyond_udp_port = get_parameter("udp_port").as_int();
  config_.use_sensor_time = get_parameter("use_sensor_time").as_bool();
  config_.frame_id = get_parameter("frame_id").as_string();
  config_.setup_sensor = get_parameter("setup_sensor").as_bool();
  config_.return_mode =
    nebula::drivers::return_mode_from_string_seyond(get_parameter("return_mode").as_string());
  config_.reflectance_mode = nebula::drivers::reflectance_mode_from_string_seyond(
    get_parameter("reflectance_mode").as_string());
  config_.sync_mode =
    nebula::drivers::sync_mode_from_string_seyond(get_parameter("time_sync").as_string());
}

}  // namespace nebula::ros

RCLCPP_COMPONENTS_REGISTER_NODE(nebula::ros::SeyondRosWrapper)
