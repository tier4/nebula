// Copyright 2025 TIER IV, Inc.
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

#include "nebula_sample/sample_ros_wrapper.hpp"

#include <nebula_core_common/util/expected.hpp>
#include <nebula_core_ros/parameter_descriptors.hpp>
#include <nebula_sample_common/sample_configuration.hpp>
#include <nebula_sample_hw_interfaces/sample_hw_interface.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebula::ros
{

namespace
{
template <typename T>
util::expected<T, ConfigError> declare_required_parameter(
  rclcpp::Node & node, const std::string & name,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  try {
    return node.declare_parameter<T>(name, descriptor);
  } catch (const std::exception & e) {
    return ConfigError{"Failed to declare/read parameter '" + name + "': " + e.what()};
  }
}
}  // namespace

util::expected<drivers::SampleSensorConfiguration, ConfigError> load_config_from_ros_parameters(
  rclcpp::Node & node)
{
  drivers::SampleSensorConfiguration config{};

  const auto host_ip =
    declare_required_parameter<std::string>(node, "connection.host_ip", param_read_only());
  if (!host_ip.has_value()) {
    return host_ip.error();
  }
  config.connection.host_ip = host_ip.value();

  const auto sensor_ip =
    declare_required_parameter<std::string>(node, "connection.sensor_ip", param_read_only());
  if (!sensor_ip.has_value()) {
    return sensor_ip.error();
  }
  config.connection.sensor_ip = sensor_ip.value();

  const auto data_port =
    declare_required_parameter<int64_t>(node, "connection.data_port", param_read_only());
  if (!data_port.has_value()) {
    return data_port.error();
  }
  if (data_port.value() < 0 || data_port.value() > 65535) {
    return ConfigError{
      "Parameter 'connection.data_port' must be in [0, 65535], got " +
      std::to_string(data_port.value())};
  }
  config.connection.data_port = static_cast<uint16_t>(data_port.value());

  const auto azimuth_min =
    declare_required_parameter<double>(node, "fov.azimuth.min_deg", param_read_write());
  if (!azimuth_min.has_value()) {
    return azimuth_min.error();
  }
  const auto azimuth_max =
    declare_required_parameter<double>(node, "fov.azimuth.max_deg", param_read_write());
  if (!azimuth_max.has_value()) {
    return azimuth_max.error();
  }
  const auto elevation_min =
    declare_required_parameter<double>(node, "fov.elevation.min_deg", param_read_write());
  if (!elevation_min.has_value()) {
    return elevation_min.error();
  }
  const auto elevation_max =
    declare_required_parameter<double>(node, "fov.elevation.max_deg", param_read_write());
  if (!elevation_max.has_value()) {
    return elevation_max.error();
  }

  config.fov.azimuth.start = static_cast<float>(azimuth_min.value());
  config.fov.azimuth.end = static_cast<float>(azimuth_max.value());
  config.fov.elevation.start = static_cast<float>(elevation_min.value());
  config.fov.elevation.end = static_cast<float>(elevation_max.value());

  return config;
}

SampleRosWrapper::SampleRosWrapper(const rclcpp::NodeOptions & options)
: Node("nebula_sample_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  const bool launch_hw = declare_parameter<bool>("launch_hw", true, param_read_only());
  declare_parameter<std::string>("sensor_model", "SampleSensor", param_read_only());
  frame_id_ = declare_parameter<std::string>("frame_id", "sample_lidar", param_read_write());

  const auto config_or_error = load_config_from_ros_parameters(*this);
  if (!config_or_error.has_value()) {
    throw std::runtime_error(
      "Invalid sample sensor configuration: " + config_or_error.error().message);
  }
  config_ = config_or_error.value();

  points_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("points_raw", rclcpp::SensorDataQoS());

  decoder_.emplace(
    config_.fov, [this](const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s) {
      publish_pointcloud_callback(pointcloud, timestamp_s);
    });

  if (launch_hw) {
    hw_interface_.emplace(config_.connection);
    const auto callback_result = hw_interface_->register_scan_callback(
      [this](
        const std::vector<uint8_t> & raw_packet,
        const drivers::connections::UdpSocket::RxMetadata & metadata) {
        receive_cloud_packet_callback(raw_packet, metadata);
      });
    if (!callback_result.has_value()) {
      throw std::runtime_error(
        "Failed to register sample sensor packet callback: " +
        std::string(drivers::SampleHwInterface::to_cstr(callback_result.error())));
    }

    const auto stream_start_result = hw_interface_->sensor_interface_start();
    if (!stream_start_result.has_value()) {
      throw std::runtime_error(
        "Failed to start sample sensor stream: " +
        std::string(drivers::SampleHwInterface::to_cstr(stream_start_result.error())));
    }
  }
}

void SampleRosWrapper::publish_pointcloud_callback(
  const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s)
{
  (void)timestamp_s;
  if (points_pub_->get_subscription_count() > 0 && pointcloud) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = rclcpp::Time(timestamp_s);
    ros_pc_msg_ptr->header.frame_id = frame_id_;
    points_pub_->publish(std::move(ros_pc_msg_ptr));
  }
}

void SampleRosWrapper::receive_cloud_packet_callback(
  const std::vector<uint8_t> & packet, const drivers::connections::UdpSocket::RxMetadata & metadata)
{
  (void)metadata;
  if (!decoder_) {
    return;
  }

  const auto decode_result = decoder_->unpack(packet);
  if (!decode_result.metadata_or_error.has_value()) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000, "Packet decode failed: %s.",
      drivers::to_cstr(decode_result.metadata_or_error.error()));
  }
}

const char * SampleRosWrapper::to_cstr(const Error error)
{
  switch (error) {
    case Error::HW_INTERFACE_NOT_INITIALIZED:
      return "hardware interface not initialized";
    case Error::HW_STREAM_START_FAILED:
      return "hardware stream start failed";
    default:
      return "unknown wrapper error";
  }
}

}  // namespace nebula::ros

RCLCPP_COMPONENTS_REGISTER_NODE(nebula::ros::SampleRosWrapper)
