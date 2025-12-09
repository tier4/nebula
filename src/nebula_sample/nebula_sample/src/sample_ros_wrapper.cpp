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

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <utility>
#include <vector>

namespace nebula::ros
{

SampleRosWrapper::SampleRosWrapper(const rclcpp::NodeOptions & options)
: Node("nebula_sample_ros_wrapper", options)
{
  // Parameter declaration
  declare_parameter("launch_hw", true);
  launch_hw_ = get_parameter("launch_hw").as_bool();

  // Initialize config
  sensor_cfg_ptr_ = std::make_shared<drivers::SampleSensorConfiguration>();

  // Initialize Driver
  driver_ptr_ = std::make_shared<drivers::SampleDriver>(sensor_cfg_ptr_);

  driver_ptr_->set_pointcloud_callback(
    [this](const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s) {
      (void)timestamp_s;
      if (points_pub_->get_subscription_count() > 0 && pointcloud) {
        auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
        ros_pc_msg_ptr->header.stamp = this->now();
        points_pub_->publish(std::move(ros_pc_msg_ptr));
      }
    });

  // Initialize HW Interface
  if (launch_hw_) {
    hw_interface_ptr_ = std::make_shared<drivers::SampleHwInterface>();
    hw_interface_ptr_->set_sensor_configuration(sensor_cfg_ptr_);
    hw_interface_ptr_->register_scan_callback(
      std::bind(
        &SampleRosWrapper::receive_cloud_packet_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    stream_start();
  }

  points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points_raw", 10);
}

SampleRosWrapper::~SampleRosWrapper()
{
  if (hw_interface_ptr_) {
    hw_interface_ptr_->sensor_interface_stop();
  }
}

Status SampleRosWrapper::get_status()
{
  return Status::OK;
}

Status SampleRosWrapper::stream_start()
{
  if (hw_interface_ptr_) {
    return hw_interface_ptr_->sensor_interface_start();
  }
  return Status::NOT_INITIALIZED;
}

void SampleRosWrapper::receive_cloud_packet_callback(
  const std::vector<uint8_t> & packet, const drivers::connections::UdpSocket::RxMetadata & metadata)
{
  (void)metadata;
  driver_ptr_->parse_cloud_packet(packet);
}

}  // namespace nebula::ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nebula::ros::SampleRosWrapper)
