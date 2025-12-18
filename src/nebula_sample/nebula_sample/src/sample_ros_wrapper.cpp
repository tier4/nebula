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
  // ========== ROS Parameter Declaration ==========
  // Implementation Items: Add more parameters for sensor configuration (IP, port, return mode,
  // etc.)
  declare_parameter("launch_hw", true);
  launch_hw_ = get_parameter("launch_hw").as_bool();

  // ========== Initialize Sensor Configuration ==========
  // Implementation Items: Read ROS parameters and populate sensor_cfg_ptr_ fields
  // Example:
  // sensor_cfg_ptr_->sensor_ip = get_parameter("sensor_ip").as_string();
  // sensor_cfg_ptr_->host_ip = get_parameter("host_ip").as_string();
  // sensor_cfg_ptr_->data_port = get_parameter("data_port").as_int();
  sensor_cfg_ptr_ = std::make_shared<drivers::SampleSensorConfiguration>();

  // ========== Initialize Driver ==========
  driver_ptr_ = std::make_shared<drivers::SampleDriver>(sensor_cfg_ptr_);

  // Register callback to receive decoded point clouds from the driver
  driver_ptr_->set_pointcloud_callback(
    [this](const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s) {
      // This callback is called when the decoder completes a full scan
      (void)timestamp_s;
      // Only publish if there are subscribers and the pointcloud is valid
      if (points_pub_->get_subscription_count() > 0 && pointcloud) {
        // Convert PCL point cloud to ROS message
        auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
        ros_pc_msg_ptr->header.stamp = this->now();
        points_pub_->publish(std::move(ros_pc_msg_ptr));
      }
    });

  // ========== Initialize Hardware Interface ==========
  // Only create HW interface if launch_hw is true (false for offline bag playback)
  if (launch_hw_) {
    hw_interface_ptr_ = std::make_shared<drivers::SampleHwInterface>();
    hw_interface_ptr_->set_sensor_configuration(sensor_cfg_ptr_);

    // Register callback to receive raw packets from the HW interface
    hw_interface_ptr_->register_scan_callback(
      std::bind(
        &SampleRosWrapper::receive_cloud_packet_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Start receiving packets
    stream_start();
  }

  // ========== Create ROS Publishers ==========
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
  // This callback is called by the HW interface when a UDP packet arrives
  // Pass the packet to the driver for decoding
  (void)metadata;  // Metadata (timestamp, source IP) not used in this simple example
  driver_ptr_->parse_cloud_packet(packet);
}

}  // namespace nebula::ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nebula::ros::SampleRosWrapper)
