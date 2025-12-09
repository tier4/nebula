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

#ifndef NEBULA_SAMPLE_ROS_WRAPPER_HPP
#define NEBULA_SAMPLE_ROS_WRAPPER_HPP

#include "nebula_core_common/nebula_common.hpp"
#include "nebula_core_common/nebula_status.hpp"
#include "nebula_sample_common/sample_common.hpp"
#include "nebula_sample_decoders/sample_driver.hpp"
#include "nebula_sample_hw_interfaces/sample_hw_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <vector>

namespace nebula::ros
{

class SampleRosWrapper : public rclcpp::Node
{
public:
  explicit SampleRosWrapper(const rclcpp::NodeOptions & options);
  ~SampleRosWrapper() override;

  Status get_status();
  Status stream_start();

private:
  void receive_cloud_packet_callback(
    const std::vector<uint8_t> & packet,
    const drivers::connections::UdpSocket::RxMetadata & metadata);

  std::shared_ptr<drivers::SampleSensorConfiguration> sensor_cfg_ptr_;

  std::shared_ptr<drivers::SampleDriver> driver_ptr_;
  std::shared_ptr<drivers::SampleHwInterface> hw_interface_ptr_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;

  bool launch_hw_;
};

}  // namespace nebula::ros

#endif  // NEBULA_SAMPLE_ROS_WRAPPER_HPP
