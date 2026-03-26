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

#ifndef NEBULA_SEYOND_ROS_WRAPPER_HPP
#define NEBULA_SEYOND_ROS_WRAPPER_HPP

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_seyond/hw_interface_wrapper.hpp>
#include <nebula_seyond/hw_monitor_wrapper.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_decoders/seyond_decoder.hpp>
#include <nebula_seyond_hw_interfaces/seyond_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <vector>

namespace nebula::ros
{

class SeyondRosWrapper : public rclcpp::Node
{
public:
  explicit SeyondRosWrapper(const rclcpp::NodeOptions & options);

private:
  void receive_packet_callback(
    std::vector<uint8_t> & packet,
    const nebula::drivers::connections::UdpSocket::RxMetadata & metadata);
  void publish_cloud(nebula::drivers::NebulaPointCloudPtr cloud, uint64_t base_timestamp_ns);

  void declare_parameters();
  void get_parameters();

  std::shared_ptr<nebula::drivers::SeyondHwInterface> hw_interface_;
  std::unique_ptr<SeyondHwInterfaceWrapper> hw_interface_wrapper_;
  std::unique_ptr<SeyondHwMonitorWrapper> hw_monitor_wrapper_;
  std::unique_ptr<nebula::drivers::SeyondDecoder> decoder_;
  diagnostic_updater::Updater diagnostic_updater_;
  nebula::drivers::SeyondSensorConfiguration config_;
  bool launch_hw_{true};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace nebula::ros

#endif  // NEBULA_SEYOND_ROS_WRAPPER_HPP
