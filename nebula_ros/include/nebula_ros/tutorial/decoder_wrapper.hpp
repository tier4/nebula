// Copyright 2024 TIER IV, Inc.
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

#pragma once

#include "nebula_ros/common/watchdog_timer.hpp"

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/tutorial/tutorial_common.hpp>
#include <nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_tutorial/tutorial_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace nebula::ros
{
using TutorialDriver = nebula::drivers::HesaiDriver;

class TutorialDecoderWrapper
{
public:
  TutorialDecoderWrapper(
    rclcpp::Node * const parent_node,
    const std::shared_ptr<nebula::drivers::TutorialHwInterface> & hw_interface,
    std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & config);

  void process_cloud_packet(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & new_config);

  nebula::Status status();

private:
  nebula::Status status_;
  rclcpp::Logger logger_;

  const std::shared_ptr<nebula::drivers::TutorialHwInterface> hw_interface_;
  std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> sensor_cfg_;

  std::string calibration_file_path_;

  std::shared_ptr<TutorialDriver> driver_ptr_;
  std::mutex mtx_driver_ptr_;

  rclcpp::Publisher<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_pub_;
  nebula_msgs::msg::NebulaPackets::UniquePtr current_scan_msg_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::shared_ptr<WatchdogTimer> cloud_watchdog_;
};
}  // namespace nebula::ros
