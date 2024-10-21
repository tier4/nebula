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

#include "nebula_ros/common/mt_queue.hpp"
#include "nebula_ros/tutorial/decoder_wrapper.hpp"
#include "nebula_ros/tutorial/hw_interface_wrapper.hpp"
#include "nebula_ros/tutorial/hw_monitor_wrapper.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/tutorial/tutorial_common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>

#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace nebula::ros
{
using TutorialCalibrationConfiguration = nebula::drivers::HesaiCalibrationConfiguration;
constexpr auto return_mode_from_string = nebula::drivers::return_mode_from_string_hesai;

class TutorialRosWrapper final : public rclcpp::Node
{
public:
  explicit TutorialRosWrapper(const rclcpp::NodeOptions & options);
  ~TutorialRosWrapper() noexcept override = default;

  Status get_status();

  Status stream_start();

private:
  void receive_cloud_packet_callback(std::vector<uint8_t> & packet);

  void receive_scan_message_callback(std::unique_ptr<nebula_msgs::msg::NebulaPackets> scan_msg);

  Status declare_and_get_sensor_config_params();

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & p);

  Status validate_and_set_config(
    std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & new_config);

  Status wrapper_status_;

  std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> sensor_cfg_ptr_;
  std::mutex mtx_config_;

  MtQueue<std::unique_ptr<nebula_msgs::msg::NebulaPacket>> packet_queue_;

  /// @brief Thread to isolate decoding from receiving
  std::thread decoder_thread_;

  rclcpp::Subscription<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_sub_;

  bool launch_hw_;

  std::optional<TutorialHwInterfaceWrapper> hw_interface_wrapper_;
  std::optional<TutorialHwMonitorWrapper> hw_monitor_wrapper_;
  std::optional<TutorialDecoderWrapper> decoder_wrapper_;

  OnSetParametersCallbackHandle::SharedPtr parameter_event_cb_;
};

}  // namespace nebula::ros
