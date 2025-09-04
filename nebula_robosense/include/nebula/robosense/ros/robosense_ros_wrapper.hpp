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

#include "nebula_ros/robosense/decoder_wrapper.hpp"
#include "nebula_ros/robosense/hw_interface_wrapper.hpp"
#include "nebula_ros/robosense/hw_monitor_wrapper.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <boost_tcp_driver/tcp_driver.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/robosense/robosense_common.hpp>
#include <nebula_decoders/nebula_decoders_robosense/robosense_info_driver.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <robosense_msgs/msg/detail/robosense_info_packet__struct.hpp>
#include <robosense_msgs/msg/robosense_info_packet.hpp>
#include <robosense_msgs/msg/robosense_scan.hpp>

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

/// @brief Ros wrapper of robosense driver
class RobosenseRosWrapper final : public rclcpp::Node
{
public:
  explicit RobosenseRosWrapper(const rclcpp::NodeOptions & options);
  ~RobosenseRosWrapper() noexcept {};

  /// @brief Get current status of this driver
  /// @return Current status
  Status get_status();

  /// @brief Start point cloud streaming (Call CloudInterfaceStart of HwInterface)
  /// @return Resulting status
  Status stream_start();

private:
  void receive_cloud_packet_callback(std::vector<uint8_t> & packet);

  void receive_info_packet_callback(std::vector<uint8_t> & packet);

  void receive_scan_message_callback(std::unique_ptr<robosense_msgs::msg::RobosenseScan> scan_msg);

  nebula::Status declare_and_get_sensor_config_params();

  /// @brief rclcpp parameter callback
  /// @param parameters Received parameters
  /// @return SetParametersResult
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & p);

  nebula::Status validate_and_set_config(
    std::shared_ptr<const drivers::RobosenseSensorConfiguration> & new_config);

  nebula::Status wrapper_status_;

  std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> sensor_cfg_ptr_;

  rclcpp::Publisher<robosense_msgs::msg::RobosenseInfoPacket>::SharedPtr info_packets_pub_;

  rclcpp::Subscription<robosense_msgs::msg::RobosenseScan>::SharedPtr packets_sub_;
  rclcpp::Subscription<robosense_msgs::msg::RobosenseInfoPacket>::SharedPtr info_packets_sub_;

  bool launch_hw_;

  std::optional<RobosenseHwInterfaceWrapper> hw_interface_wrapper_;
  std::optional<RobosenseHwMonitorWrapper> hw_monitor_wrapper_;
  std::optional<RobosenseDecoderWrapper> decoder_wrapper_;

  std::optional<nebula::drivers::RobosenseInfoDriver> info_driver_;

  std::mutex mtx_config_;

  OnSetParametersCallbackHandle::SharedPtr parameter_event_cb_;
};

}  // namespace nebula::ros
