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
#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/continental/continental_srr520_decoder_wrapper.hpp"
#include "nebula_ros/continental/continental_srr520_hw_interface_wrapper.hpp"

#include <nebula_common/continental/continental_srr520.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_srr520_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace nebula
{
namespace ros
{

/// @brief Ros wrapper of continental srr520 driver
class ContinentalSRR520RosWrapper final : public rclcpp::Node
{
public:
  explicit ContinentalSRR520RosWrapper(const rclcpp::NodeOptions & options);
  ~ContinentalSRR520RosWrapper() noexcept {};

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Start data streaming (Call SensorInterfaceStart of HwInterface)
  /// @return Resulting status
  Status StreamStart();

private:
  /// @brief Callback from the hw interface's raw data
  void ReceivePacketCallback(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg_ptr);

  /// @brief Callback from replayed NebulaPackets
  void ReceivePacketsCallback(std::unique_ptr<nebula_msgs::msg::NebulaPackets> packet_packets_msg);

  /// @brief Retrieve the parameters from ROS and set the driver and hw interface
  /// @return Resulting status
  Status DeclareAndGetSensorConfigParams();

  /// @brief rclcpp parameter callback
  /// @param parameters Received parameters
  /// @return SetParametersResult
  rcl_interfaces::msg::SetParametersResult OnParameterChange(
    const std::vector<rclcpp::Parameter> & p);

  Status ValidateAndSetConfig(
    std::shared_ptr<const drivers::continental_srr520::ContinentalSRR520SensorConfiguration> &
      new_config);

  Status wrapper_status_;

  std::shared_ptr<const drivers::continental_srr520::ContinentalSRR520SensorConfiguration>
    config_ptr_{};

  /// @brief Stores received packets that have not been processed yet by the decoder thread
  mt_queue<std::unique_ptr<nebula_msgs::msg::NebulaPacket>> packet_queue_;
  /// @brief Thread to isolate decoding from receiving
  std::thread decoder_thread_;

  rclcpp::Subscription<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_sub_{};

  bool launch_hw_{};

  std::optional<ContinentalSRR520HwInterfaceWrapper> hw_interface_wrapper_{};
  std::optional<ContinentalSRR520DecoderWrapper> decoder_wrapper_{};

  std::mutex mtx_config_;

  OnSetParametersCallbackHandle::SharedPtr parameter_event_cb_;
};

}  // namespace ros
}  // namespace nebula
