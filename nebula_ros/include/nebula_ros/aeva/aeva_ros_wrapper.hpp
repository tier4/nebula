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

#include "nebula_ros/aeva/hw_monitor_wrapper.hpp"
#include "nebula_ros/common/watchdog_timer.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <nebula_common/aeva/config_types.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/util/mt_queue.hpp>
#include <nebula_decoders/nebula_decoders_aeva/aeva_aries2_decoder.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_aeva/aeva_hw_interface.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace nebula::ros
{

/// @brief Ros wrapper of hesai driver
class AevaRosWrapper final : public rclcpp::Node
{
public:
  explicit AevaRosWrapper(const rclcpp::NodeOptions & options);

private:
  Status declareAndGetSensorConfigParams();
  Status validateAndSetConfig(std::shared_ptr<const drivers::aeva::Aeries2Config> & new_config);

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & p);

  void recordRawPacket(const std::vector<uint8_t> & bytes);

  rclcpp::Publisher<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_pub_;
  std::mutex mtx_current_scan_msg_;
  nebula_msgs::msg::NebulaPackets::UniquePtr current_scan_msg_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::shared_ptr<nebula::ros::WatchdogTimer> cloud_watchdog_;

  rclcpp::Subscription<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_sub_;

  std::shared_ptr<const drivers::aeva::Aeries2Config> sensor_cfg_ptr_;

  std::optional<drivers::AevaHwInterface> hw_interface_;
  std::optional<AevaHwMonitorWrapper> hw_monitor_;
  drivers::AevaAries2Decoder decoder_;

  OnSetParametersCallbackHandle::SharedPtr parameter_event_cb_;
};

}  // namespace nebula::ros
