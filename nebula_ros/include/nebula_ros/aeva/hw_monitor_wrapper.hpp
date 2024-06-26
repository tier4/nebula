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

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/aeva/config_types.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace nebula::ros
{

using nlohmann::json;

class AevaHwMonitorWrapper
{
public:
  AevaHwMonitorWrapper(
    rclcpp::Node * const parent_node, std::shared_ptr<const drivers::aeva::Aeries2Config> config);

  void onTelemetryFragment(const json & diff);

  void onHealthCodes(std::vector<uint32_t> health_codes);

private:
  struct NodeTelemetry
  {
    json values;
    rclcpp::Time last_update;
  };

  struct TelemetryState
  {
    std::map<std::string, NodeTelemetry> entries;
  };

  struct HealthState
  {
    std::vector<uint32_t> codes;
    rclcpp::Time last_update;
  };

  void publishDiagnostics();

  rclcpp::Logger logger_;
  rclcpp::Node * const parent_node_;
  std::shared_ptr<const drivers::aeva::Aeries2Config> config_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_{};
  rclcpp::TimerBase::SharedPtr diagnostics_pub_timer_{};
  uint16_t diag_span_;

  std::mutex mtx_hardware_id_;
  std::optional<std::string> hardware_id_;

  std::mutex mtx_telemetry_;
  TelemetryState telemetry_;

  std::mutex mtx_health_;
  std::optional<HealthState> health_;
};
}  // namespace nebula::ros
