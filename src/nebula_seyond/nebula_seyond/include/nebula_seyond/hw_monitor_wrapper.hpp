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

#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_hw_interfaces/seyond_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace nebula::ros
{

class SeyondHwMonitorWrapper
{
public:
  SeyondHwMonitorWrapper(
    rclcpp::Node * parent_node, diagnostic_updater::Updater & diagnostic_updater,
    const std::shared_ptr<nebula::drivers::SeyondHwInterface> & hw_interface,
    const std::shared_ptr<const nebula::drivers::SeyondSensorConfiguration> & config);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::SeyondSensorConfiguration> & new_config);

  [[nodiscard]] nebula::Status status() const;

private:
  struct MonitorSnapshot
  {
    std::string udp_ports_ip;
    std::string return_mode;
    std::string reflectance_mode;
    std::string frame_rate;
    std::string v_angle_offset;
  };

  void initialize_diagnostics(diagnostic_updater::Updater & diagnostic_updater);
  void fetch_diagnostics(diagnostic_updater::Updater & diagnostic_updater);
  void check_connection(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void check_configuration(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  [[nodiscard]] bool is_stale(const rclcpp::Time & stamp) const;

  rclcpp::Logger logger_;
  nebula::Status status_;
  std::shared_ptr<nebula::drivers::SeyondHwInterface> hw_interface_;
  std::shared_ptr<const nebula::drivers::SeyondSensorConfiguration> config_;
  rclcpp::Node * const parent_node_;
  uint16_t diag_span_ms_;
  rclcpp::TimerBase::SharedPtr fetch_diagnostics_timer_;
  std::unique_ptr<rclcpp::Time> last_update_time_;
  MonitorSnapshot snapshot_;
  std::string last_error_;
  mutable std::mutex mutex_;
};

}  // namespace nebula::ros
