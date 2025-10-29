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

#include "nebula_ros/common/single_consumer_processor.hpp"
#include "nebula_ros/common/sync_tooling/sync_tooling_worker.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <string>

namespace nebula::ros
{

using nlohmann::json;

class HesaiHwMonitorWrapper
{
public:
  HesaiHwMonitorWrapper(
    rclcpp::Node * parent_node, diagnostic_updater::Updater & diagnostic_updater,
    const std::shared_ptr<nebula::drivers::HesaiHwInterface> & hw_interface,
    const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config,
    const std::shared_ptr<SyncToolingWorker> & sync_tooling_worker);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & /* new_config */)
  {
  }

  // This is currently static as it returns a dummy status for legacy reasons
  static nebula::Status status();

private:
  static void add_json_item_to_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & diagnostics, const std::string & key,
    const json & value);

  void initialize_hesai_diagnostics(diagnostic_updater::Updater & diagnostic_updater);

  static std::string get_ptree_value(boost::property_tree::ptree * pt, const std::string & key);

  static std::string get_fixed_precision_string(double val, int pre);

  void fetch_status();

  void fetch_monitor_http();

  void fetch_monitor_tcp();

  void fetch_sync_diag();

  void hesai_check_status(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void hesai_check_ptp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void hesai_check_temperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void hesai_check_rpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void hesai_check_voltage_http(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void hesai_check_voltage(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void submit_clock_state(const HesaiLidarStatusBase & status);

  void fetch_diagnostics_from_sensor();

  [[nodiscard]] bool is_stale(const rclcpp::Time & last_update) const;

  rclcpp::Logger logger_;
  nebula::Status status_;

  const std::shared_ptr<nebula::drivers::HesaiHwInterface> hw_interface_;
  rclcpp::Node * const parent_node_;

  uint16_t diag_span_ms_;
  bool monitor_enabled_;

  rclcpp::TimerBase::SharedPtr fetch_diagnostics_timer_;

  std::shared_ptr<HesaiLidarStatusBase> current_status_;
  std::shared_ptr<HesaiLidarMonitor> current_monitor_;
  std::shared_ptr<HesaiConfigBase> current_config_;
  std::shared_ptr<boost::property_tree::ptree> current_lidar_monitor_tree_;

  std::unique_ptr<rclcpp::Time> current_status_time_;
  std::unique_ptr<rclcpp::Time> current_config_time_;
  std::unique_ptr<rclcpp::Time> current_lidar_monitor_time_;

  uint8_t current_diag_status_{diagnostic_msgs::msg::DiagnosticStatus::STALE};
  uint8_t current_monitor_status_{diagnostic_msgs::msg::DiagnosticStatus::STALE};

  std::mutex mtx_lidar_status_;
  std::mutex mtx_lidar_monitor_;

  std::shared_ptr<SyncToolingWorker> sync_tooling_worker_;
  /// @brief A separate thread that handles blocking TCP requests to the sensor,
  /// with a thread-safe queue that ensures there is at most one in-flight request at a time.
  SingleConsumerProcessor<std::monostate> fetch_diagnostics_processor_;

  static constexpr auto msg_not_supported = "Not supported";
  static constexpr auto msg_error = "Error";
  static constexpr auto msg_separator = ": ";
};
}  // namespace nebula::ros
