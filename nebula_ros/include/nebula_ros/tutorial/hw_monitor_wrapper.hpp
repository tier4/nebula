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
#include <nebula_common/tutorial/tutorial_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_tutorial/tutorial_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>

namespace nebula::ros
{

class TutorialHwMonitorWrapper
{
public:
  TutorialHwMonitorWrapper(
    rclcpp::Node * const parent_node,
    const std::shared_ptr<nebula::drivers::TutorialHwInterface> & hw_interface,
    std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & config);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & /* new_config */)
  {
    // Update the relevant parameters in all sub-modules and re-instantiate them if necessary
  }

  nebula::Status status();

private:
  void initialize_diagnostics();

  /// @brief Parse the stored diagnostic info and add it to the diagnostic status
  void parse_and_add_diagnostic_info(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  /// @brief Check if stored diagnostic info is up-to-date and force diagnostic updater update
  void trigger_diagnostics_update();

  /// @brief Get and store diagnostic info from sensor
  void fetch_diagnostic_info();

  rclcpp::Node * const parent_node_;
  rclcpp::Logger logger_;
  nebula::Status status_;
  uint16_t diag_span_;
  diagnostic_updater::Updater diagnostics_updater_;

  const std::shared_ptr<nebula::drivers::TutorialHwInterface> hw_interface_;

  rclcpp::TimerBase::SharedPtr diagnostics_update_timer_;
  rclcpp::TimerBase::SharedPtr fetch_diagnostics_timer_;

  std::map<std::string, std::string> current_diag_info_;
  std::optional<rclcpp::Time> current_diag_info_time_;
  std::mutex mtx_current_diag_info_;
};
}  // namespace nebula::ros
