// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/robosense/hw_monitor_wrapper.hpp"

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>

namespace nebula::ros
{
RobosenseHwMonitorWrapper::RobosenseHwMonitorWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & config)
: parent_(parent_node),
  logger_(parent_->get_logger().get_child("HwMonitorWrapper")),
  diagnostics_updater_(),
  status_(nebula::Status::NOT_INITIALIZED),
  sensor_cfg_ptr_(config)
{
  auto descriptor = param_read_only().set__additional_constraints("milliseconds");
  diag_span_ = parent_->declare_parameter<uint16_t>("diag_span", descriptor);
}

void RobosenseHwMonitorWrapper::initialize_robosense_diagnostics()
{
  std::scoped_lock lock(mtx_config_, mtx_current_sensor_info_);

  if (current_sensor_info_.empty()) {
    RCLCPP_WARN(logger_, "Tried to initialize diagnostics updater without any diagnostics data");
    return;
  }

  auto hw_id = nebula::drivers::sensor_model_to_string(sensor_cfg_ptr_->sensor_model) + ": " +
               current_sensor_info_["serial_number"];

  RCLCPP_INFO(logger_, "InitializeRobosenseDiagnostics");
  diagnostics_updater_.emplace(parent_);
  diagnostics_updater_->setHardwareID(hw_id);
  RCLCPP_INFO_STREAM(logger_, "hardware_id: " + hw_id);

  diagnostics_updater_->add(
    "robosense_status", this, &RobosenseHwMonitorWrapper::robosense_check_status);

  auto on_timer_update = [this] {
    RCLCPP_DEBUG(logger_, "OnUpdateTimer");
    auto now = parent_->get_clock()->now();
    auto dif = (now - current_info_time_).seconds();

    RCLCPP_DEBUG_STREAM(logger_, "dif(status): " << dif);

    if (diag_span_ * 2.0 < dif * 1000) {
      RCLCPP_WARN(logger_, "STALE");
    } else {
      RCLCPP_DEBUG(logger_, "OK");
    }
    diagnostics_updater_->force_update();
  };

  diagnostics_update_timer_ =
    parent_->create_wall_timer(std::chrono::milliseconds(1000), std::move(on_timer_update));

  RCLCPP_DEBUG_STREAM(logger_, "add_timer");
}

void RobosenseHwMonitorWrapper::robosense_check_status(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::lock_guard lock(mtx_current_sensor_info_);
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  for (const auto & info : current_sensor_info_) {
    diagnostics.add(info.first, info.second);
  }

  diagnostics.summary(level, "OK");
}

void RobosenseHwMonitorWrapper::diagnostics_callback(
  const std::map<std::string, std::string> & diag_info)
{
  auto current_time = parent_->get_clock()->now();

  {
    std::lock_guard lock(mtx_current_sensor_info_);
    current_sensor_info_ = diag_info;
    current_info_time_ = current_time;
  }

  if (!diagnostics_updater_) {
    initialize_robosense_diagnostics();
  }
}

void RobosenseHwMonitorWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_config_);

  if (!new_config) {
    throw std::invalid_argument("Config is not nullable");
  }

  if (new_config->sensor_model != sensor_cfg_ptr_->sensor_model) {
    throw std::invalid_argument("Sensor model is read-only during runtime");
  }

  sensor_cfg_ptr_ = new_config;
}

}  // namespace nebula::ros
