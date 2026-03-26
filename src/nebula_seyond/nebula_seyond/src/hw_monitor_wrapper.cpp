// Copyright 2026 TIER IV, Inc.

#include "nebula_seyond/hw_monitor_wrapper.hpp"

#include "nebula_core_ros/parameter_descriptors.hpp"

#include <rclcpp/duration.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace nebula::ros
{

SeyondHwMonitorWrapper::SeyondHwMonitorWrapper(
  rclcpp::Node * const parent_node, diagnostic_updater::Updater & diagnostic_updater,
  const std::shared_ptr<nebula::drivers::SeyondHwInterface> & hw_interface,
  const std::shared_ptr<const nebula::drivers::SeyondSensorConfiguration> & config)
: logger_(parent_node->get_logger().get_child("HwMonitor")),
  status_(Status::OK),
  hw_interface_(hw_interface),
  config_(config),
  parent_node_(parent_node),
  last_update_time_(std::make_unique<rclcpp::Time>(parent_node->get_clock()->now()))
{
  diag_span_ms_ = parent_node->declare_parameter<uint16_t>("diag_span", param_read_only());
  initialize_diagnostics(diagnostic_updater);
}

void SeyondHwMonitorWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::SeyondSensorConfiguration> & new_config)
{
  std::scoped_lock lock(mutex_);
  config_ = new_config;
}

nebula::Status SeyondHwMonitorWrapper::status() const
{
  return status_;
}

void SeyondHwMonitorWrapper::initialize_diagnostics(
  diagnostic_updater::Updater & diagnostic_updater)
{
  diagnostic_updater.add("seyond_connection", this, &SeyondHwMonitorWrapper::check_connection);
  diagnostic_updater.add(
    "seyond_configuration", this, &SeyondHwMonitorWrapper::check_configuration);

  auto fetch = [this, &diagnostic_updater]() {
    fetch_diagnostics(diagnostic_updater);
    diagnostic_updater.force_update();
  };

  fetch_diagnostics_timer_ =
    parent_node_->create_wall_timer(std::chrono::milliseconds(diag_span_ms_), std::move(fetch));
}

void SeyondHwMonitorWrapper::fetch_diagnostics(diagnostic_updater::Updater & diagnostic_updater)
{
  (void)diagnostic_updater;

  MonitorSnapshot snapshot;
  std::string value;
  bool any_success = false;
  std::string first_error;

  auto fetch_attribute = [this, &value, &any_success, &first_error](
                           const std::string & name, std::string & target) {
    value.clear();
    auto status = hw_interface_->get_attribute(name, value);
    if (status == Status::OK) {
      target = value;
      any_success = true;
      return;
    }
    if (first_error.empty()) {
      first_error = name + " fetch failed";
    }
  };

  fetch_attribute("udp_ports_ip", snapshot.udp_ports_ip);
  fetch_attribute("return_mode", snapshot.return_mode);
  fetch_attribute("reflectance_mode", snapshot.reflectance_mode);
  fetch_attribute("frame_rate", snapshot.frame_rate);
  fetch_attribute("v_angle_offset", snapshot.v_angle_offset);

  std::scoped_lock lock(mutex_);
  if (any_success) {
    snapshot_ = std::move(snapshot);
    last_error_.clear();
    last_update_time_ = std::make_unique<rclcpp::Time>(parent_node_->get_clock()->now());
    status_ = Status::OK;
  } else {
    last_error_ = first_error.empty() ? std::string{"no diagnostics available"} : first_error;
    status_ = Status::HTTP_CONNECTION_ERROR;
  }
}

void SeyondHwMonitorWrapper::check_connection(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mutex_);

  if (!last_update_time_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "no diagnostics yet");
    return;
  }

  if (status_ != Status::OK) {
    diagnostics.add("error", last_error_);
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "sensor unreachable");
    return;
  }

  diagnostics.add("sensor_ip", config_ ? config_->connection.sensor_ip : std::string{});
  diagnostics.add("udp_ports_ip", snapshot_.udp_ports_ip);

  if (is_stale(*last_update_time_)) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "diagnostics stale");
    return;
  }

  diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "sensor reachable");
}

void SeyondHwMonitorWrapper::check_configuration(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mutex_);

  if (!config_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "missing configuration");
    return;
  }

  diagnostics.add("target_sensor_ip", config_->connection.sensor_ip);
  diagnostics.add("target_host_ip", config_->connection.host_ip);
  diagnostics.add("target_udp_port", std::to_string(config_->connection.udp_port));
  diagnostics.add("target_udp_message_port", std::to_string(config_->connection.udp_message_port));
  diagnostics.add("target_udp_status_port", std::to_string(config_->connection.udp_status_port));
  diagnostics.add("current_udp_ports_ip", snapshot_.udp_ports_ip);
  diagnostics.add("current_return_mode", snapshot_.return_mode);
  diagnostics.add("current_reflectance_mode", snapshot_.reflectance_mode);
  diagnostics.add("current_frame_rate", snapshot_.frame_rate);
  diagnostics.add("current_v_angle_offset", snapshot_.v_angle_offset);

  if (status_ != Status::OK) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "using cached data");
    return;
  }

  diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "configuration fetched");
}

bool SeyondHwMonitorWrapper::is_stale(const rclcpp::Time & stamp) const
{
  const auto elapsed = parent_node_->get_clock()->now() - stamp;
  return elapsed > rclcpp::Duration::from_seconds((2.0 * diag_span_ms_) / 1000.0);
}

}  // namespace nebula::ros
