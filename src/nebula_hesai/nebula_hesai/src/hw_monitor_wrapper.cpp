// Copyright 2024 TIER IV, Inc.

#include "nebula_hesai/hw_monitor_wrapper.hpp"

#include "nebula_core_ros/parameter_descriptors.hpp"
#include "nebula_core_ros/sync_tooling/sync_tooling_worker.hpp"
#include "nebula_hesai_hw_interfaces/hesai_cmd_response.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cassert>
#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebula::ros
{

using nlohmann::json;

void HesaiHwMonitorWrapper::add_json_item_to_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics, const std::string & key,
  const json & value)
{
  if (key.find("reserved") != std::string::npos) return;

  switch (value.type()) {
    case nlohmann::detail::value_t::string:
      diagnostics.add(key, value.template get<std::string>());
      break;
    default:
      diagnostics.add(key, value.dump());
  }
}

HesaiHwMonitorWrapper::HesaiHwMonitorWrapper(
  rclcpp::Node * const parent_node, diagnostic_updater::Updater & diagnostic_updater,
  const std::shared_ptr<nebula::drivers::HesaiHwInterface> & hw_interface,
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config,
  const std::shared_ptr<SyncToolingWorker> & sync_tooling_worker)
: logger_(parent_node->get_logger().get_child("HwMonitor")),
  status_(Status::OK),
  hw_interface_(hw_interface),
  parent_node_(parent_node),
  monitor_enabled_(drivers::supports_lidar_monitor(config->sensor_model)),
  sync_tooling_worker_(sync_tooling_worker),
  fetch_diagnostics_processor_([this](std::monostate) { fetch_diagnostics_from_sensor(); }, 1)
{
  diag_span_ms_ = parent_node->declare_parameter<uint16_t>("diag_span", param_read_only());
  initialize_hesai_diagnostics(diagnostic_updater);
}

void HesaiHwMonitorWrapper::initialize_hesai_diagnostics(
  diagnostic_updater::Updater & diagnostic_updater)
{
  using std::chrono_literals::operator""s;
  std::ostringstream os;
  diagnostic_updater.add("hesai_status", this, &HesaiHwMonitorWrapper::hesai_check_status);
  diagnostic_updater.add("hesai_ptp", this, &HesaiHwMonitorWrapper::hesai_check_ptp);
  diagnostic_updater.add(
    "hesai_temperature", this, &HesaiHwMonitorWrapper::hesai_check_temperature);
  diagnostic_updater.add("hesai_rpm", this, &HesaiHwMonitorWrapper::hesai_check_rpm);

  current_status_.reset();
  current_status_time_ = std::make_unique<rclcpp::Time>(parent_node_->get_clock()->now());
  current_diag_status_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;

  current_monitor_.reset();
  current_lidar_monitor_time_ = std::make_unique<rclcpp::Time>(parent_node_->get_clock()->now());
  current_monitor_status_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;

  auto fetch_diag_from_sensor = [this]() {
    // One request can be processed at a time. In cases where the previous request is not yet
    // processed, we skip one timer iteration (i.e. try_push returns false as the queue is still
    // full)
    fetch_diagnostics_processor_.try_push({});
  };

  fetch_diagnostics_timer_ = parent_node_->create_wall_timer(
    std::chrono::milliseconds(diag_span_ms_), std::move(fetch_diag_from_sensor));

  if (monitor_enabled_) {
    if (hw_interface_->use_http_get_lidar_monitor()) {
      diagnostic_updater.add(
        "hesai_voltage", this, &HesaiHwMonitorWrapper::hesai_check_voltage_http);
    } else {
      diagnostic_updater.add("hesai_voltage", this, &HesaiHwMonitorWrapper::hesai_check_voltage);
    }
  }
}

void HesaiHwMonitorWrapper::fetch_diagnostics_from_sensor()
{
  fetch_status();

  if (monitor_enabled_) {
    hw_interface_->use_http_get_lidar_monitor() ? fetch_monitor_http() : fetch_monitor_tcp();
  }

  if (sync_tooling_worker_) {
    fetch_sync_diag();
  }
}

std::string HesaiHwMonitorWrapper::get_ptree_value(
  boost::property_tree::ptree * pt, const std::string & key)
{
  boost::optional<std::string> value = pt->get_optional<std::string>(key);
  if (value) {
    return value.get();
  }

  return msg_not_supported;
}
std::string HesaiHwMonitorWrapper::get_fixed_precision_string(double val, int pre)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(pre) << val;
  return ss.str();
}

void HesaiHwMonitorWrapper::fetch_status()
{
  RCLCPP_DEBUG(logger_, "on_hesai_status_timer");
  try {
    auto result = hw_interface_->get_lidar_status();
    submit_clock_state(*result);
    std::scoped_lock lock(mtx_lidar_status_);
    current_status_time_ = std::make_unique<rclcpp::Time>(parent_node_->get_clock()->now());
    current_status_ = result;
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("HesaiHwMonitorWrapper::on_hesai_status_timer(std::system_error)"),
      error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorWrapper::on_hesai_status_timer(boost::system::system_error)"),
      error.what());
  } catch (const std::runtime_error & error) {
    RCLCPP_ERROR_STREAM(
      logger_, "Failed to get lidar status (timeout or communication error): " << error.what());
  }
  RCLCPP_DEBUG(logger_, "on_hesai_status_timer END");
}

void HesaiHwMonitorWrapper::submit_clock_state(const HesaiLidarStatusBase & status)
{
  if (!sync_tooling_worker_) return;

  auto j = status.to_json();
  if (j.contains("ptp_status")) {
    auto status = j["ptp_status"].template get<std::string>();
    if (status == "locked") {
      sync_tooling_worker_->submit_self_reported_clock_state(SelfReportedClockStateUpdate::LOCKED);
    } else if (status == "tracking") {
      sync_tooling_worker_->submit_self_reported_clock_state(
        SelfReportedClockStateUpdate::TRACKING);
    } else if (status == "free run") {
      sync_tooling_worker_->submit_self_reported_clock_state(
        SelfReportedClockStateUpdate::UNSYNCHRONIZED);
    } else if (status == "frozen") {
      sync_tooling_worker_->submit_self_reported_clock_state(SelfReportedClockStateUpdate::LOST);
    } else {
      sync_tooling_worker_->submit_self_reported_clock_state(SelfReportedClockStateUpdate::INVALID);
    }
  }
}

void HesaiHwMonitorWrapper::fetch_monitor_http()
{
  RCLCPP_DEBUG(logger_, "on_hesai_lidar_monitor_timer_http");
  try {
    hw_interface_->get_lidar_monitor_async_http([this](const std::string & str) {
      std::scoped_lock lock(mtx_lidar_monitor_);
      current_lidar_monitor_time_ =
        std::make_unique<rclcpp::Time>(parent_node_->get_clock()->now());
      current_lidar_monitor_tree_ =
        std::make_unique<boost::property_tree::ptree>(hw_interface_->parse_json(str));
    });
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer_http(std::system_error)"),
      error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer_http(boost::system::system_"
        "error)"),
      error.what());
  } catch (const std::runtime_error & error) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Failed to get lidar monitor via HTTP (timeout or communication error): " << error.what());
  }
  RCLCPP_DEBUG(logger_, "on_hesai_lidar_monitor_timer_http END");
}

void HesaiHwMonitorWrapper::fetch_monitor_tcp()
{
  RCLCPP_DEBUG(logger_, "on_hesai_lidar_monitor_timer");
  try {
    auto result = hw_interface_->get_lidar_monitor();
    std::scoped_lock lock(mtx_lidar_monitor_);
    current_lidar_monitor_time_ = std::make_unique<rclcpp::Time>(parent_node_->get_clock()->now());
    current_monitor_ = std::make_shared<HesaiLidarMonitor>(result);
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer(std::system_error)"),
      error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer(boost::system::system_"
        "error)"),
      error.what());
  } catch (const std::runtime_error & error) {
    RCLCPP_ERROR_STREAM(
      logger_, "Failed to get lidar monitor (timeout or communication error): " << error.what());
  }
  RCLCPP_DEBUG(logger_, "on_hesai_lidar_monitor_timer END");
}

void HesaiHwMonitorWrapper::fetch_sync_diag()
{
  if (!sync_tooling_worker_) return;

  try {
    auto port_ds = hw_interface_->get_ptp_diag_port();
    auto clock_id = make_ptp_clock_id(port_ds.portIdentity.clock_id.to_json());
    sync_tooling_worker_->submit_port_state_update(
      clock_id, port_ds.portIdentity.port_number.value(), port_ds.portState);

    sync_tooling_worker_->submit_clock_alias(
      port_ds.portIdentity.clock_id.to_json().template get<std::string>());
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Could not get port dataset from sensor: " << e.what());
  }

  try {
    auto time_status_np = hw_interface_->get_ptp_diag_time();
    std::optional<std::string> master_clock_id{};
    if (time_status_np.gmPresent.value()) {
      master_clock_id.emplace(time_status_np.gmIdentity.to_json().template get<std::string>());
    }
    sync_tooling_worker_->submit_master_update(master_clock_id);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Could not get time status dataset from sensor: " << e.what());
  }
}

void HesaiHwMonitorWrapper::hesai_check_status(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_status_);

  if (!current_status_ || !current_status_time_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "");
    return;
  }

  json data = current_status_->to_json();
  for (const auto & [key, value] : data.items()) {
    if (
      key == "motor_speed" || key == "temperature" ||
      (key.find("ptp") != std::string::npos || key.find("gps") != std::string::npos)) {
      continue;
    }

    add_json_item_to_diagnostics(diagnostics, key, value);
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg;
  if (is_stale(*current_status_time_)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    msg = "[STALE]";
  }

  diagnostics.summary(level, msg);
}

void HesaiHwMonitorWrapper::hesai_check_ptp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_status_);

  if (!current_status_ || !current_status_time_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "");
    return;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  std::string msg = "not synchronized";

  json data = current_status_->to_json();
  for (const auto & [key, value] : data.items()) {
    if (key.find("ptp") == std::string::npos && key.find("gps") == std::string::npos) {
      continue;
    }

    if (value.type() == json::value_t::string) {
      auto str = value.template get<std::string>();
      if (str == "locked") {
        level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        msg = "synchronized";
      } else if (str == "tracking") {
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        msg = "synchronized, degraded";
      }
    }

    add_json_item_to_diagnostics(diagnostics, key, value);
  }

  if (is_stale(*current_status_time_)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    msg = "[STALE] " + msg;
  }

  diagnostics.summary(level, msg);
}

void HesaiHwMonitorWrapper::hesai_check_temperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_status_);

  if (!current_status_ || !current_status_time_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "");
    return;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  json data = current_status_->to_json();
  if (data.contains("temperature")) {
    for (const auto & [key, value] : data["temperature"].items()) {
      add_json_item_to_diagnostics(diagnostics, key, value);
    }
  }

  std::string msg;
  if (is_stale(*current_status_time_)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    msg = "[STALE]";
  }

  diagnostics.summary(level, msg);
}

void HesaiHwMonitorWrapper::hesai_check_rpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_status_);

  if (!current_status_ || !current_status_time_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "");
    return;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  json data = current_status_->to_json();
  if (data.contains("motor_speed")) {
    add_json_item_to_diagnostics(diagnostics, "motor_speed", data["motor_speed"]);
  }

  std::string msg;
  if (is_stale(*current_status_time_)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    msg = "[STALE]";
  }

  diagnostics.summary(level, msg);
}

void HesaiHwMonitorWrapper::hesai_check_voltage_http(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_monitor_);

  if (!current_lidar_monitor_tree_ || !current_lidar_monitor_time_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "");
    return;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string key;

  std::string mes;
  key = "lidarInCur";
  try {
    mes = get_ptree_value(current_lidar_monitor_tree_.get(), "Body." + key);
  } catch (boost::bad_lexical_cast & ex) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = std::string(msg_error) + msg_separator + std::string(ex.what());
  }
  add_json_item_to_diagnostics(diagnostics, key, mes);
  key = "lidarInVol";
  try {
    mes = get_ptree_value(current_lidar_monitor_tree_.get(), "Body." + key);
  } catch (boost::bad_lexical_cast & ex) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = std::string(msg_error) + msg_separator + std::string(ex.what());
  }
  add_json_item_to_diagnostics(diagnostics, key, mes);

  std::string msg;
  if (is_stale(*current_lidar_monitor_time_)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    msg = "[STALE]";
  }

  diagnostics.summary(level, msg);
}

void HesaiHwMonitorWrapper::hesai_check_voltage(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_monitor_);

  if (!current_monitor_ || !current_lidar_monitor_time_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "");
    return;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  json data = current_monitor_->to_json();
  for (const auto & [key, value] : data.items()) {
    add_json_item_to_diagnostics(diagnostics, key, value);
  }

  std::string msg;
  if (is_stale(*current_lidar_monitor_time_)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    msg = "[STALE]";
  }

  diagnostics.summary(level, msg);
}

[[nodiscard]] bool HesaiHwMonitorWrapper::is_stale(const rclcpp::Time & last_update) const
{
  assert(parent_node_);
  auto diag_span = std::chrono::milliseconds(diag_span_ms_);
  return last_update < parent_node_->now() - rclcpp::Duration(diag_span);
}

Status HesaiHwMonitorWrapper::status()
{
  return Status::OK;
}
}  // namespace nebula::ros
