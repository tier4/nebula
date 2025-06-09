// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/hesai/hw_monitor_wrapper.hpp"

#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp"
#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/common/sync_diag_client.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>

#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.hpp>

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
  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config)
: logger_(parent_node->get_logger().get_child("HwMonitor")),
  status_(Status::OK),
  hw_interface_(hw_interface),
  parent_node_(parent_node)
{
  diag_span_ = parent_node->declare_parameter<uint16_t>("diag_span", param_read_only());

  bool monitor_enabled = config->sensor_model != drivers::SensorModel::HESAI_PANDARAT128 &&
                         config->sensor_model != drivers::SensorModel::HESAI_PANDAR40P &&
                         config->sensor_model != drivers::SensorModel::HESAI_PANDAR64;

  initialize_hesai_diagnostics(diagnostic_updater, monitor_enabled);

  if (config->sync_diagnostics_topic) {
    sync_diag_client_.emplace(
      parent_node_, *config->sync_diagnostics_topic, config->frame_id, config->ptp_domain);
  }
}

void HesaiHwMonitorWrapper::initialize_hesai_diagnostics(
  diagnostic_updater::Updater & diagnostic_updater, bool monitor_enabled)
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

  auto fetch_diag_from_sensor = [this, monitor_enabled]() {
    on_hesai_status_timer();

    if (monitor_enabled) {
      hw_interface_->use_http_get_lidar_monitor() ? on_hesai_lidar_monitor_timer_http()
                                                  : on_hesai_lidar_monitor_timer();
    }

    if (sync_diag_client_) {
      on_sync_diag_timer();
    }
  };

  fetch_diagnostics_timer_ = parent_node_->create_wall_timer(
    std::chrono::milliseconds(diag_span_), std::move(fetch_diag_from_sensor));

  if (monitor_enabled) {
    if (hw_interface_->use_http_get_lidar_monitor()) {
      diagnostic_updater.add(
        "hesai_voltage", this, &HesaiHwMonitorWrapper::hesai_check_voltage_http);
    } else {
      diagnostic_updater.add("hesai_voltage", this, &HesaiHwMonitorWrapper::hesai_check_voltage);
    }
  }
}

std::string HesaiHwMonitorWrapper::get_ptree_value(
  boost::property_tree::ptree * pt, const std::string & key)
{
  boost::optional<std::string> value = pt->get_optional<std::string>(key);
  if (value) {
    return value.get();
  } else {
    return MSG_NOT_SUPPORTED_;
  }
}
std::string HesaiHwMonitorWrapper::get_fixed_precision_string(double val, int pre)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(pre) << val;
  return ss.str();
}

void HesaiHwMonitorWrapper::on_hesai_status_timer()
{
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_status_timer" << std::endl);
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
  }
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_status_timer END" << std::endl);
}

void HesaiHwMonitorWrapper::submit_clock_state(const HesaiLidarStatusBase & status)
{
  if (!sync_diag_client_) return;

  auto j = status.to_json();
  if (j.contains("ptp_status")) {
    auto status = j["ptp_status"].template get<std::string>();
    if (status == "locked") {
      sync_diag_client_->submit_self_reported_clock_state(SelfReportedClockStateUpdate::LOCKED);
    } else if (status == "tracking") {
      sync_diag_client_->submit_self_reported_clock_state(SelfReportedClockStateUpdate::TRACKING);
    } else if (status == "free run") {
      sync_diag_client_->submit_self_reported_clock_state(
        SelfReportedClockStateUpdate::UNSYNCHRONIZED);
    } else if (status == "frozen") {
      sync_diag_client_->submit_self_reported_clock_state(SelfReportedClockStateUpdate::LOST);
    } else {
      sync_diag_client_->submit_self_reported_clock_state(SelfReportedClockStateUpdate::INVALID);
    }
  }
}

void HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer_http()
{
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_lidar_monitor_timer_http");
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
  }
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_lidar_monitor_timer_http END");
}

void HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer()
{
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_lidar_monitor_timer");
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
  }
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_lidar_monitor_timer END");
}

void HesaiHwMonitorWrapper::on_sync_diag_timer()
{
  if (!sync_diag_client_) return;

  try {
    auto port_ds = hw_interface_->get_ptp_diag_port();
    auto clock_id = make_ptp_clock_id(port_ds.portIdentity.clock_id.to_json());
    sync_diag_client_->submit_port_state_update(
      clock_id, port_ds.portIdentity.port_number.value(), port_ds.portState);

    sync_diag_client_->submit_clock_alias(
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
    sync_diag_client_->submit_master_update(master_clock_id);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Could not get time status dataset from sensor: " << e.what());
  }
}

void HesaiHwMonitorWrapper::hesai_check_status(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_status_);
  if (current_status_) {
    json data = current_status_->to_json();
    for (const auto & [key, value] : data.items()) {
      if (
        key == "motor_speed" || key == "temperature" ||
        (key.find("ptp") != std::string::npos || key.find("gps") != std::string::npos)) {
        continue;
      }

      add_json_item_to_diagnostics(diagnostics, key, value);
    }
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_ptp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  std::string msg = "not synchronized";
  std::scoped_lock lock(mtx_lidar_status_);
  if (current_status_) {
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
    diagnostics.summary(level, msg);
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_temperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  std::scoped_lock lock(mtx_lidar_status_);
  if (current_status_) {
    json data = current_status_->to_json();
    if (data.contains("temperature")) {
      for (const auto & [key, value] : data["temperature"].items()) {
        add_json_item_to_diagnostics(diagnostics, key, value);
      }
    }
    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_rpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_status_);
  if (current_status_) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    json data = current_status_->to_json();
    if (data.contains("motor_speed")) {
      add_json_item_to_diagnostics(diagnostics, "motor_speed", data["motor_speed"]);
    }
    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_voltage_http(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_monitor_);
  if (current_lidar_monitor_tree_) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    std::string key = "";

    std::string mes;
    key = "lidarInCur";
    try {
      mes = get_ptree_value(current_lidar_monitor_tree_.get(), "Body." + key);
    } catch (boost::bad_lexical_cast & ex) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      mes = MSG_ERROR_ + std::string(ex.what());
    }
    add_json_item_to_diagnostics(diagnostics, key, mes);
    key = "lidarInVol";
    try {
      mes = get_ptree_value(current_lidar_monitor_tree_.get(), "Body." + key);
    } catch (boost::bad_lexical_cast & ex) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      mes = MSG_ERROR_ + std::string(ex.what());
    }
    add_json_item_to_diagnostics(diagnostics, key, mes);

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_voltage(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_monitor_);
  if (current_monitor_) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    json data = current_monitor_->to_json();
    for (const auto & [key, value] : data.items()) {
      add_json_item_to_diagnostics(diagnostics, key, value);
    }

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

Status HesaiHwMonitorWrapper::status()
{
  return Status::OK;
}
}  // namespace nebula::ros
