// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/velodyne/hw_monitor_wrapper.hpp"

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace nebula::ros
{
VelodyneHwMonitorWrapper::VelodyneHwMonitorWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<nebula::drivers::VelodyneHwInterface> & hw_interface,
  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config)
: logger_(parent_node->get_logger().get_child("HwMonitor")),
  diagnostics_updater_(
    (parent_node->declare_parameter<bool>("diagnostic_updater.use_fqn", true), parent_node)),
  hw_interface_(hw_interface),
  parent_node_(parent_node),
  sensor_configuration_(config)
{
  diag_span_ = parent_node->declare_parameter<uint16_t>("diag_span", param_read_only());
  show_advanced_diagnostics_ =
    parent_node->declare_parameter<bool>("advanced_diagnostics", param_read_only());

  std::cout << "Get model name and serial." << std::endl;
  auto str = hw_interface_->get_snapshot();
  if (!str.has_value()) return;

  auto snapshot_tree =
    std::make_shared<boost::property_tree::ptree>(hw_interface_->parse_json(str.value()));
  current_diag_tree_ =
    std::make_shared<boost::property_tree::ptree>(snapshot_tree->get_child("diag"));
  current_status_tree_ =
    std::make_shared<boost::property_tree::ptree>(snapshot_tree->get_child("status"));

  try {
    // get_ptree_value requires a mutex but we are only accessing a local variable
    std::mutex dummy_mtx;
    info_model_ = get_ptree_value(snapshot_tree, dummy_mtx, key_info_model);
    info_serial_ = get_ptree_value(snapshot_tree, dummy_mtx, key_info_serial);
    RCLCPP_INFO_STREAM(logger_, "Model: " << info_model_);
    RCLCPP_INFO_STREAM(logger_, "Serial: " << info_serial_);
  } catch (boost::bad_lexical_cast & ex) {
    RCLCPP_ERROR(logger_, " Error: Can't get model and serial");
    return;
  }

  initialize_velodyne_diagnostics();
}

void VelodyneHwMonitorWrapper::initialize_velodyne_diagnostics()
{
  RCLCPP_INFO_STREAM(logger_, "InitializeVelodyneDiagnostics");
  using std::chrono_literals::operator""s;
  std::ostringstream os;
  auto hardware_id = info_model_ + ": " + info_serial_;
  diagnostics_updater_.setHardwareID(hardware_id);
  RCLCPP_INFO_STREAM(logger_, "Hardware ID: " << hardware_id);

  if (show_advanced_diagnostics_) {
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_hv-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_top_hv);
    if (sensor_configuration_->sensor_model != nebula::drivers::SensorModel::VELODYNE_VLP16) {
      diagnostics_updater_.add(
        "velodyne_volt_temp_top_ad_temp-" + sensor_configuration_->frame_id, this,
        &VelodyneHwMonitorWrapper::velodyne_check_top_ad_temp);
    }
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_lm20_temp-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_top_lm20_temp);
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_5v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_top_pwr5v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_2_5v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_top_pwr25v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_3_3v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_top_pwr33v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_raw-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_top_pwr_raw);
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_vccint-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_top_pwr_vccint);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_i_out-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_bot_i_out);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_1_2v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_bot_pwr12v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_lm20_temp-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_bot_lm20_temp);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_5v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_bot_pwr5v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_2_5v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_bot_pwr25v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_3_3v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_bot_pwr33v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_v_in-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_bot_pwr_v_in);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_1_25v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_bot_pwr125v);
    diagnostics_updater_.add(
      "velodyne_vhv-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_vhv);
    diagnostics_updater_.add(
      "velodyne_adc_nf-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_adc_nf);
    diagnostics_updater_.add(
      "velodyne_adc_stats-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_adc_stats);
    diagnostics_updater_.add(
      "velodyne_ixe-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_ixe);
    diagnostics_updater_.add(
      "velodyne_adctp_stat-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_adctp_stat);

    diagnostics_updater_.add(
      "velodyne_status_gps_pps_state-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_gps_pps_state);
    diagnostics_updater_.add(
      "velodyne_status_gps_pps_position-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_gps_position);
    diagnostics_updater_.add(
      "velodyne_status_motor_state-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_motor_state);
    diagnostics_updater_.add(
      "velodyne_status_motor_rpm-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_motor_rpm);
    diagnostics_updater_.add(
      "velodyne_status_motor_lock-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_motor_lock);
    diagnostics_updater_.add(
      "velodyne_status_motor_phase-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_motor_phase);
    diagnostics_updater_.add(
      "velodyne_status_laser_state-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::velodyne_check_laser_state);
  }

  diagnostics_updater_.add(
    "velodyne_status", this, &VelodyneHwMonitorWrapper::velodyne_check_status);
  diagnostics_updater_.add("velodyne_pps", this, &VelodyneHwMonitorWrapper::velodyne_check_pps);
  diagnostics_updater_.add(
    "velodyne_temperature", this, &VelodyneHwMonitorWrapper::velodyne_check_temperature);
  diagnostics_updater_.add("velodyne_rpm", this, &VelodyneHwMonitorWrapper::velodyne_check_rpm);
  diagnostics_updater_.add(
    "velodyne_voltage", this, &VelodyneHwMonitorWrapper::velodyne_check_voltage);

  diagnostics_updater_.setPeriod(1.0);
}

void VelodyneHwMonitorWrapper::on_velodyne_diagnostics_timer()
{
  std::cout << "OnVelodyneDiagnosticsTimer" << std::endl;
  auto str = hw_interface_->get_diag();
  if (!str.has_value()) return;

  {
    std::lock_guard lock(mtx_diag_);
    current_diag_tree_ =
      std::make_shared<boost::property_tree::ptree>(hw_interface_->parse_json(str.value()));
  }
  diagnostics_updater_.force_update();
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::velodyne_get_top_hv()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_top_hv));
    val = 101.0 * (val * 5.0 / 4096.0 - 5.0);
    if (val < -150.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_hv + message_sep_ + voltage_low_message;
    } else if (-132.0 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_hv + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_top_ad_temp()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_top_ad_temp));
    val = val * 5.0 / 4096.0;
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_top_lm20_temp()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_top_lm20_temp));
    val = -1481.96 + std::sqrt(2.1962e6 + ((1.8639 - val * 5.0 / 4096.0) / 3.88e-6));
    if (val < -25.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_lm20_temp + message_sep_ + temperature_cold_message;
    } else if (90.0 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_lm20_temp + message_sep_ + temperature_hot_message;
    }
    //    mes = boost::lexical_cast<std::string>(val) + " C";
    mes = get_fixed_precision_string(val) + " C";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_top_pwr5v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_top_pwr_5v));
    val = 2.0 * val * 5.0 / 4096.0;
    if (val < 4.8) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v + message_sep_ + voltage_low_message;
    } else if (5.2 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_top_pwr25v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_top_pwr_2_5v));
    val = val * 5.0 / 4096.0;
    if (val < 2.3) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_2_5v + message_sep_ + voltage_low_message;
    } else if (2.7 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_2_5v + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_top_pwr33v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_top_pwr_3_3v));
    val = val * 5.0 / 4096.0;
    if (val < 3.1) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_3_3v + message_sep_ + voltage_low_message;
    } else if (3.5 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_3_3v + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_top_pwr5v_raw()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_top_pwr_5v_raw));
    val = 2.0 * val * 5.0 / 4096.0;
    if (val < 2.3) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v_raw + message_sep_ + voltage_low_message;
    } else if (2.7 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v_raw + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_top_pwr_raw()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_top_pwr_raw));
    val = val * 5.0 / 4096.0;
    if (val < 1.6) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_raw + message_sep_ + voltage_low_message;
    } else if (1.9 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_raw + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_top_pwr_vccint()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_top_pwr_vccint));
    val = val * 5.0 / 4096.0;
    if (val < 1.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_vccint + message_sep_ + voltage_low_message;
    } else if (1.4 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_vccint + message_sep_ + voltage_high_message;
    }
    //    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_bot_i_out()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_bot_i_out));
    val = 10.0 * (val * 5.0 / 4096.0 - 2.5);
    if (val < 0.3) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_i_out + message_sep_ + ampere_low_message;
    } else if (1.0 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_i_out + message_sep_ + ampere_high_message;
    }
    mes = get_fixed_precision_string(val) + " A";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_bot_pwr12v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_bot_pwr_1_2v));
    val = val * 5.0 / 4096.0;
    if (val < 1.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_2v + message_sep_ + voltage_low_message;
    } else if (1.4 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_2v + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_bot_lm20_temp()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_bot_lm20_temp));
    val = -1481.96 + std::sqrt(2.1962e6 + ((1.8639 - val * 5.0 / 4096.0) / 3.88e-6));
    if (val < -25.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_lm20_temp + message_sep_ + temperature_cold_message;
    } else if (90.0 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_lm20_temp + message_sep_ + temperature_hot_message;
    }
    //    mes = boost::lexical_cast<std::string>(val) + " C";
    mes = get_fixed_precision_string(val) + " C";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_bot_pwr5v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_bot_pwr_5v));
    val = 2.0 * val * 5.0 / 4096.0;
    if (val < 4.8) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_5v + message_sep_ + voltage_low_message;
    } else if (5.2 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_5v + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_bot_pwr25v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_bot_pwr_2_5v));
    val = val * 5.0 / 4096.0;
    if (val < 2.3) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_2_5v + message_sep_ + voltage_low_message;
    } else if (2.7 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_2_5v + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_bot_pwr33v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_bot_pwr_3_3v));
    val = val * 5.0 / 4096.0;
    if (val < 3.1) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_3_3v + message_sep_ + voltage_low_message;
    } else if (3.5 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_3_3v + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_bot_pwr_v_in()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_bot_pwr_v_in));
    val = 11.0 * val * 5.0 / 4096.0;
    if (val < 8.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_v_in + message_sep_ + voltage_low_message;
    } else if (19.0 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_v_in + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_bot_pwr125v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      get_ptree_value(current_diag_tree_, mtx_diag_, key_volt_temp_bot_pwr_1_25v));
    val = val * 5.0 / 4096.0;
    if (val < 1.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_25v + message_sep_ + voltage_low_message;
    } else if (1.4 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_25v + message_sep_ + voltage_high_message;
    }
    mes = get_fixed_precision_string(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::velodyne_get_vhv()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(get_ptree_value(current_diag_tree_, mtx_diag_, key_vhv));
    mes = boost::lexical_cast<std::string>(val);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::velodyne_get_adc_nf()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    std::ostringstream os;
    boost::optional<boost::property_tree::ptree &> child =
      current_diag_tree_->get_child_optional(key_adc_nf);
    if (child) {
      std::ostringstream os;
      for (auto v = child->begin(); v != child->end(); ++v) {
        os << v->second.get<std::string>("") << ", ";
      }
      mes = os.str();
    } else {
      mes = not_supported_message;
    }
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_adc_stats()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    std::ostringstream os;
    boost::optional<boost::property_tree::ptree &> child =
      current_diag_tree_->get_child_optional(key_adc_stats);
    if (child) {
      std::ostringstream os;
      for (auto v = child->begin(); v != child->end(); ++v) {
        os << "(";
        os << "mean: " << v->second.get<std::string>("mean") << ", ";
        os << "stddev: " << v->second.get<std::string>("stddev") << ", ";
        os << "), ";
      }
      mes = os.str();
    } else {
      mes = not_supported_message;
    }
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::velodyne_get_ixe()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = get_ptree_value(current_diag_tree_, mtx_diag_, key_ixe);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_adctp_stat()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    std::ostringstream os;
    boost::optional<boost::property_tree::ptree &> child =
      current_diag_tree_->get_child_optional(key_adctp_stat);
    if (child) {
      std::ostringstream os;
      for (auto v = child->begin(); v != child->end(); ++v) {
        os << v->second.get<std::string>("") << ", ";
      }
      mes = os.str();
    } else {
      mes = not_supported_message;
    }
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_gps_pps_state()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = get_ptree_value(current_status_tree_, mtx_status_, key_status_gps_pps_state);
    if (mes == "Absent") {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = mes;
    } else if (mes == "Error") {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      error_mes = mes;
    }
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_gps_position()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = get_ptree_value(current_status_tree_, mtx_status_, key_status_gps_pps_position);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_motor_state()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = get_ptree_value(current_status_tree_, mtx_status_, key_status_motor_state);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_motor_rpm()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = get_ptree_value(current_status_tree_, mtx_status_, key_status_motor_rpm);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_motor_lock()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = get_ptree_value(current_status_tree_, mtx_status_, key_status_motor_lock);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_motor_phase()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = get_ptree_value(current_status_tree_, mtx_status_, key_status_motor_phase);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::velodyne_get_laser_state()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = get_ptree_value(current_status_tree_, mtx_status_, key_status_laser_state);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

void VelodyneHwMonitorWrapper::velodyne_check_top_hv(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_top_hv();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_top_ad_temp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_top_ad_temp();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_top_lm20_temp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_top_lm20_temp();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_top_pwr5v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_top_pwr5v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_top_pwr25v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_top_pwr25v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_top_pwr33v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_top_pwr33v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_top_pwr_raw(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_top_pwr_raw();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_top_pwr_vccint(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_top_pwr_vccint();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_bot_i_out(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_bot_i_out();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_bot_pwr12v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_bot_pwr12v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_bot_lm20_temp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_bot_lm20_temp();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_bot_pwr5v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_bot_pwr5v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_bot_pwr25v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_bot_pwr25v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_bot_pwr33v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_bot_pwr33v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_bot_pwr_v_in(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_bot_pwr_v_in();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_bot_pwr125v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_bot_pwr125v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_vhv(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_vhv();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_adc_nf(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_adc_nf();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_adc_stats(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_adc_stats();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_ixe(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_ixe();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_adctp_stat(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    auto tpl = velodyne_get_adctp_stat();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::on_velodyne_status_timer()
{
  auto str = hw_interface_->get_status();
  if (!str.has_value()) return;
  {
    std::lock_guard lock(mtx_status_);
    current_status_tree_ =
      std::make_shared<boost::property_tree::ptree>(hw_interface_->parse_json(str.value()));
  }
  diagnostics_updater_.force_update();
}

void VelodyneHwMonitorWrapper::velodyne_check_gps_pps_state(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_status_tree_ && !current_status_tree_->empty()) {
    auto tpl = velodyne_get_gps_pps_state();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_gps_position(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_status_tree_ && !current_status_tree_->empty()) {
    auto tpl = velodyne_get_gps_position();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_motor_state(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_status_tree_ && !current_status_tree_->empty()) {
    auto tpl = velodyne_get_motor_state();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_motor_rpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_status_tree_ && !current_status_tree_->empty()) {
    auto tpl = velodyne_get_motor_rpm();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_motor_lock(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_status_tree_ && !current_status_tree_->empty()) {
    auto tpl = velodyne_get_motor_lock();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_motor_phase(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_status_tree_ && !current_status_tree_->empty()) {
    auto tpl = velodyne_get_motor_phase();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_laser_state(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_status_tree_ && !current_status_tree_->empty()) {
    auto tpl = velodyne_get_laser_state();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_status(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_status_tree_ && !current_status_tree_->empty()) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = velodyne_get_motor_state();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_motor_state, std::get<2>(tpl));

    tpl = velodyne_get_laser_state();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_laser_state, std::get<2>(tpl));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_pps(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_status_tree_ && !current_status_tree_->empty()) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = velodyne_get_gps_pps_state();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_gps_pps_state, std::get<2>(tpl));

    tpl = velodyne_get_gps_position();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_gps_pps_position, std::get<2>(tpl));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_temperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = velodyne_get_top_lm20_temp();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_lm20_temp, std::get<2>(tpl));

    tpl = velodyne_get_bot_lm20_temp();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_lm20_temp, std::get<2>(tpl));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_rpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = velodyne_get_motor_rpm();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_motor_rpm, std::get<2>(tpl));

    tpl = velodyne_get_motor_lock();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_motor_lock, std::get<2>(tpl));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  }
}

void VelodyneHwMonitorWrapper::velodyne_check_voltage(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (current_diag_tree_ && !current_diag_tree_->empty()) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = velodyne_get_top_hv();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_hv, std::get<2>(tpl));

    tpl = velodyne_get_top_pwr5v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_5v, std::get<2>(tpl));

    tpl = velodyne_get_top_pwr25v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_2_5v, std::get<2>(tpl));

    tpl = velodyne_get_top_pwr33v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_3_3v, std::get<2>(tpl));

    if (sensor_configuration_->sensor_model == nebula::drivers::SensorModel::VELODYNE_VLP16) {
      tpl = velodyne_get_top_pwr5v_raw();
      if (std::get<0>(tpl)) {
        level = std::max(level, std::get<1>(tpl));
        if (0 < std::get<3>(tpl).length()) {
          msg.emplace_back(std::get<3>(tpl));
        }
      }
      diagnostics.add(name_volt_temp_top_pwr_5v_raw, std::get<2>(tpl));
    } else {
      tpl = velodyne_get_top_pwr_raw();
      if (std::get<0>(tpl)) {
        level = std::max(level, std::get<1>(tpl));
        if (0 < std::get<3>(tpl).length()) {
          msg.emplace_back(std::get<3>(tpl));
        }
      }
      diagnostics.add(name_volt_temp_top_pwr_raw, std::get<2>(tpl));
    }

    tpl = velodyne_get_top_pwr_vccint();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_vccint, std::get<2>(tpl));

    tpl = velodyne_get_bot_i_out();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_i_out, std::get<2>(tpl));

    tpl = velodyne_get_bot_pwr12v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_1_2v, std::get<2>(tpl));

    tpl = velodyne_get_bot_pwr5v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_5v, std::get<2>(tpl));

    tpl = velodyne_get_bot_pwr25v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_2_5v, std::get<2>(tpl));

    tpl = velodyne_get_bot_pwr33v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_3_3v, std::get<2>(tpl));

    tpl = velodyne_get_bot_pwr_v_in();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_v_in, std::get<2>(tpl));

    tpl = velodyne_get_bot_pwr125v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_1_25v, std::get<2>(tpl));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  }
}

std::string VelodyneHwMonitorWrapper::get_ptree_value(
  std::shared_ptr<boost::property_tree::ptree> pt, std::mutex & mtx_pt, const std::string & key)
{
  std::lock_guard lock(mtx_pt);
  boost::optional<std::string> value = pt->get_optional<std::string>(key);
  if (value) {
    return value.get();
  } else {
    return not_supported_message;
  }
}

std::string VelodyneHwMonitorWrapper::get_fixed_precision_string(double val, int pre)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(pre) << val;
  return ss.str();
}

Status VelodyneHwMonitorWrapper::status()
{
  return Status::OK;
}
}  // namespace nebula::ros
