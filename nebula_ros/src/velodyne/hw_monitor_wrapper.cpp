// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/velodyne/hw_monitor_wrapper.hpp"

namespace nebula
{
namespace ros
{
VelodyneHwMonitorWrapper::VelodyneHwMonitorWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<nebula::drivers::VelodyneHwInterface> & hw_interface,
  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config)
: logger_(parent_node->get_logger().get_child("HwMonitor")),
  diagnostics_updater_(parent_node),
  status_(Status::OK),
  hw_interface_(hw_interface),
  parent_node_(parent_node),
  sensor_configuration_(config)
{
  diag_span_ = parent_node->declare_parameter<uint16_t>("diag_span", param_read_only());
  show_advanced_diagnostics_ =
    parent_node->declare_parameter<bool>("advanced_diagnostics", param_read_only());

  std::cout << "Get model name and serial." << std::endl;
  auto str = hw_interface_->GetSnapshot();
  current_snapshot_time.reset(new rclcpp::Time(parent_node_->now()));
  current_snapshot_tree =
    std::make_shared<boost::property_tree::ptree>(hw_interface_->ParseJson(str));
  current_diag_tree =
    std::make_shared<boost::property_tree::ptree>(current_snapshot_tree->get_child("diag"));
  current_status_tree =
    std::make_shared<boost::property_tree::ptree>(current_snapshot_tree->get_child("status"));
  current_snapshot.reset(new std::string(str));

  try {
    info_model_ = GetPtreeValue(current_snapshot_tree, mtx_snapshot_, key_info_model);
    info_serial_ = GetPtreeValue(current_snapshot_tree, mtx_snapshot_, key_info_serial);
    RCLCPP_INFO_STREAM(logger_, "Model: " << info_model_);
    RCLCPP_INFO_STREAM(logger_, "Serial: " << info_serial_);
  } catch (boost::bad_lexical_cast & ex) {
    RCLCPP_ERROR(logger_, " Error: Can't get model and serial");
    return;
  }

  InitializeVelodyneDiagnostics();
}

void VelodyneHwMonitorWrapper::InitializeVelodyneDiagnostics()
{
  RCLCPP_INFO_STREAM(logger_, "InitializeVelodyneDiagnostics");
  using std::chrono_literals::operator""s;
  std::ostringstream os;
  auto hardware_id = info_model_ + ": " + info_serial_;
  diagnostics_updater_.setHardwareID(hardware_id);
  RCLCPP_INFO_STREAM(logger_, "hardware_id" << hardware_id);

  if (show_advanced_diagnostics_) {
    diagnostics_updater_.add(
      "velodyne_snapshot-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckSnapshot);

    diagnostics_updater_.add(
      "velodyne_volt_temp_top_hv-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckTopHv);
    if (sensor_configuration_->sensor_model != nebula::drivers::SensorModel::VELODYNE_VLP16) {
      diagnostics_updater_.add(
        "velodyne_volt_temp_top_ad_temp-" + sensor_configuration_->frame_id, this,
        &VelodyneHwMonitorWrapper::VelodyneCheckTopAdTemp);
    }
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_lm20_temp-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckTopLm20Temp);
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_5v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckTopPwr5v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_2_5v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckTopPwr25v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_3_3v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckTopPwr33v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_raw-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckTopPwrRaw);
    diagnostics_updater_.add(
      "velodyne_volt_temp_top_pwr_vccint-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckTopPwrVccint);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_i_out-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckBotIOut);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_1_2v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckBotPwr12v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_lm20_temp-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckBotLm20Temp);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_5v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckBotPwr5v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_2_5v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckBotPwr25v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_3_3v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckBotPwr33v);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_v_in-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckBotPwrVIn);
    diagnostics_updater_.add(
      "velodyne_volt_temp_bot_pwr_1_25v-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckBotPwr125v);
    diagnostics_updater_.add(
      "velodyne_vhv-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckVhv);
    diagnostics_updater_.add(
      "velodyne_adc_nf-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckAdcNf);
    diagnostics_updater_.add(
      "velodyne_adc_stats-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckAdcStats);
    diagnostics_updater_.add(
      "velodyne_ixe-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckIxe);
    diagnostics_updater_.add(
      "velodyne_adctp_stat-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckAdctpStat);

    diagnostics_updater_.add(
      "velodyne_status_gps_pps_state-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckGpsPpsState);
    diagnostics_updater_.add(
      "velodyne_status_gps_pps_position-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckGpsPosition);
    diagnostics_updater_.add(
      "velodyne_status_motor_state-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckMotorState);
    diagnostics_updater_.add(
      "velodyne_status_motor_rpm-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckMotorRpm);
    diagnostics_updater_.add(
      "velodyne_status_motor_lock-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckMotorLock);
    diagnostics_updater_.add(
      "velodyne_status_motor_phase-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckMotorPhase);
    diagnostics_updater_.add(
      "velodyne_status_laser_state-" + sensor_configuration_->frame_id, this,
      &VelodyneHwMonitorWrapper::VelodyneCheckLaserState);
  }

  diagnostics_updater_.add("velodyne_status", this, &VelodyneHwMonitorWrapper::VelodyneCheckStatus);
  diagnostics_updater_.add("velodyne_pps", this, &VelodyneHwMonitorWrapper::VelodyneCheckPps);
  diagnostics_updater_.add(
    "velodyne_temperature", this, &VelodyneHwMonitorWrapper::VelodyneCheckTemperature);
  diagnostics_updater_.add("velodyne_rpm", this, &VelodyneHwMonitorWrapper::VelodyneCheckRpm);
  diagnostics_updater_.add(
    "velodyne_voltage", this, &VelodyneHwMonitorWrapper::VelodyneCheckVoltage);

  {
    std::lock_guard lock(mtx_snapshot_);
    current_snapshot.reset(new std::string(""));
    current_snapshot_time.reset(new rclcpp::Time(parent_node_->now()));
  }

  current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;

  auto on_timer_snapshot = [this] { OnVelodyneSnapshotTimer(); };
  diagnostics_snapshot_timer_ = parent_node_->create_wall_timer(
    std::chrono::milliseconds(diag_span_), std::move(on_timer_snapshot));

  auto on_timer_update = [this] {
    auto now = parent_node_->now();
    double dif;
    {
      std::lock_guard lock(mtx_snapshot_);
      dif = (now - *current_snapshot_time).seconds();
    }
    if (diag_span_ * 2.0 < dif * 1000) {
      current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      RCLCPP_DEBUG_STREAM(logger_, "STALE");
    } else {
      current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
      RCLCPP_DEBUG_STREAM(logger_, "OK");
    }
    diagnostics_updater_.force_update();
  };
  diagnostics_update_timer_ =
    parent_node_->create_wall_timer(std::chrono::milliseconds(1000), std::move(on_timer_update));
}

void VelodyneHwMonitorWrapper::OnVelodyneSnapshotTimer()
{
  auto str = hw_interface_->GetSnapshot();
  auto ptree = hw_interface_->ParseJson(str);

  {
    std::lock_guard lock(mtx_snapshot_);

    current_snapshot_time.reset(new rclcpp::Time(parent_node_->now()));
    current_snapshot_tree = std::make_shared<boost::property_tree::ptree>(ptree);
    current_diag_tree =
      std::make_shared<boost::property_tree::ptree>(current_snapshot_tree->get_child("diag"));
    current_status_tree =
      std::make_shared<boost::property_tree::ptree>(current_snapshot_tree->get_child("status"));
    current_snapshot.reset(new std::string(str));
  }
}

void VelodyneHwMonitorWrapper::OnVelodyneDiagnosticsTimer()
{
  std::cout << "OnVelodyneDiagnosticsTimer" << std::endl;
  auto str = hw_interface_->GetDiag();
  {
    std::lock_guard lock(mtx_diag_);
    current_diag_tree =
      std::make_shared<boost::property_tree::ptree>(hw_interface_->ParseJson(str));
  }
  diagnostics_updater_.force_update();
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetTopHv()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_top_hv));
    val = 101.0 * (val * 5.0 / 4096.0 - 5.0);
    if (val < -150.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_hv + message_sep + voltage_low_message;
    } else if (-132.0 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_hv + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetTopAdTemp()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_top_ad_temp));
    val = val * 5.0 / 4096.0;
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::VelodyneGetTopLm20Temp()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_top_lm20_temp));
    val = -1481.96 + std::sqrt(2.1962e6 + ((1.8639 - val * 5.0 / 4096.0) / 3.88e-6));
    if (val < -25.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_lm20_temp + message_sep + temperature_cold_message;
    } else if (90.0 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_lm20_temp + message_sep + temperature_hot_message;
    }
    //    mes = boost::lexical_cast<std::string>(val) + " C";
    mes = GetFixedPrecisionString(val) + " C";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetTopPwr5v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_top_pwr_5v));
    val = 2.0 * val * 5.0 / 4096.0;
    if (val < 4.8) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v + message_sep + voltage_low_message;
    } else if (5.2 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetTopPwr25v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_top_pwr_2_5v));
    val = val * 5.0 / 4096.0;
    if (val < 2.3) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_2_5v + message_sep + voltage_low_message;
    } else if (2.7 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_2_5v + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetTopPwr33v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_top_pwr_3_3v));
    val = val * 5.0 / 4096.0;
    if (val < 3.1) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_3_3v + message_sep + voltage_low_message;
    } else if (3.5 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_3_3v + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::VelodyneGetTopPwr5vRaw()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_top_pwr_5v_raw));
    val = 2.0 * val * 5.0 / 4096.0;
    if (val < 2.3) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v_raw + message_sep + voltage_low_message;
    } else if (2.7 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_5v_raw + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetTopPwrRaw()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_top_pwr_raw));
    val = val * 5.0 / 4096.0;
    if (val < 1.6) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_raw + message_sep + voltage_low_message;
    } else if (1.9 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_raw + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::VelodyneGetTopPwrVccint()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_top_pwr_vccint));
    val = val * 5.0 / 4096.0;
    if (val < 1.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_vccint + message_sep + voltage_low_message;
    } else if (1.4 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_top_pwr_vccint + message_sep + voltage_high_message;
    }
    //    mes = boost::lexical_cast<std::string>(val) + " V";
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetBotIOut()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_bot_i_out));
    val = 10.0 * (val * 5.0 / 4096.0 - 2.5);
    if (val < 0.3) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_i_out + message_sep + ampere_low_message;
    } else if (1.0 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_i_out + message_sep + ampere_high_message;
    }
    mes = GetFixedPrecisionString(val) + " A";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetBotPwr12v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_bot_pwr_1_2v));
    val = val * 5.0 / 4096.0;
    if (val < 1.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_2v + message_sep + voltage_low_message;
    } else if (1.4 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_2v + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::VelodyneGetBotLm20Temp()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_bot_lm20_temp));
    val = -1481.96 + std::sqrt(2.1962e6 + ((1.8639 - val * 5.0 / 4096.0) / 3.88e-6));
    if (val < -25.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_lm20_temp + message_sep + temperature_cold_message;
    } else if (90.0 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_lm20_temp + message_sep + temperature_hot_message;
    }
    //    mes = boost::lexical_cast<std::string>(val) + " C";
    mes = GetFixedPrecisionString(val) + " C";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetBotPwr5v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_bot_pwr_5v));
    val = 2.0 * val * 5.0 / 4096.0;
    if (val < 4.8) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_5v + message_sep + voltage_low_message;
    } else if (5.2 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_5v + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetBotPwr25v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_bot_pwr_2_5v));
    val = val * 5.0 / 4096.0;
    if (val < 2.3) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_2_5v + message_sep + voltage_low_message;
    } else if (2.7 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_2_5v + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetBotPwr33v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_bot_pwr_3_3v));
    val = val * 5.0 / 4096.0;
    if (val < 3.1) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_3_3v + message_sep + voltage_low_message;
    } else if (3.5 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_3_3v + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetBotPwrVIn()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_bot_pwr_v_in));
    val = 11.0 * val * 5.0 / 4096.0;
    if (val < 8.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_v_in + message_sep + voltage_low_message;
    } else if (19.0 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_v_in + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::VelodyneGetBotPwr125v()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(
      GetPtreeValue(current_diag_tree, mtx_diag_, key_volt_temp_bot_pwr_1_25v));
    val = val * 5.0 / 4096.0;
    if (val < 1.0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_25v + message_sep + voltage_low_message;
    } else if (1.4 < val) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      error_mes = name_volt_temp_bot_pwr_1_25v + message_sep + voltage_high_message;
    }
    mes = GetFixedPrecisionString(val) + " V";
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetVhv()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    double val = 0.0;
    val = boost::lexical_cast<double>(GetPtreeValue(current_diag_tree, mtx_diag_, key_vhv));
    mes = boost::lexical_cast<std::string>(val);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetAdcNf()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    std::ostringstream os;
    boost::optional<boost::property_tree::ptree &> child =
      current_diag_tree->get_child_optional(key_adc_nf);
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

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetAdcStats()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    std::ostringstream os;
    boost::optional<boost::property_tree::ptree &> child =
      current_diag_tree->get_child_optional(key_adc_stats);
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

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetIxe()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = GetPtreeValue(current_diag_tree, mtx_diag_, key_ixe);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetAdctpStat()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    std::ostringstream os;
    boost::optional<boost::property_tree::ptree &> child =
      current_diag_tree->get_child_optional(key_adctp_stat);
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
VelodyneHwMonitorWrapper::VelodyneGetGpsPpsState()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = GetPtreeValue(current_status_tree, mtx_status_, key_status_gps_pps_state);
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
VelodyneHwMonitorWrapper::VelodyneGetGpsPosition()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = GetPtreeValue(current_status_tree, mtx_status_, key_status_gps_pps_position);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::VelodyneGetMotorState()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = GetPtreeValue(current_status_tree, mtx_status_, key_status_motor_state);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetMotorRpm()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = GetPtreeValue(current_status_tree, mtx_status_, key_status_motor_rpm);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string> VelodyneHwMonitorWrapper::VelodyneGetMotorLock()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = GetPtreeValue(current_status_tree, mtx_status_, key_status_motor_lock);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::VelodyneGetMotorPhase()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = GetPtreeValue(current_status_tree, mtx_status_, key_status_motor_phase);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
VelodyneHwMonitorWrapper::VelodyneGetLaserState()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = GetPtreeValue(current_status_tree, mtx_status_, key_status_laser_state);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = error_message;
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

void VelodyneHwMonitorWrapper::VelodyneCheckTopHv(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetTopHv();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckTopAdTemp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetTopAdTemp();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckTopLm20Temp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetTopLm20Temp();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckTopPwr5v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetTopPwr5v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckTopPwr25v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetTopPwr25v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckTopPwr33v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetTopPwr33v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckTopPwrRaw(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetTopPwrRaw();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckTopPwrVccint(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetTopPwrVccint();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckBotIOut(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetBotIOut();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckBotPwr12v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetBotPwr12v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckBotLm20Temp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetBotLm20Temp();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckBotPwr5v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetBotPwr5v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckBotPwr25v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetBotPwr25v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckBotPwr33v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetBotPwr33v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckBotPwrVIn(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetBotPwrVIn();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckBotPwr125v(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetBotPwr125v();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckVhv(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetVhv();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckAdcNf(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetAdcNf();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckAdcStats(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetAdcStats();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckIxe(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetIxe();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckAdctpStat(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    auto tpl = VelodyneGetAdctpStat();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::OnVelodyneStatusTimer()
{
  auto str = hw_interface_->GetStatus();
  {
    std::lock_guard lock(mtx_status_);
    current_status_tree =
      std::make_shared<boost::property_tree::ptree>(hw_interface_->ParseJson(str));
  }
  diagnostics_updater_.force_update();
}

void VelodyneHwMonitorWrapper::VelodyneCheckGpsPpsState(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_status_tree &&
    !VelodyneHwMonitorWrapper::current_status_tree->empty()) {
    auto tpl = VelodyneGetGpsPpsState();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckGpsPosition(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_status_tree &&
    !VelodyneHwMonitorWrapper::current_status_tree->empty()) {
    auto tpl = VelodyneGetGpsPosition();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckMotorState(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_status_tree &&
    !VelodyneHwMonitorWrapper::current_status_tree->empty()) {
    auto tpl = VelodyneGetMotorState();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckMotorRpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_status_tree &&
    !VelodyneHwMonitorWrapper::current_status_tree->empty()) {
    auto tpl = VelodyneGetMotorRpm();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckMotorLock(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_status_tree &&
    !VelodyneHwMonitorWrapper::current_status_tree->empty()) {
    auto tpl = VelodyneGetMotorLock();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckMotorPhase(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_status_tree &&
    !VelodyneHwMonitorWrapper::current_status_tree->empty()) {
    auto tpl = VelodyneGetMotorPhase();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckLaserState(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_status_tree &&
    !VelodyneHwMonitorWrapper::current_status_tree->empty()) {
    auto tpl = VelodyneGetLaserState();
    diagnostics.add("sensor", sensor_configuration_->frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void VelodyneHwMonitorWrapper::VelodyneCheckSnapshot(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = current_diag_status;
  diagnostics.add("sensor", sensor_configuration_->frame_id);
  diagnostics.summary(level, *current_snapshot);
}

void VelodyneHwMonitorWrapper::VelodyneCheckStatus(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_status_tree &&
    !VelodyneHwMonitorWrapper::current_status_tree->empty()) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = VelodyneGetMotorState();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_motor_state, std::get<2>(tpl));

    tpl = VelodyneGetLaserState();
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

void VelodyneHwMonitorWrapper::VelodyneCheckPps(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_status_tree &&
    !VelodyneHwMonitorWrapper::current_status_tree->empty()) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = VelodyneGetGpsPpsState();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_gps_pps_state, std::get<2>(tpl));

    tpl = VelodyneGetGpsPosition();
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

void VelodyneHwMonitorWrapper::VelodyneCheckTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = VelodyneGetTopLm20Temp();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_lm20_temp, std::get<2>(tpl));

    tpl = VelodyneGetBotLm20Temp();
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

void VelodyneHwMonitorWrapper::VelodyneCheckRpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = VelodyneGetMotorRpm();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_status_motor_rpm, std::get<2>(tpl));

    tpl = VelodyneGetMotorLock();
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

void VelodyneHwMonitorWrapper::VelodyneCheckVoltage(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    VelodyneHwMonitorWrapper::current_diag_tree &&
    !VelodyneHwMonitorWrapper::current_diag_tree->empty()) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    auto tpl = VelodyneGetTopHv();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_hv, std::get<2>(tpl));

    tpl = VelodyneGetTopPwr5v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_5v, std::get<2>(tpl));

    tpl = VelodyneGetTopPwr25v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_2_5v, std::get<2>(tpl));

    tpl = VelodyneGetTopPwr33v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_3_3v, std::get<2>(tpl));

    if (sensor_configuration_->sensor_model == nebula::drivers::SensorModel::VELODYNE_VLP16) {
      tpl = VelodyneGetTopPwr5vRaw();
      if (std::get<0>(tpl)) {
        level = std::max(level, std::get<1>(tpl));
        if (0 < std::get<3>(tpl).length()) {
          msg.emplace_back(std::get<3>(tpl));
        }
      }
      diagnostics.add(name_volt_temp_top_pwr_5v_raw, std::get<2>(tpl));
    } else {
      tpl = VelodyneGetTopPwrRaw();
      if (std::get<0>(tpl)) {
        level = std::max(level, std::get<1>(tpl));
        if (0 < std::get<3>(tpl).length()) {
          msg.emplace_back(std::get<3>(tpl));
        }
      }
      diagnostics.add(name_volt_temp_top_pwr_raw, std::get<2>(tpl));
    }

    tpl = VelodyneGetTopPwrVccint();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_top_pwr_vccint, std::get<2>(tpl));

    tpl = VelodyneGetBotIOut();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_i_out, std::get<2>(tpl));

    tpl = VelodyneGetBotPwr12v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_1_2v, std::get<2>(tpl));

    tpl = VelodyneGetBotPwr5v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_5v, std::get<2>(tpl));

    tpl = VelodyneGetBotPwr25v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_2_5v, std::get<2>(tpl));

    tpl = VelodyneGetBotPwr33v();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_3_3v, std::get<2>(tpl));

    tpl = VelodyneGetBotPwrVIn();
    if (std::get<0>(tpl)) {
      level = std::max(level, std::get<1>(tpl));
      if (0 < std::get<3>(tpl).length()) {
        msg.emplace_back(std::get<3>(tpl));
      }
    }
    diagnostics.add(name_volt_temp_bot_pwr_v_in, std::get<2>(tpl));

    tpl = VelodyneGetBotPwr125v();
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

std::string VelodyneHwMonitorWrapper::GetPtreeValue(
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

std::string VelodyneHwMonitorWrapper::GetFixedPrecisionString(double val, int pre)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(pre) << val;
  return ss.str();
}

Status VelodyneHwMonitorWrapper::Status()
{
  return Status::OK;
}
}  // namespace ros
}  // namespace nebula
