// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/hesai/hw_monitor_wrapper.hpp"

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <nebula_common/nebula_common.hpp>

namespace nebula
{
namespace ros
{
HesaiHwMonitorWrapper::HesaiHwMonitorWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<nebula::drivers::HesaiHwInterface> & hw_interface,
  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config)
: logger_(parent_node->get_logger().get_child("HwMonitor")),
  diagnostics_updater_(parent_node),
  status_(Status::OK),
  hw_interface_(hw_interface),
  parent_node_(parent_node)
{
  diag_span_ = parent_node->declare_parameter<uint16_t>("diag_span", param_read_only());

  switch (config->sensor_model) {
    case nebula::drivers::SensorModel::HESAI_PANDARXT32:
    case nebula::drivers::SensorModel::HESAI_PANDARXT32M:
    case nebula::drivers::SensorModel::HESAI_PANDARAT128:
      temperature_names_.emplace_back("Bottom circuit board T1");
      temperature_names_.emplace_back("Bottom circuit board T2");
      temperature_names_.emplace_back("Laser emitting board RT_L1 (Internal)");
      temperature_names_.emplace_back("Laser emitting board RT_L2");
      temperature_names_.emplace_back("Receiving board RT_R");
      temperature_names_.emplace_back("Receiving board RT2");
      temperature_names_.emplace_back("Top circuit RT3");
      temperature_names_.emplace_back("Not used");
      break;
    case nebula::drivers::SensorModel::HESAI_PANDAR64:
    case nebula::drivers::SensorModel::HESAI_PANDAR40P:
    case nebula::drivers::SensorModel::HESAI_PANDAR40M:
    case nebula::drivers::SensorModel::HESAI_PANDARQT64:
    case nebula::drivers::SensorModel::HESAI_PANDARQT128:
    case nebula::drivers::SensorModel::HESAI_PANDAR128_E3X:
    case nebula::drivers::SensorModel::HESAI_PANDAR128_E4X:
    default:
      temperature_names_.emplace_back("Bottom circuit RT1");
      temperature_names_.emplace_back("Bottom circuit RT2");
      temperature_names_.emplace_back("Internal Temperature");
      temperature_names_.emplace_back("Laser emitting board RT1");
      temperature_names_.emplace_back("Laser emitting board RT2");
      temperature_names_.emplace_back("Receiving board RT1");
      temperature_names_.emplace_back("Top circuit RT1");
      temperature_names_.emplace_back("Top circuit RT2");
      break;
  }

  supports_monitor_ = config->sensor_model != drivers::SensorModel::HESAI_PANDARAT128;

  auto result = hw_interface->GetInventory();
  current_inventory_.reset(new HesaiInventory(result));
  current_inventory_time_.reset(new rclcpp::Time(parent_node->get_clock()->now()));
  std::cout << "HesaiInventory" << std::endl;
  std::cout << result << std::endl;
  info_model_ = result.get_str_model();
  info_serial_ = std::string(std::begin(result.sn), std::end(result.sn));
  RCLCPP_INFO_STREAM(logger_, "Model: " << info_model_);
  RCLCPP_INFO_STREAM(logger_, "Serial: " << info_serial_);
  initialize_hesai_diagnostics();
}

void HesaiHwMonitorWrapper::initialize_hesai_diagnostics()
{
  RCLCPP_INFO_STREAM(logger_, "initialize_hesai_diagnostics");
  using std::chrono_literals::operator""s;
  std::ostringstream os;
  auto hardware_id = info_model_ + ": " + info_serial_;
  diagnostics_updater_.setHardwareID(hardware_id);
  RCLCPP_INFO_STREAM(logger_, "Hardware ID: " + hardware_id);

  diagnostics_updater_.add("hesai_status", this, &HesaiHwMonitorWrapper::hesai_check_status);
  diagnostics_updater_.add("hesai_ptp", this, &HesaiHwMonitorWrapper::hesai_check_ptp);
  diagnostics_updater_.add(
    "hesai_temperature", this, &HesaiHwMonitorWrapper::hesai_check_temperature);
  diagnostics_updater_.add("hesai_rpm", this, &HesaiHwMonitorWrapper::hesai_check_rpm);

  current_status_.reset();
  current_monitor_.reset();
  current_status_time_.reset(new rclcpp::Time(parent_node_->get_clock()->now()));
  current_lidar_monitor_time_.reset(new rclcpp::Time(parent_node_->get_clock()->now()));
  current_diag_status_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  current_monitor_status_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;

  auto fetch_diag_from_sensor = [this]() {
    on_hesai_status_timer();

    if (!supports_monitor_) return;

    if (hw_interface_->UseHttpGetLidarMonitor()) {
      on_hesai_lidar_monitor_timer_http();
    } else {
      on_hesai_lidar_monitor_timer();
    }
  };

  fetch_diagnostics_timer_ = parent_node_->create_wall_timer(
    std::chrono::milliseconds(diag_span_), std::move(fetch_diag_from_sensor));

  if (hw_interface_->UseHttpGetLidarMonitor()) {
    diagnostics_updater_.add(
      "hesai_voltage", this, &HesaiHwMonitorWrapper::hesai_check_voltage_http);
  } else {
    diagnostics_updater_.add("hesai_voltage", this, &HesaiHwMonitorWrapper::hesai_check_voltage);
  }

  auto on_timer_update = [this] {
    RCLCPP_DEBUG_STREAM(logger_, "OnUpdateTimer");
    auto now = parent_node_->get_clock()->now();
    auto dif = (now - *current_status_time_).seconds();

    RCLCPP_DEBUG_STREAM(logger_, "dif(status): " << dif);

    if (diag_span_ * 2.0 < dif * 1000) {
      current_diag_status_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      RCLCPP_DEBUG_STREAM(logger_, "STALE");
    } else {
      current_diag_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
      RCLCPP_DEBUG_STREAM(logger_, "OK");
    }
    dif = (now - *current_lidar_monitor_time_).seconds();
    RCLCPP_DEBUG_STREAM(logger_, "dif(monitor): " << dif);
    if (diag_span_ * 2.0 < dif * 1000) {
      current_monitor_status_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      RCLCPP_DEBUG_STREAM(logger_, "STALE");
    } else {
      current_monitor_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
      RCLCPP_DEBUG_STREAM(logger_, "OK");
    }
    diagnostics_updater_.force_update();
  };
  diagnostics_update_timer_ =
    parent_node_->create_wall_timer(std::chrono::milliseconds(1000), std::move(on_timer_update));

  RCLCPP_DEBUG_STREAM(logger_, "add_timer");
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
    auto result = hw_interface_->GetLidarStatus();
    std::scoped_lock lock(mtx_lidar_status_);
    current_status_time_.reset(new rclcpp::Time(parent_node_->get_clock()->now()));
    current_status_.reset(new HesaiLidarStatus(result));
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

void HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer_http()
{
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_lidar_monitor_timer_http");
  try {
    hw_interface_->GetLidarMonitorAsyncHttp([this](const std::string & str) {
      std::scoped_lock lock(mtx_lidar_monitor_);
      current_lidar_monitor_time_.reset(new rclcpp::Time(parent_node_->get_clock()->now()));
      current_lidar_monitor_tree_ =
        std::make_unique<boost::property_tree::ptree>(hw_interface_->ParseJson(str));
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
    auto result = hw_interface_->GetLidarMonitor();
    std::scoped_lock lock(mtx_lidar_monitor_);
    current_lidar_monitor_time_.reset(new rclcpp::Time(parent_node_->get_clock()->now()));
    current_monitor_.reset(new HesaiLidarMonitor(result));
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

void HesaiHwMonitorWrapper::hesai_check_status(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_status_);
  if (current_status_) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    diagnostics.add("system_uptime", std::to_string(current_status_->system_uptime.value()));
    diagnostics.add("startup_times", std::to_string(current_status_->startup_times.value()));
    diagnostics.add(
      "total_operation_time", std::to_string(current_status_->total_operation_time.value()));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_ptp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_status_);
  if (current_status_) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    auto gps_status = current_status_->get_str_gps_pps_lock();
    auto gprmc_status = current_status_->get_str_gps_gprmc_status();
    auto ptp_status = current_status_->get_str_ptp_clock_status();
    std::transform(gps_status.cbegin(), gps_status.cend(), gps_status.begin(), toupper);
    std::transform(gprmc_status.cbegin(), gprmc_status.cend(), gprmc_status.begin(), toupper);
    std::transform(ptp_status.cbegin(), ptp_status.cend(), ptp_status.begin(), toupper);
    diagnostics.add("gps_pps_lock", gps_status);
    diagnostics.add("gps_gprmc_status", gprmc_status);
    diagnostics.add("ptp_clock_status", ptp_status);
    if (gps_status != "UNKNOWN") {
      msg.emplace_back("gps_pps_lock: " + gps_status);
    }
    if (gprmc_status != "UNKNOWN") {
      msg.emplace_back("gprmc_status: " + gprmc_status);
    }
    if (ptp_status != "UNKNOWN") {
      msg.emplace_back("ptp_status: " + ptp_status);
    }
    if (ptp_status == "FREE RUN" && gps_status == "UNKNOWN") {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_temperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_status_);
  if (current_status_) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    for (size_t i = 0; i < std::size(current_status_->temperature); i++) {
      diagnostics.add(
        temperature_names_[i],
        get_fixed_precision_string(current_status_->temperature[i].value() * 0.01, 3));
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
    diagnostics.add("motor_speed", std::to_string(current_status_->motor_speed.value()));

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
    diagnostics.add(key, mes);
    key = "lidarInVol";
    try {
      mes = get_ptree_value(current_lidar_monitor_tree_.get(), "Body." + key);
    } catch (boost::bad_lexical_cast & ex) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      mes = MSG_ERROR_ + std::string(ex.what());
    }
    diagnostics.add(key, mes);

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
    diagnostics.add(
      "input_voltage",
      get_fixed_precision_string(current_monitor_->input_voltage.value() * 0.01, 3) + " V");
    diagnostics.add(
      "input_current",
      get_fixed_precision_string(current_monitor_->input_current.value() * 0.01, 3) +
        " m"
        "A");
    diagnostics.add(
      "input_power",
      get_fixed_precision_string(current_monitor_->input_power.value() * 0.01, 3) + " W");

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

Status HesaiHwMonitorWrapper::status()
{
  return Status::OK;
}
}  // namespace ros
}  // namespace nebula
