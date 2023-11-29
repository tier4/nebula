#include "nebula_ros/robosense/robosense_hw_monitor_ros_wrapper.hpp"

#include <boost/algorithm/string/join.hpp>
#include <boost/lexical_cast.hpp>

#include <curl/curl.h>
#include <math.h>

#include <future>

namespace nebula
{
namespace ros
{
RobosenseHwMonitorRosWrapper::RobosenseHwMonitorRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("robosense_hw_monitor_ros_wrapper", options),
  hw_interface_(),
  diagnostics_updater_(this)
{
  if (mtx_config_.try_lock()) {
    interface_status_ = GetParameters(sensor_configuration_);
    mtx_config_.unlock();
  }
  if (Status::OK != interface_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
    return;
  }
  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));
  // Initialize sensor_configuration
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialize sensor_configuration");
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::RobosenseSensorConfiguration>(sensor_configuration_);
  RCLCPP_INFO_STREAM(this->get_logger(), "hw_interface_.InitializeSensorConfiguration");

  info_model = GetInfoModel(sensor_cfg_ptr->sensor_model);
  if (
    sensor_cfg_ptr->sensor_model == drivers::SensorModel::ROBOSENSE_HELIOS ||
    sensor_cfg_ptr->sensor_model == drivers::SensorModel::ROBOSENSE_HELIOS16P) {
    robosense_helios_monitor_info_sub_ =
      create_subscription<robosense_msgs::msg::RobosenseHeliosMonitorInfo>(
        "robosense_helios_monitor_info", rclcpp::SensorDataQoS(),
        std::bind(
          &RobosenseHwMonitorRosWrapper::ReceiveHeliosMonitorInfoCallback, this,
          std::placeholders::_1));
  } else if (
    sensor_cfg_ptr->sensor_model == drivers::SensorModel::ROBOSENSE_BPEARL ||
    sensor_cfg_ptr->sensor_model == drivers::SensorModel::ROBOSENSE_RUBYPLUS) {
    robosense_bp_ruby_monitor_info_sub_ =
      create_subscription<robosense_msgs::msg::RobosenseBpRubyMonitorInfo>(
        "robosense_bp_ruby_monitor_info", rclcpp::SensorDataQoS(),
        std::bind(
          &RobosenseHwMonitorRosWrapper::ReceiveBpRubyMonitorInfoCallback, this,
          std::placeholders::_1));
  } else if (sensor_cfg_ptr->sensor_model == drivers::SensorModel::ROBOSENSE_M1PLUS) {
    robosense_mems_monitor_info_sub_ =
      create_subscription<robosense_msgs::msg::RobosenseMemsMonitorInfo>(
        "robosense_mems_monitor_info", rclcpp::SensorDataQoS(),
        std::bind(
          &RobosenseHwMonitorRosWrapper::ReceiveMemsMonitorInfoCallback, this,
          std::placeholders::_1));
  } else if (sensor_cfg_ptr->sensor_model == drivers::SensorModel::ROBOSENSE_E1) {
    robosense_flash_monitor_info_sub_ =
      create_subscription<robosense_msgs::msg::RobosenseFlashMonitorInfo>(
        "robosense_flash_monitor_info", rclcpp::SensorDataQoS(),
        std::bind(
          &RobosenseHwMonitorRosWrapper::ReceiveFlashMonitorInfoCallback, this,
          std::placeholders::_1));
  }
}

void RobosenseHwMonitorRosWrapper::ReceiveHeliosMonitorInfoCallback(
  const robosense_msgs::msg::RobosenseHeliosMonitorInfo::SharedPtr monitor_info)
{
  device_param_val.rpm = monitor_info->motor_rmp;
  device_param_val.bot_fpga_temperature_val =
    0.123041 * monitor_info->bot_fpga_temperature - 273.15;
  device_param_val.heliosDeviceCalculateParamsInfo.recv_A_temperature_val =
    0.048828 * monitor_info->recv_a_temperature - 50;
  device_param_val.heliosDeviceCalculateParamsInfo.recv_B_temperature_val =
    0.048828 * monitor_info->recv_b_temperature - 50;
  device_param_val.main_fpga_temperature_val = 0.048828 * monitor_info->main_fpga_temperature - 50;
  device_param_val.heliosDeviceCalculateParamsInfo.main_fpga_core_temperature_val =
    0.123041 * monitor_info->main_fpga_core_temperature - 273.15;
  device_param_val.lane_up = monitor_info->lane_up;
  device_param_val.lane_up_cnt = monitor_info->lane_up_cnt;
  device_param_val.main_status = monitor_info->main_status;
  device_param_val.gps_status = monitor_info->gps_status;
  if (!is_get_info_serial_) {
    for (int i = 0; i < 6; ++i) {
      device_param_val.sn[i] = monitor_info->device_sn[i];
    }
    info_serial = to_hex(device_param_val.sn, 6);
    InitializeRobosenseDiagnostics();
    is_get_info_serial_ = true;
  }

  diagnostics_updater_.force_update();
}

void RobosenseHwMonitorRosWrapper::ReceiveBpRubyMonitorInfoCallback(
  const robosense_msgs::msg::RobosenseBpRubyMonitorInfo::SharedPtr monitor_info)
{
  device_param_val.rpm = monitor_info->motor_rmp;
  device_param_val.bpRubyDeviceCalculateParamsInfo.apd_temperature_val =
    monitor_info->apd_temperature * 0.01;
  device_param_val.bpRubyDeviceCalculateParamsInfo.main_bot_temperature_val =
    monitor_info->main_bot_temperature * 0.01;
  device_param_val.bot_fpga_temperature_val = monitor_info->bot_fpga_temperature * 0.01;
  device_param_val.main_fpga_temperature_val = monitor_info->main_fpga_temperature * 0.01;
  device_param_val.gps_status = monitor_info->gps_status;
  if (!is_get_info_serial_) {
    for (int i = 0; i < 6; ++i) {
      device_param_val.sn[i] = monitor_info->device_sn[i];
    }
    info_serial = to_hex(device_param_val.sn, 6);
    InitializeRobosenseDiagnostics();
    is_get_info_serial_ = true;
  }
  diagnostics_updater_.force_update();
}

void RobosenseHwMonitorRosWrapper::ReceiveMemsMonitorInfoCallback(
  const robosense_msgs::msg::RobosenseMemsMonitorInfo::SharedPtr monitor_info)
{
  device_param_val.memsDeviceCalculateParamsInfo.battery_volt_val =
    monitor_info->battery_volt * 0.01;
  device_param_val.lidar_fault_status = monitor_info->lidar_fault_status;
  device_param_val.lidar_roi_status = monitor_info->lidar_roi_status;
  if (!is_get_info_serial_) {
    for (int i = 0; i < 6; ++i) {
      device_param_val.sn[i] = monitor_info->device_sn[i];
    }
    info_serial = to_hex(device_param_val.sn, 6);
    InitializeRobosenseDiagnostics();
    is_get_info_serial_ = true;
  }

  diagnostics_updater_.force_update();
}

void RobosenseHwMonitorRosWrapper::ReceiveFlashMonitorInfoCallback(
  const robosense_msgs::msg::RobosenseFlashMonitorInfo::SharedPtr monitor_info)
{
  device_param_val.flashDeviceCalculateParamsInfo.pl_vmon_vin_p_val =
    monitor_info->pl_vmon_vin_p * 0.001;
  device_param_val.fpga_pl_kernal_temperature = monitor_info->fpga_pl_kernal_temperature - 80;
  device_param_val.fpga_ps_kernal_temperature = monitor_info->fpga_ps_kernal_temperature - 80;
  device_param_val.window_temperature = monitor_info->window_temperature - 80;
  if (!is_get_info_serial_) {
    for (int i = 0; i < 6; ++i) {
      device_param_val.sn[i] = monitor_info->device_sn[i];
    }
    info_serial = to_hex(device_param_val.sn, 6);
    InitializeRobosenseDiagnostics();
    is_get_info_serial_ = true;
  }
}

Status RobosenseHwMonitorRosWrapper::MonitorStart()
{
  return interface_status_;
}

Status RobosenseHwMonitorRosWrapper::MonitorStop()
{
  return Status::OK;
}
Status RobosenseHwMonitorRosWrapper::Shutdown()
{
  return Status::OK;
}

Status RobosenseHwMonitorRosWrapper::InitializeHwMonitor(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

std::string RobosenseHwMonitorRosWrapper::GetInfoModel(nebula::drivers::SensorModel sensor_model)
{
  std::string str_info_model;
  switch (sensor_model) {
    case nebula::drivers::SensorModel::ROBOSENSE_HELIOS:
      str_info_model = "RobosenseHelios";
      break;
    case nebula::drivers::SensorModel::ROBOSENSE_HELIOS16P:
      str_info_model = "RobosenseHelios16P";
      break;
    case nebula::drivers::SensorModel::ROBOSENSE_BPEARL:
      str_info_model = "RobosenseBpearl";
      break;
    case nebula::drivers::SensorModel::ROBOSENSE_RUBYPLUS:
      str_info_model = "RobosenseRubyPlus";
      break;
    case nebula::drivers::SensorModel::ROBOSENSE_M1PLUS:
      str_info_model = "RobosenseM1Plus";
      break;
    case nebula::drivers::SensorModel::ROBOSENSE_E1:
      str_info_model = "RobosenseE1";
      break;
    default:
      str_info_model = "Unkonw";
  }
  return str_info_model;
}

Status RobosenseHwMonitorRosWrapper::GetParameters(
  drivers::RobosenseSensorConfiguration & sensor_configuration)
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", "");
    sensor_configuration.sensor_model =
      nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("return_mode", "", descriptor);
    sensor_configuration.return_mode = nebula::drivers::ReturnModeFromStringRobosense(
      this->get_parameter("return_mode").as_string(), sensor_configuration.sensor_model);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("host_ip", "255.255.255.255", descriptor);
    sensor_configuration.host_ip = this->get_parameter("host_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("lidar_ip", "192.168.1.201", descriptor);
    sensor_configuration.sensor_ip = this->get_parameter("lidar_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "robosense", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("data_port", 2368, descriptor);
    sensor_configuration.data_port = this->get_parameter("data_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("difop_port", 2369, descriptor);
    sensor_configuration.difop_port = this->get_parameter("difop_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0).set__to_value(360).set__step(0.01);
    descriptor.floating_point_range = {range};
    this->declare_parameter<double>("scan_phase", 0., descriptor);
    sensor_configuration.scan_phase = this->get_parameter("scan_phase").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("packet_mtu_size", 1500, descriptor);
    sensor_configuration.packet_mtu_size = this->get_parameter("packet_mtu_size").as_int();
  }
  {
    if (
      sensor_configuration.sensor_model != drivers::SensorModel::ROBOSENSE_E1 &&
      sensor_configuration.sensor_model != nebula::drivers::SensorModel::ROBOSENSE_M1PLUS) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      descriptor.read_only = false;
      descriptor.dynamic_typing = false;
      rcl_interfaces::msg::IntegerRange range;
      if (
        sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_HELIOS ||
        sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_HELIOS16P ||
        sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_RUBYPLUS ||
        sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_BPEARL) {
        descriptor.additional_constraints = "300, 600, 1200";
        range.set__from_value(300).set__to_value(1200).set__step(300);
        descriptor.integer_range = {range};
        this->declare_parameter<uint16_t>("rotation_speed", 600, descriptor);
      }
      sensor_configuration.rotation_speed = this->get_parameter("rotation_speed").as_int();
    }
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(360).set__step(1);
    descriptor.integer_range = {range};
    this->declare_parameter<uint16_t>("cloud_min_angle", 0, descriptor);
    sensor_configuration.cloud_min_angle = this->get_parameter("cloud_min_angle").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(360).set__step(1);
    descriptor.integer_range = {range};
    this->declare_parameter<uint16_t>("cloud_max_angle", 360, descriptor);
    sensor_configuration.cloud_max_angle = this->get_parameter("cloud_max_angle").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Dual return distance threshold [0.01, 0.5]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0.01).set__to_value(0.5).set__step(0.01);
    descriptor.floating_point_range = {range};
    this->declare_parameter<double>("dual_return_distance_threshold", 0.1, descriptor);
    sensor_configuration.dual_return_distance_threshold =
      this->get_parameter("dual_return_distance_threshold").as_double();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360) {  // ||
    return Status::SENSOR_CONFIG_ERROR;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void RobosenseHwMonitorRosWrapper::InitializeRobosenseDiagnostics()
{
  RCLCPP_INFO_STREAM(get_logger(), " InitializeRobosenseDiagnostics");
  using std::chrono_literals::operator""s;
  std::ostringstream os;
  auto hardware_id = info_model + ": " + info_serial;
  diagnostics_updater_.setHardwareID(hardware_id);
  RCLCPP_INFO_STREAM(get_logger(), "hardware_id" << hardware_id);
  if (
    info_model.find("RobosenseHelios") != std::string::npos ||
    info_model.find("RobosenseBpearl") != std::string::npos ||
    info_model.find("RobosenseRubyPlus") != std::string::npos) {
    diagnostics_updater_.add(
      "robosense_rpm", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckRpm);
    diagnostics_updater_.add(
      "main_fpga_temperature", this,
      &RobosenseHwMonitorRosWrapper::RobosenseCheckMainFpgaTemperature);
    diagnostics_updater_.add(
      "bot_fpga_temperature", this,
      &RobosenseHwMonitorRosWrapper::RobosenseCheckBotFpgaTemperature);
    diagnostics_updater_.add(
      "gps_status", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckGpsStatus);
    if (info_model.find("RobosenseHelios") != std::string::npos) {
      diagnostics_updater_.add(
        "recv_a_temperature", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckRecvATemperature);
      diagnostics_updater_.add(
        "recv_b_temperature", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckRecvBTemperature);
      diagnostics_updater_.add(
        "main_fpga_core_temperature", this,
        &RobosenseHwMonitorRosWrapper::RobosenseCheckMainFpgaCoreTemperature);
      diagnostics_updater_.add(
        "lane_up", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckLaneUp);
      diagnostics_updater_.add(
        "lane_up_cnt", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckLaneUpCnt);
      diagnostics_updater_.add(
        "main_status", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckMainStatus);
    } else {
      diagnostics_updater_.add(
        "main_bot_temperature", this,
        &RobosenseHwMonitorRosWrapper::RobosenseCheckMainBotTemperature);
      diagnostics_updater_.add(
        "apd_temperature", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckApdTemperature);
    }
  } else if (info_model.find("RobosenseM1Plus") != std::string::npos) {
    diagnostics_updater_.add(
      "battery_volt", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckBatteryVolt);
    diagnostics_updater_.add(
      "lidar_fault_status", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckLidarFaultStatus);
    diagnostics_updater_.add(
      "lidar_roi_status", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckLidarRoiStatus);
  } else if (info_model.find("RobosenseE1") != std::string::npos) {
    diagnostics_updater_.add(
      "pl_vmon_vin_p", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckPlVmonVinP);
    diagnostics_updater_.add(
      "fpga_pl_kernal_temperature", this,
      &RobosenseHwMonitorRosWrapper::RobosenseCheckFpgaPlKernalTemperature);
    diagnostics_updater_.add(
      "fpga_ps_kernal_temperature", this,
      &RobosenseHwMonitorRosWrapper::RobosenseCheckFpgaPsKernalTemperature);
    diagnostics_updater_.add(
      "window_temperature", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckWindowTemperature);
  }
  current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetMotorRpm()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.rpm);

  std::string error_mes;

  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetBotFpgaTemperature()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.bot_fpga_temperature_val);
  std::string error_mes;

  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetRecvATemperature()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes =
    std::to_string(device_param_val.heliosDeviceCalculateParamsInfo.recv_A_temperature_val);
  std::string error_mes;

  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetRecvBTemperature()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes =
    std::to_string(device_param_val.heliosDeviceCalculateParamsInfo.recv_B_temperature_val);
  std::string error_mes;

  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetMainFpgaTemperature()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.main_fpga_temperature_val);
  std::string error_mes;

  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetMainCoreFpgaTemperature()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes =
    std::to_string(device_param_val.heliosDeviceCalculateParamsInfo.main_fpga_core_temperature_val);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetApdTemperature()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes =
    std::to_string(device_param_val.bpRubyDeviceCalculateParamsInfo.apd_temperature_val);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetMainBotTemperature()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes =
    std::to_string(device_param_val.bpRubyDeviceCalculateParamsInfo.main_bot_temperature_val);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetBatteryVolt()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.memsDeviceCalculateParamsInfo.battery_volt_val);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetLidarFaultStatus()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.lidar_fault_status);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetLidarRoiStatus()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.lidar_roi_status);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetPlVmonVinP()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes =
    std::to_string(device_param_val.flashDeviceCalculateParamsInfo.pl_vmon_vin_p_val);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetFpgaPlKernalTemperature()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.fpga_pl_kernal_temperature);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetFpgaPsKernalTemperature()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.fpga_ps_kernal_temperature);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetWindowTemperature()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.window_temperature);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetLaneUp()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.lane_up);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetLaneUpCnt()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.lane_up_cnt);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetMainStatus()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.main_status);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

std::tuple<bool, uint8_t, std::string, std::string>
RobosenseHwMonitorRosWrapper::RobosenseGetGpsStatus()
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes = std::to_string(device_param_val.gps_status);
  std::string error_mes;
  return std::make_tuple(not_ex, level, mes, error_mes);
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckRpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetMotorRpm();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Motor RPM", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckBotFpgaTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetBotFpgaTemperature();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Bot_Fpga_Temperature", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckRecvATemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetRecvATemperature();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Rec_A_Temperature", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckRecvBTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetRecvBTemperature();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Rec_B_Temperature", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckMainFpgaTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetMainFpgaTemperature();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Main_Fpga_Temperature", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckMainFpgaCoreTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetMainCoreFpgaTemperature();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Main_Fpga_Core_Temperature", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckLaneUp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetLaneUp();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Lane_Up", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckLaneUpCnt(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetLaneUpCnt();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Lane_Up_Cnt", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckMainStatus(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetMainStatus();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Main_Status", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckGpsStatus(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetGpsStatus();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Gps_Status", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckApdTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetApdTemperature();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Apd_Temperature", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckMainBotTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetMainBotTemperature();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Main_Bot_Temperature", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckBatteryVolt(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetBatteryVolt();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Battery_Voltage", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckLidarFaultStatus(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetLidarFaultStatus();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Lidar_Fault_Status", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckLidarRoiStatus(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetLidarRoiStatus();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("Lidar_Roi_Status", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckPlVmonVinP(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetPlVmonVinP();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("pl_vmon vin_p", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckFpgaPlKernalTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetFpgaPlKernalTemperature();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("fpga_pl_kernal_temperature", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckFpgaPsKernalTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetFpgaPsKernalTemperature();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("fpga_ps_kernal_temperature", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckWindowTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  auto tpl = RobosenseGetWindowTemperature();
  if (std::get<0>(tpl)) {
    level = std::max(level, std::get<1>(tpl));
    if (0 < std::get<3>(tpl).length()) {
      msg.emplace_back(std::get<3>(tpl));
    }
  }
  diagnostics.add("window_temperature", std::get<2>(tpl));
  diagnostics.summary(level, boost::algorithm::join(msg, ", "));
}

RCLCPP_COMPONENTS_REGISTER_NODE(RobosenseHwMonitorRosWrapper)
}  // namespace ros
}  // namespace nebula
