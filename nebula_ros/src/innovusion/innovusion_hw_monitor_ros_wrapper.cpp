#include "nebula_ros/innovusion/innovusion_hw_monitor_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
InnovusionHwMonitorRosWrapper::InnovusionHwMonitorRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("Innovusion_hw_monitor_ros_wrapper", options),
  hw_interface_(),
  diagnostics_updater_(this)
{
  cbg_r_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cbg_m_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  interface_status_ = GetParameters(sensor_configuration_);
  if (Status::OK != interface_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
    return;
  }
  InitializeHwMonitor(sensor_configuration_);

  // Initialize sensor_configuration
  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialize sensor_configuration");
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::InnovusionSensorConfiguration>(sensor_configuration_);
  RCLCPP_INFO_STREAM(this->get_logger(), "hw_interface_.InitializeSensorConfiguration");
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  // Initialize diagnostics updater
  InitializeInnovusionDiagnostics();
}

Status InnovusionHwMonitorRosWrapper::MonitorStart() { return interface_status_; }

Status InnovusionHwMonitorRosWrapper::MonitorStop() { return Status::OK; }
Status InnovusionHwMonitorRosWrapper::Shutdown() { return Status::OK; }

Status InnovusionHwMonitorRosWrapper::InitializeHwMonitor(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{

  switch (sensor_configuration.sensor_model) {
    case nebula::drivers::SensorModel::INNOVUSION_ROBIN:
      key_lidar_info_ = ":get-lidar-info";
      key_lidar_snapshot_ = ":get-lidar-status";
      key_lidar_rpm_ = "motor.polygon_speed";
      key_laser_voltage_ = "laser_info.laser_5V";
      key_lidar_up_time_ = "uptime";
      key_det_temp_ = "temperature.det";
      key_laser_temp_ = "temperature.laser";
      key_lidar_sn_ = "sn";
      break;
    case nebula::drivers::SensorModel::INNOVUSION_FALCON:
      key_lidar_info_ = ":get-lidar-info";
      key_lidar_snapshot_ = ":get_lidar_status";
      key_lidar_rpm_ = "motor_status.polygon_speed";
      key_laser_voltage_ = "laser_status.pump_voltage";
      key_lidar_up_time_ = "uptime";
      key_det_temp_ = "temperature.det";
      key_laser_temp_ = "temperature.laser";
      key_lidar_sn_ = "sn";
      break;
    default:
      return Status::INVALID_SENSOR_MODEL;
      break;
  }
  return Status::OK;
}

void InnovusionHwMonitorRosWrapper::InitializeInnovusionDiagnostics()
{ 
  hw_interface_.GetSnapshotAsync([this](const std::string & str) {
    lidar_info_ =
      std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
    try {
      std::string info_serial = GetPtreeValue(lidar_info_, key_lidar_sn_);
      auto hardware_id = "innovusion:" + info_serial;
      diagnostics_updater_.setHardwareID(hardware_id);
    } catch (boost::bad_lexical_cast & ex) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), this->get_name() << " Error:"
                                             << "Can't get lidar sn");
      return;
    }
  }, key_lidar_info_);

    // get lidar snapshot
  hw_interface_.GetSnapshotAsync([this](const std::string & str) {
    current_snapshot_time_.reset(new rclcpp::Time(this->get_clock()->now()));
    current_snapshot_ =
      std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
  }, key_lidar_snapshot_);

  diagnostics_updater_.add(
    "innovusion_motor", this, &InnovusionHwMonitorRosWrapper::InnovusionCheckMotorRpm);
  diagnostics_updater_.add(
    "innovusion_laser_voltage", this, &InnovusionHwMonitorRosWrapper::InnovusionCheckLaserVoltage);
  diagnostics_updater_.add(
    "innovusion_lidar_uptime", this, &InnovusionHwMonitorRosWrapper::InnovusionCheckUptime);
  diagnostics_updater_.add(
    "innovusion_det_temp", this, &InnovusionHwMonitorRosWrapper::InnovusionCheckDetTemp);
  diagnostics_updater_.add(
    "innovusion_laser_temp", this, &InnovusionHwMonitorRosWrapper::InnovusionCheckLaserTemp);

  // get time snapshot cycle
  auto on_timer_snapshot = [this] { OnInnovusionSnapshotTimer(); };
  diagnostics_snapshot_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_snapshot)>>(
    this->get_clock(), std::chrono::milliseconds(diag_span_), std::move(on_timer_snapshot),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(diagnostics_snapshot_timer_, cbg_m_);

  auto on_timer_update = [this] {
    auto now = this->get_clock()->now();
    auto dif = (now - *current_snapshot_time_).seconds();
    if (diag_span_ * 2.0 < dif * 1000) {
      current_diag_status_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      RCLCPP_DEBUG_STREAM(get_logger(), "STALE");
    } else {
      current_diag_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
      RCLCPP_DEBUG_STREAM(get_logger(), "OK");
    }
    diagnostics_updater_.force_update();
  };
  diagnostics_update_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_update)>>(
    this->get_clock(), std::chrono::milliseconds(1000), std::move(on_timer_update),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(diagnostics_update_timer_, cbg_r_);
}

void InnovusionHwMonitorRosWrapper::OnInnovusionSnapshotTimer()
{
  // get lidar snapshot
  hw_interface_.GetSnapshotAsync([this](const std::string & str) {
    current_snapshot_time_.reset(new rclcpp::Time(this->get_clock()->now()));
    current_snapshot_ =
      std::make_shared<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
  }, key_lidar_snapshot_);
}

Status InnovusionHwMonitorRosWrapper::GetParameters(
  drivers::InnovusionSensorConfiguration & sensor_configuration)
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
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("return_mode", "", descriptor);
    sensor_configuration.return_mode =
      nebula::drivers::ReturnModeFromString(this->get_parameter("return_mode").as_string());
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
    this->declare_parameter<std::string>("sensor_ip", "192.168.1.201", descriptor);
    sensor_configuration.sensor_ip = this->get_parameter("sensor_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "innovusion", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("data_port", 8010, descriptor);
    sensor_configuration.data_port = this->get_parameter("data_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "milliseconds";
    this->declare_parameter<uint16_t>("diag_span", 1000, descriptor);
    this->diag_span_ = this->get_parameter("diag_span").as_int();
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

std::tuple<bool, uint8_t, std::string, std::string>
InnovusionHwMonitorRosWrapper::InnovusionCommonGet(
  std::shared_ptr<boost::property_tree::ptree> pt,std::string key)
{
  bool not_ex = true;
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string mes;
  std::string error_mes;
  try {
    mes = GetPtreeValue(pt, key);
  } catch (boost::bad_lexical_cast & ex) {
    not_ex = false;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    mes = "Error";
  }
  return std::make_tuple(not_ex, level, mes, error_mes);
}

void InnovusionHwMonitorRosWrapper::InnovusionCheckMotorRpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    InnovusionHwMonitorRosWrapper::current_snapshot_ &&
    !InnovusionHwMonitorRosWrapper::current_snapshot_->empty()) {
    auto tpl = InnovusionCommonGet(current_snapshot_, key_lidar_rpm_);
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void InnovusionHwMonitorRosWrapper::InnovusionCheckLaserVoltage(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    InnovusionHwMonitorRosWrapper::current_snapshot_ &&
    !InnovusionHwMonitorRosWrapper::current_snapshot_->empty()) {
    auto tpl = InnovusionCommonGet(current_snapshot_, key_laser_voltage_);
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void InnovusionHwMonitorRosWrapper::InnovusionCheckUptime(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    InnovusionHwMonitorRosWrapper::current_snapshot_ &&
    !InnovusionHwMonitorRosWrapper::current_snapshot_->empty()) {
    auto tpl = InnovusionCommonGet(current_snapshot_, key_lidar_up_time_);
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void InnovusionHwMonitorRosWrapper::InnovusionCheckDetTemp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    InnovusionHwMonitorRosWrapper::current_snapshot_ &&
    !InnovusionHwMonitorRosWrapper::current_snapshot_->empty()) {
    auto tpl = InnovusionCommonGet(current_snapshot_, key_det_temp_);
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

void InnovusionHwMonitorRosWrapper::InnovusionCheckLaserTemp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (
    InnovusionHwMonitorRosWrapper::current_snapshot_ &&
    !InnovusionHwMonitorRosWrapper::current_snapshot_->empty()) {
    auto tpl = InnovusionCommonGet(current_snapshot_, key_laser_temp_);
    diagnostics.add("sensor", sensor_configuration_.frame_id);
    diagnostics.summary(std::get<1>(tpl), std::get<2>(tpl));
  }
}

std::string InnovusionHwMonitorRosWrapper::GetPtreeValue(
  std::shared_ptr<boost::property_tree::ptree> pt, const std::string & key)
{
  boost::optional<std::string> value = pt->get_optional<std::string>(key);
  if (value) {
    return value.get();
  } else {
    return "Not support";
  }
}

InnovusionHwMonitorRosWrapper::~InnovusionHwMonitorRosWrapper() {
  RCLCPP_INFO_STREAM(get_logger(), "Closing TcpDriver");
}

RCLCPP_COMPONENTS_REGISTER_NODE(InnovusionHwMonitorRosWrapper)
}  // namespace ros
}  // namespace nebula
