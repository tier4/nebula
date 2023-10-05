#include "nebula_ros/robosense/robosense_hw_monitor_ros_wrapper.hpp"

#include <memory>

namespace nebula
{
namespace ros
{
RobosenseHwMonitorRosWrapper::RobosenseHwMonitorRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("robosense_hw_monitor_ros_wrapper", options), diagnostics_updater_(this)
{
  interface_status_ = GetParameters(sensor_configuration_);
  if (Status::OK != interface_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
    return;
  }

  auto sensor_cfg_ptr =
    std::make_shared<drivers::RobosenseSensorConfiguration>(sensor_configuration_);

  info_driver_ = std::make_unique<drivers::RobosenseInfoDriver>(sensor_cfg_ptr);

  robosense_info_sub_ = create_subscription<robosense_msgs::msg::RobosensePacket>(
    "robosense_difop_packets", rclcpp::SensorDataQoS(),
    std::bind(&RobosenseHwMonitorRosWrapper::ReceiveInfoMsgCallback, this, std::placeholders::_1));

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RobosenseHwMonitorRosWrapper::paramCallback, this, std::placeholders::_1));
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
//
Status RobosenseHwMonitorRosWrapper::InitializeHwMonitor(
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
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
      this->get_parameter("return_mode").as_string());
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
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "pandar", descriptor);
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
    this->declare_parameter<uint16_t>("gnss_port", 2369, descriptor);
    sensor_configuration.gnss_port = this->get_parameter("gnss_port").as_int();
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
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    rcl_interfaces::msg::IntegerRange range;
    if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_BPEARL) {
      descriptor.additional_constraints = "300, 600, 1200";
      range.set__from_value(300).set__to_value(1200).set__step(300);
      descriptor.integer_range = {range};
      this->declare_parameter<uint16_t>("rotation_speed", 600, descriptor);
    } else if (
      sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_HELIOS_5515) {
      descriptor.additional_constraints = "600, 1200";
      range.set__from_value(600).set__to_value(1200).set__step(600);
      descriptor.integer_range = {range};
      this->declare_parameter<uint16_t>("rotation_speed", 600, descriptor);
    }
    sensor_configuration.rotation_speed = this->get_parameter("rotation_speed").as_int();
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

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360) {  // ||
    return Status::SENSOR_CONFIG_ERROR;
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "milliseconds";
    this->declare_parameter<uint16_t>("diag_span", 1000, descriptor);
    diag_span_ = this->get_parameter("diag_span").as_int();
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void RobosenseHwMonitorRosWrapper::InitializeRobosenseDiagnostics()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "InitializeRobosenseDiagnostics");
  diagnostics_updater_.setHardwareID(*hardware_id_);
  RCLCPP_INFO_STREAM(this->get_logger(), "hardware_id: " + *hardware_id_);

  diagnostics_updater_.add(
    "robosense_status", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckStatus);

  auto on_timer_status = [this] { OnRobosenseStatusTimer(); };
  diagnostics_status_timer_ =
    this->create_wall_timer(std::chrono::milliseconds(diag_span_), std::move(on_timer_status));

  RCLCPP_DEBUG_STREAM(get_logger(), "add_timer");
}

void RobosenseHwMonitorRosWrapper::OnRobosenseStatusTimer()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "OnRobosenseStatusTimer" << std::endl);
  info_driver_->DecodeInfoPacket(info_packet_buffer_);
  current_sensor_info_ = info_driver_->GetSensorInfo();
  current_info_time_ = std::make_unique<rclcpp::Time>(this->get_clock()->now());
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckStatus(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (!hardware_id_.has_value()) {
    return;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  for (const auto & info : current_sensor_info_) {
    diagnostics.add(info.first, info.second);
  }

  diagnostics.summary(level, "OK");
}

rcl_interfaces::msg::SetParametersResult RobosenseHwMonitorRosWrapper::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "add_on_set_parameters_callback");
  RCLCPP_DEBUG_STREAM(get_logger(), parameters);
  RCLCPP_DEBUG_STREAM(get_logger(), sensor_configuration_);
  RCLCPP_INFO_STREAM(this->get_logger(), parameters);

  drivers::RobosenseSensorConfiguration new_param{sensor_configuration_};
  RCLCPP_INFO_STREAM(this->get_logger(), new_param);
  uint16_t new_diag_span = 0;
  if (get_param(parameters, "diag_span", new_diag_span)) {
    sensor_configuration_ = new_param;
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
      std::make_shared<drivers::RobosenseSensorConfiguration>(sensor_configuration_);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "hw_interface_.SetSensorConfiguration");
  }

  auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  result->successful = true;
  result->reason = "success";

  RCLCPP_DEBUG_STREAM(this->get_logger(), "add_on_set_parameters_callback success");
  return *result;
}

void RobosenseHwMonitorRosWrapper::ReceiveInfoMsgCallback(
  const robosense_msgs::msg::RobosensePacket::SharedPtr info_msg)
{
  info_packet_buffer_ = std::vector<uint8_t>(info_msg->data.begin(), info_msg->data.end());

  if (!hardware_id_.has_value()) {
    info_driver_->DecodeInfoPacket(info_packet_buffer_);
    current_sensor_info_ = info_driver_->GetSensorInfo();
    current_info_time_ = std::make_unique<rclcpp::Time>(this->get_clock()->now());

    RCLCPP_INFO_STREAM(this->get_logger(), "Model:" << sensor_configuration_.sensor_model);
    RCLCPP_INFO_STREAM(this->get_logger(), "Serial:" << current_sensor_info_["serial_number"]);

    hardware_id_.emplace(
      nebula::drivers::SensorModelToString(sensor_configuration_.sensor_model) + ": " +
      current_sensor_info_["serial_number"]);
    InitializeRobosenseDiagnostics();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RobosenseHwMonitorRosWrapper)
}  // namespace ros
}  // namespace nebula
