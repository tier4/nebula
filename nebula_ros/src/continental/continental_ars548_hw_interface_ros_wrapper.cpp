// Copyright 2023 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nebula_ros/continental/continental_ars548_hw_interface_ros_wrapper.hpp"

#include <chrono>
#include <thread>

namespace nebula
{
namespace ros
{
ContinentalARS548HwInterfaceRosWrapper::ContinentalARS548HwInterfaceRosWrapper(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("continental_ars548_hw_interface_ros_wrapper", options),
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
  std::shared_ptr<drivers::ContinentalARS548SensorConfiguration> sensor_cfg_ptr =
    std::make_shared<drivers::ContinentalARS548SensorConfiguration>(sensor_configuration_);
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  hw_interface_.RegisterScanCallback(std::bind(
    &ContinentalARS548HwInterfaceRosWrapper::ReceivePacketsDataCallback, this,
    std::placeholders::_1));
  packets_pub_ = this->create_publisher<nebula_msgs::msg::NebulaPackets>(
    "nebula_packets", rclcpp::SensorDataQoS());

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ContinentalARS548HwInterfaceRosWrapper::paramCallback, this, std::placeholders::_1));

  driving_direction_sub_ = create_subscription<std_msgs::msg::Bool>(
    "driving_direction", rclcpp::SensorDataQoS(),
    std::bind(
      &ContinentalARS548HwInterfaceRosWrapper::DrivingDirectionCallback, this,
      std::placeholders::_1));

  sensor_config_service_server_ = this->create_service<std_srvs::srv::Empty>(
    "configure_radar", std::bind(
                         &ContinentalARS548HwInterfaceRosWrapper::SensorConfigureRequestCallback,
                         this, std::placeholders::_1, std::placeholders::_2));

  StreamStart();
}

ContinentalARS548HwInterfaceRosWrapper::~ContinentalARS548HwInterfaceRosWrapper()
{
}

Status ContinentalARS548HwInterfaceRosWrapper::StreamStart()
{
  if (Status::OK == interface_status_) {
    interface_status_ = hw_interface_.CloudInterfaceStart();
    diagnostics_updater_.add(
      "radar_status", this, &ContinentalARS548HwInterfaceRosWrapper::ContinentalMonitorStatus);
  }

  return interface_status_;
}

Status ContinentalARS548HwInterfaceRosWrapper::StreamStop()
{
  return Status::OK;
}
Status ContinentalARS548HwInterfaceRosWrapper::Shutdown()
{
  return Status::OK;
}

Status ContinentalARS548HwInterfaceRosWrapper::InitializeHwInterface(  // todo: don't think
                                                                       // this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

Status ContinentalARS548HwInterfaceRosWrapper::GetParameters(
  drivers::ContinentalARS548SensorConfiguration & sensor_configuration)
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", descriptor);
    sensor_configuration.sensor_model =
      nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("host_ip", descriptor);
    sensor_configuration.host_ip = this->get_parameter("host_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_ip", descriptor);
    sensor_configuration.sensor_ip = this->get_parameter("sensor_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("multicast_ip", descriptor);
    sensor_configuration.multicast_ip = this->get_parameter("multicast_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("data_port", descriptor);
    sensor_configuration.data_port = this->get_parameter("data_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("configuration_host_port", descriptor);
    sensor_configuration.configuration_host_port =
      this->get_parameter("configuration_host_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("configuration_sensor_port", descriptor);
    sensor_configuration.configuration_sensor_port =
      this->get_parameter("configuration_sensor_port").as_int();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  if (sensor_configuration.frame_id.empty()) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void ContinentalARS548HwInterfaceRosWrapper::ReceivePacketsDataCallback(
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> scan_buffer)
{
  packets_pub_->publish(std::move(scan_buffer));
}

rcl_interfaces::msg::SetParametersResult ContinentalARS548HwInterfaceRosWrapper::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "add_on_set_parameters_callback");
  RCLCPP_DEBUG_STREAM(this->get_logger(), p);
  RCLCPP_DEBUG_STREAM(this->get_logger(), sensor_configuration_);
  RCLCPP_INFO_STREAM(this->get_logger(), p);

  drivers::ContinentalARS548SensorConfiguration new_param{sensor_configuration_};
  RCLCPP_INFO_STREAM(this->get_logger(), new_param);
  std::string sensor_model_str;
  std::string return_mode_str;
  if (
    get_param(p, "sensor_model", sensor_model_str) ||
    get_param(p, "return_mode", return_mode_str) || get_param(p, "host_ip", new_param.host_ip) ||
    get_param(p, "sensor_ip", new_param.sensor_ip) ||
    get_param(p, "frame_id", new_param.frame_id) ||
    get_param(p, "data_port", new_param.data_port) ||
    get_param(p, "configuration_host_port", new_param.configuration_host_port) ||
    get_param(p, "configuration_sensor_port", new_param.configuration_sensor_port)) {
    if (0 < sensor_model_str.length())
      new_param.sensor_model = nebula::drivers::SensorModelFromString(sensor_model_str);

    sensor_configuration_ = new_param;
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
      std::make_shared<drivers::ContinentalARS548SensorConfiguration>(sensor_configuration_);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "hw_interface_.SetSensorConfiguration");
    hw_interface_.SetSensorConfiguration(
      std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
    hw_interface_.CheckAndSetConfig();
  }

  auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  result->successful = true;
  result->reason = "success";

  RCLCPP_DEBUG_STREAM(this->get_logger(), "add_on_set_parameters_callback success");

  return *result;
}

void ContinentalARS548HwInterfaceRosWrapper::DrivingDirectionCallback(
  [[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
{
  hw_interface_.SetDrivingDirection(0);
  hw_interface_.SetAccelerationLateralCog(0.0);
  hw_interface_.SetAccelerationLongitudinalCog(0.0);

  hw_interface_.SetCharacteristicSpeed(0.0);
  hw_interface_.SetSteeringAngleFrontAxle(0.0);
  hw_interface_.SetVelocityVehicle(5.0);
  hw_interface_.SetYawRate(0.25);
}

void ContinentalARS548HwInterfaceRosWrapper::SensorConfigureRequestCallback(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  hw_interface_.SetVehicleParameters(10.f, 4.5f, 3.f, 2.5f);
}

std::vector<rcl_interfaces::msg::SetParametersResult>
ContinentalARS548HwInterfaceRosWrapper::updateParameters()
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "updateParameters start");
  std::ostringstream os_sensor_model;
  os_sensor_model << sensor_configuration_.sensor_model;
  RCLCPP_INFO_STREAM(this->get_logger(), "set_parameters");
  auto results = set_parameters(
    {rclcpp::Parameter("sensor_model", os_sensor_model.str()),
     rclcpp::Parameter("host_ip", sensor_configuration_.host_ip),
     rclcpp::Parameter("sensor_ip", sensor_configuration_.sensor_ip),
     rclcpp::Parameter("frame_id", sensor_configuration_.frame_id),
     rclcpp::Parameter("data_port", sensor_configuration_.data_port),
     rclcpp::Parameter("configuration_host_port", sensor_configuration_.configuration_host_port),
     rclcpp::Parameter(
       "configuration_sensor_port", sensor_configuration_.configuration_sensor_port)});
  RCLCPP_DEBUG_STREAM(this->get_logger(), "updateParameters end");
  return results;
}

void ContinentalARS548HwInterfaceRosWrapper::ContinentalMonitorStatus(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  auto sensor_status = hw_interface_.GetRadarStatus();
  diagnostics.add("timestamp_nanoseconds", std::to_string(sensor_status.timestamp_nanoseconds));
  diagnostics.add("timestamp_seconds", std::to_string(sensor_status.timestamp_seconds));
  diagnostics.add("timestamp_sync_status", sensor_status.timestamp_sync_status);
  diagnostics.add("sw_version_major", std::to_string(sensor_status.sw_version_major));
  diagnostics.add("sw_version_minor", std::to_string(sensor_status.sw_version_minor));
  diagnostics.add("sw_version_patch", std::to_string(sensor_status.sw_version_patch));
  diagnostics.add("longitudinal", std::to_string(sensor_status.longitudinal));
  diagnostics.add("lateral", std::to_string(sensor_status.lateral));
  diagnostics.add("vertical", std::to_string(sensor_status.vertical));
  diagnostics.add("yaw", std::to_string(sensor_status.yaw));
  diagnostics.add("pitch", std::to_string(sensor_status.pitch));
  diagnostics.add("plug_orientation", sensor_status.plug_orientation);
  diagnostics.add("length", std::to_string(sensor_status.length));
  diagnostics.add("width", std::to_string(sensor_status.width));
  diagnostics.add("height", std::to_string(sensor_status.height));
  diagnostics.add("wheel_base", std::to_string(sensor_status.wheel_base));
  diagnostics.add("max_distance", std::to_string(sensor_status.max_distance));
  diagnostics.add("frequency_slot", sensor_status.frequency_slot);
  diagnostics.add("cycle_time", std::to_string(sensor_status.cycle_time));
  diagnostics.add("time_slot", std::to_string(sensor_status.time_slot));
  diagnostics.add("hcc", sensor_status.hcc);
  diagnostics.add("power_save_standstill", sensor_status.power_save_standstill);
  diagnostics.add("sensor_ip_address0", sensor_status.sensor_ip_address0);
  diagnostics.add("sensor_ip_address1", sensor_status.sensor_ip_address1);
  diagnostics.add("configuration_counter", std::to_string(sensor_status.configuration_counter));
  diagnostics.add("status_longitudinal_velocity", sensor_status.status_longitudinal_velocity);
  diagnostics.add(
    "status_longitudinal_acceleration", sensor_status.status_longitudinal_acceleration);
  diagnostics.add("status_lateral_acceleration", sensor_status.status_lateral_acceleration);
  diagnostics.add("status_yaw_rate", sensor_status.status_yaw_rate);
  diagnostics.add("status_steering_angle", sensor_status.status_steering_angle);
  diagnostics.add("status_driving_direction", sensor_status.status_driving_direction);
  diagnostics.add("characteristic_speed", sensor_status.characteristic_speed);
  diagnostics.add("radar_status", sensor_status.radar_status);
  diagnostics.add("voltage_status", sensor_status.voltage_status);
  diagnostics.add("temperature_status", sensor_status.temperature_status);
  diagnostics.add("blockage_status", sensor_status.blockage_status);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ContinentalARS548HwInterfaceRosWrapper)
}  // namespace ros
}  // namespace nebula
