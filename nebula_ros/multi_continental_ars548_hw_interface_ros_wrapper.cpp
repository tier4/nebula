// Copyright 2024 TIER IV, Inc.
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

#include <nebula_ros/continental/multi_continental_ars548_hw_interface_ros_wrapper.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <thread>

namespace nebula
{
namespace ros
{
MultiContinentalArs548HwInterfaceRosWrapper::MultiContinentalArs548HwInterfaceRosWrapper(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("multi_continental_ars548_hw_interface_ros_wrapper", options), hw_interface_()
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
  std::shared_ptr<drivers::continental_ars548::MultiContinentalArs548SensorConfiguration>
    sensor_cfg_ptr =
      std::make_shared<drivers::continental_ars548::MultiContinentalArs548SensorConfiguration>(
        sensor_configuration_);
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  assert(sensor_configuration_.sensor_ips.size() == sensor_configuration_.frame_ids.size());
  hw_interface_.RegisterScanCallback(std::bind(
    &MultiContinentalArs548HwInterfaceRosWrapper::ReceivePacketsDataCallback, this,
    std::placeholders::_1, std::placeholders::_2));

  for (std::size_t sensor_id = 0; sensor_id < sensor_configuration_.sensor_ips.size();
       sensor_id++) {
    const std::string sensor_ip = sensor_configuration_.sensor_ips[sensor_id];
    const std::string frame_id = sensor_configuration_.frame_ids[sensor_id];
    packets_pub_map_[sensor_ip] = this->create_publisher<nebula_msgs::msg::NebulaPackets>(
      frame_id + "/nebula_packets", rclcpp::SensorDataQoS());
  }

  set_param_res_ = this->add_on_set_parameters_callback(std::bind(
    &MultiContinentalArs548HwInterfaceRosWrapper::paramCallback, this, std::placeholders::_1));

  StreamStart();
}

MultiContinentalArs548HwInterfaceRosWrapper::~MultiContinentalArs548HwInterfaceRosWrapper()
{
}

Status MultiContinentalArs548HwInterfaceRosWrapper::StreamStart()
{
  if (Status::OK == interface_status_) {
    interface_status_ = hw_interface_.SensorInterfaceStart();
  }

  if (Status::OK == interface_status_) {
    odometry_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "odometry_input", rclcpp::QoS{1},
      std::bind(
        &MultiContinentalArs548HwInterfaceRosWrapper::OdometryCallback, this,
        std::placeholders::_1));

    acceleration_sub_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "acceleration_input", rclcpp::QoS{1},
      std::bind(
        &MultiContinentalArs548HwInterfaceRosWrapper::AccelerationCallback, this,
        std::placeholders::_1));

    steering_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
      "steering_angle_input", rclcpp::QoS{1},
      std::bind(
        &MultiContinentalArs548HwInterfaceRosWrapper::SteeringAngleCallback, this,
        std::placeholders::_1));
  }

  return interface_status_;
}

Status MultiContinentalArs548HwInterfaceRosWrapper::StreamStop()
{
  return Status::OK;
}
Status MultiContinentalArs548HwInterfaceRosWrapper::Shutdown()
{
  return Status::OK;
}

Status MultiContinentalArs548HwInterfaceRosWrapper::InitializeHwInterface(  // todo: don't think
                                                                            // this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

Status MultiContinentalArs548HwInterfaceRosWrapper::GetParameters(
  drivers::continental_ars548::MultiContinentalArs548SensorConfiguration & sensor_configuration)
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
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::vector<std::string>>("sensor_ips", descriptor);
    sensor_configuration.sensor_ips = this->get_parameter("sensor_ips").as_string_array();
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
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::vector<std::string>>("frame_ids", descriptor);
    sensor_configuration.frame_ids = this->get_parameter("frame_ids").as_string_array();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("base_frame", descriptor);
    sensor_configuration.base_frame = this->get_parameter("base_frame").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("object_frame", descriptor);
    sensor_configuration.object_frame = this->get_parameter("object_frame").as_string();
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
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<bool>("use_sensor_time", descriptor);
    sensor_configuration.use_sensor_time = this->get_parameter("use_sensor_time").as_bool();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void MultiContinentalArs548HwInterfaceRosWrapper::ReceivePacketsDataCallback(
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> scan_buffer, const std::string & sensor_ip)
{
  packets_pub_map_[sensor_ip]->publish(std::move(scan_buffer));
}

rcl_interfaces::msg::SetParametersResult MultiContinentalArs548HwInterfaceRosWrapper::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "add_on_set_parameters_callback");
  RCLCPP_DEBUG_STREAM(this->get_logger(), p);
  RCLCPP_DEBUG_STREAM(this->get_logger(), sensor_configuration_);
  RCLCPP_INFO_STREAM(this->get_logger(), p);

  drivers::continental_ars548::MultiContinentalArs548SensorConfiguration new_param{
    sensor_configuration_};
  RCLCPP_INFO_STREAM(this->get_logger(), new_param);
  std::string sensor_model_str;

  if (
    get_param(p, "sensor_model", sensor_model_str) | get_param(p, "host_ip", new_param.host_ip) |
    get_param(p, "sensor_ips", new_param.sensor_ips) |
    get_param(p, "frame_ids", new_param.frame_ids) |
    get_param(p, "data_port", new_param.data_port) |
    get_param(p, "multicast_ip", new_param.multicast_ip) |
    get_param(p, "base_frame", new_param.base_frame) |
    get_param(p, "object_frame", new_param.object_frame) |
    get_param(p, "configuration_host_port", new_param.configuration_host_port) |
    get_param(p, "configuration_sensor_port", new_param.configuration_sensor_port) |
    get_param(p, "configuration_host_port", new_param.configuration_host_port) |
    get_param(p, "configuration_sensor_port", new_param.configuration_sensor_port)) {
    if (0 < sensor_model_str.length())
      new_param.sensor_model = nebula::drivers::SensorModelFromString(sensor_model_str);

    sensor_configuration_ = new_param;
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
      std::make_shared<drivers::continental_ars548::ContinentalArs548SensorConfiguration>(
        sensor_configuration_);
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

void MultiContinentalArs548HwInterfaceRosWrapper::OdometryCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  std::scoped_lock lock(mtx_config_);

  constexpr float speed_to_standstill = 0.5f;
  constexpr float speed_to_moving = 2.f;

  if (standstill_ && std::abs(msg->twist.twist.linear.x) > speed_to_moving) {
    standstill_ = false;
  } else if (!standstill_ && std::abs(msg->twist.twist.linear.x) < speed_to_standstill) {
    standstill_ = true;
  }

  if (standstill_) {
    hw_interface_.SetDrivingDirection(0);
  } else {
    hw_interface_.SetDrivingDirection(msg->twist.twist.linear.x > 0.f ? 1 : -1);
  }

  constexpr float ms_to_kmh = 3.6f;
  hw_interface_.SetVelocityVehicle(ms_to_kmh * std::abs(msg->twist.twist.linear.x));

  constexpr float rad_to_deg = 180.f / M_PI;
  hw_interface_.SetYawRate(rad_to_deg * msg->twist.twist.angular.z);
}

void MultiContinentalArs548HwInterfaceRosWrapper::AccelerationCallback(
  const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  std::scoped_lock lock(mtx_config_);
  hw_interface_.SetAccelerationLateralCog(msg->accel.accel.linear.y);
  hw_interface_.SetAccelerationLongitudinalCog(msg->accel.accel.linear.x);
}

void MultiContinentalArs548HwInterfaceRosWrapper::SteeringAngleCallback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  std::scoped_lock lock(mtx_config_);
  constexpr float rad_to_deg = 180.f / M_PI;
  hw_interface_.SetSteeringAngleFrontAxle(rad_to_deg * msg->data);
}

std::vector<rcl_interfaces::msg::SetParametersResult>
MultiContinentalArs548HwInterfaceRosWrapper::updateParameters()
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "updateParameters start");
  std::ostringstream os_sensor_model;
  os_sensor_model << sensor_configuration_.sensor_model;
  RCLCPP_INFO_STREAM(this->get_logger(), "set_parameters");
  auto results = set_parameters(
    {rclcpp::Parameter("sensor_model", os_sensor_model.str()),
     rclcpp::Parameter("host_ip", sensor_configuration_.host_ip),
     rclcpp::Parameter("sensor_ips", sensor_configuration_.sensor_ips),
     rclcpp::Parameter("frame_ids", sensor_configuration_.frame_ids),
     rclcpp::Parameter("data_port", sensor_configuration_.data_port),
     rclcpp::Parameter("multicast_ip", sensor_configuration_.multicast_ip),
     rclcpp::Parameter("base_frame", sensor_configuration_.base_frame),
     rclcpp::Parameter("object_frame", sensor_configuration_.object_frame),
     rclcpp::Parameter("configuration_host_port", sensor_configuration_.configuration_host_port),
     rclcpp::Parameter(
       "configuration_sensor_port", sensor_configuration_.configuration_sensor_port)});
  RCLCPP_DEBUG_STREAM(this->get_logger(), "updateParameters end");
  return results;
}

RCLCPP_COMPONENTS_REGISTER_NODE(MultiContinentalArs548HwInterfaceRosWrapper)
}  // namespace ros
}  // namespace nebula
