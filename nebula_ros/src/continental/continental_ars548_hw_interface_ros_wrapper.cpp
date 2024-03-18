// Copyright 2024 Tier IV, Inc.
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

#include <nebula_ros/continental/continental_ars548_hw_interface_ros_wrapper.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <thread>

namespace nebula
{
namespace ros
{
ContinentalARS548HwInterfaceRosWrapper::ContinentalARS548HwInterfaceRosWrapper(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("continental_ars548_hw_interface_ros_wrapper", options), hw_interface_()
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
  std::shared_ptr<drivers::continental_ars548::ContinentalARS548SensorConfiguration>
    sensor_cfg_ptr =
      std::make_shared<drivers::continental_ars548::ContinentalARS548SensorConfiguration>(
        sensor_configuration_);
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  hw_interface_.RegisterScanCallback(std::bind(
    &ContinentalARS548HwInterfaceRosWrapper::ReceivePacketsDataCallback, this,
    std::placeholders::_1));
  packets_pub_ = this->create_publisher<nebula_msgs::msg::NebulaPackets>(
    "nebula_packets", rclcpp::SensorDataQoS());

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ContinentalARS548HwInterfaceRosWrapper::paramCallback, this, std::placeholders::_1));

  StreamStart();
}

ContinentalARS548HwInterfaceRosWrapper::~ContinentalARS548HwInterfaceRosWrapper()
{
}

Status ContinentalARS548HwInterfaceRosWrapper::StreamStart()
{
  if (Status::OK == interface_status_) {
    interface_status_ = hw_interface_.SensorInterfaceStart();
  }

  if (Status::OK == interface_status_) {
    odometry_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "odometry_input", rclcpp::QoS{1},
      std::bind(
        &ContinentalARS548HwInterfaceRosWrapper::OdometryCallback, this, std::placeholders::_1));

    acceleration_sub_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "acceleration_input", rclcpp::QoS{1},
      std::bind(
        &ContinentalARS548HwInterfaceRosWrapper::AccelerationCallback, this,
        std::placeholders::_1));

    steering_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
      "steering_angle_input", rclcpp::SensorDataQoS(),
      std::bind(
        &ContinentalARS548HwInterfaceRosWrapper::SteeringAngleCallback, this,
        std::placeholders::_1));

    set_network_configuration_service_server_ =
      this->create_service<continental_srvs::srv::ContinentalArs548SetNetworkConfiguration>(
        "set_network_configuration",
        std::bind(
          &ContinentalARS548HwInterfaceRosWrapper::SetNetworkConfigurationRequestCallback, this,
          std::placeholders::_1, std::placeholders::_2));

    set_sensor_mounting_service_server_ =
      this->create_service<continental_srvs::srv::ContinentalArs548SetSensorMounting>(
        "set_sensor_mounting",
        std::bind(
          &ContinentalARS548HwInterfaceRosWrapper::SetSensorMountingRequestCallback, this,
          std::placeholders::_1, std::placeholders::_2));

    set_vehicle_parameters_service_server_ =
      this->create_service<continental_srvs::srv::ContinentalArs548SetVehicleParameters>(
        "set_vehicle_parameters",
        std::bind(
          &ContinentalARS548HwInterfaceRosWrapper::SetVehicleParametersRequestCallback, this,
          std::placeholders::_1, std::placeholders::_2));

    set_radar_parameters_service_server_ =
      this->create_service<continental_srvs::srv::ContinentalArs548SetRadarParameters>(
        "set_radar_parameters",
        std::bind(
          &ContinentalARS548HwInterfaceRosWrapper::SetRadarParametersRequestCallback, this,
          std::placeholders::_1, std::placeholders::_2));
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
  drivers::continental_ars548::ContinentalARS548SensorConfiguration & sensor_configuration)
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
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("configuration_vehicle_length", descriptor);
    sensor_configuration.configuration_vehicle_length =
      static_cast<float>(this->get_parameter("configuration_vehicle_length").as_double());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("configuration_vehicle_width", descriptor);
    sensor_configuration.configuration_vehicle_width =
      static_cast<float>(this->get_parameter("configuration_vehicle_width").as_double());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("configuration_vehicle_height", descriptor);
    sensor_configuration.configuration_vehicle_height =
      static_cast<float>(this->get_parameter("configuration_vehicle_height").as_double());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("configuration_vehicle_wheelbase", descriptor);
    sensor_configuration.configuration_vehicle_wheelbase =
      static_cast<float>(this->get_parameter("configuration_vehicle_wheelbase").as_double());
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
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

  drivers::continental_ars548::ContinentalARS548SensorConfiguration new_param{
    sensor_configuration_};
  RCLCPP_INFO_STREAM(this->get_logger(), new_param);
  std::string sensor_model_str;

  if (
    get_param(p, "sensor_model", sensor_model_str) | get_param(p, "host_ip", new_param.host_ip) |
    get_param(p, "sensor_ip", new_param.sensor_ip) | get_param(p, "frame_id", new_param.frame_id) |
    get_param(p, "data_port", new_param.data_port) |
    get_param(p, "multicast_ip", new_param.multicast_ip) |
    get_param(p, "base_frame", new_param.base_frame) |
    get_param(p, "object_frame", new_param.object_frame) |
    get_param(p, "configuration_host_port", new_param.configuration_host_port) |
    get_param(p, "configuration_sensor_port", new_param.configuration_sensor_port) |
    get_param(p, "configuration_vehicle_length", new_param.configuration_vehicle_length) |
    get_param(p, "configuration_vehicle_width", new_param.configuration_vehicle_width) |
    get_param(p, "configuration_vehicle_height", new_param.configuration_vehicle_height) |
    get_param(p, "configuration_vehicle_wheelbase", new_param.configuration_vehicle_wheelbase) |
    get_param(p, "configuration_host_port", new_param.configuration_host_port) |
    get_param(p, "configuration_sensor_port", new_param.configuration_sensor_port)) {
    if (0 < sensor_model_str.length())
      new_param.sensor_model = nebula::drivers::SensorModelFromString(sensor_model_str);

    sensor_configuration_ = new_param;
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
      std::make_shared<drivers::continental_ars548::ContinentalARS548SensorConfiguration>(
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

void ContinentalARS548HwInterfaceRosWrapper::OdometryCallback(
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

void ContinentalARS548HwInterfaceRosWrapper::AccelerationCallback(
  const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  std::scoped_lock lock(mtx_config_);
  hw_interface_.SetAccelerationLateralCog(msg->accel.accel.linear.y);
  hw_interface_.SetAccelerationLongitudinalCog(msg->accel.accel.linear.x);
}

void ContinentalARS548HwInterfaceRosWrapper::SteeringAngleCallback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  std::scoped_lock lock(mtx_config_);
  constexpr float rad_to_deg = 180.f / M_PI;
  hw_interface_.SetSteeringAngleFrontAxle(rad_to_deg * msg->data);
}

void ContinentalARS548HwInterfaceRosWrapper::SetNetworkConfigurationRequestCallback(
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetNetworkConfiguration::Request>
    request,
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetNetworkConfiguration::Response>
    response)
{
  std::scoped_lock lock(mtx_config_);
  auto result = hw_interface_.SetSensorIPAddress(request->sensor_ip.data);
  response->success = result == Status::OK;
  response->message = (std::stringstream() << result).str();
}

void ContinentalARS548HwInterfaceRosWrapper::SetSensorMountingRequestCallback(
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetSensorMounting::Request> request,
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetSensorMounting::Response>
    response)
{
  std::scoped_lock lock(mtx_config_);

  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  auto tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  float longitudinal = request->longitudinal;
  float lateral = request->lateral;
  float vertical = request->vertical;
  float yaw = request->yaw;
  float pitch = request->pitch;

  if (request->autoconfigure_extrinsics) {
    RCLCPP_INFO(
      this->get_logger(),
      "autoconfigure_extrinsics was set to true, so the mounting position will be derived from the "
      "tfs");

    geometry_msgs::msg::TransformStamped base_to_sensor_tf;
    try {
      base_to_sensor_tf = tf_buffer->lookupTransform(
        sensor_configuration_.base_frame, sensor_configuration_.frame_id, rclcpp::Time(0),
        rclcpp::Duration::from_seconds(0.5));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not obtain the transform from the base frame to %s (%s)",
        sensor_configuration_.frame_id.c_str(), ex.what());
      response->success = false;
      response->message = ex.what();
      return;
    }

    const auto & quat = base_to_sensor_tf.transform.rotation;
    geometry_msgs::msg::Vector3 rpy;
    tf2::Matrix3x3(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(rpy.x, rpy.y, rpy.z);

    longitudinal = base_to_sensor_tf.transform.translation.x -
                   sensor_configuration_.configuration_vehicle_wheelbase;
    lateral = base_to_sensor_tf.transform.translation.y;
    vertical = base_to_sensor_tf.transform.translation.z;
    yaw = rpy.z;
    pitch = rpy.y;
  }

  auto result = hw_interface_.SetSensorMounting(
    longitudinal, lateral, vertical, yaw, pitch, request->plug_orientation);

  response->success = result == Status::OK;
  response->message = (std::stringstream() << result).str();
}

void ContinentalARS548HwInterfaceRosWrapper::SetVehicleParametersRequestCallback(
  [[maybe_unused]] const std::shared_ptr<
    continental_srvs::srv::ContinentalArs548SetVehicleParameters::Request>
    request,
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetVehicleParameters::Response>
    response)
{
  float vehicle_length = request->vehicle_length;
  float vehicle_width = request->vehicle_width;
  float vehicle_height = request->vehicle_height;
  float vehicle_wheelbase = request->vehicle_wheelbase;

  if (vehicle_length < 0.f) {
    RCLCPP_INFO(
      this->get_logger(),
      "Service vehicle_length is invalid. Falling back to configuration value (%.2f)",
      sensor_configuration_.configuration_vehicle_length);
    vehicle_length = sensor_configuration_.configuration_vehicle_length;
  }

  if (vehicle_width < 0.f) {
    RCLCPP_INFO(
      this->get_logger(),
      "Service vehicle_width is invalid. Falling back to configuration value (%.2f)",
      sensor_configuration_.configuration_vehicle_width);
    vehicle_width = sensor_configuration_.configuration_vehicle_width;
  }

  if (vehicle_height < 0.f) {
    RCLCPP_INFO(
      this->get_logger(),
      "Service vehicle_height is invalid. Falling back to configuration value (%.2f)",
      sensor_configuration_.configuration_vehicle_height);
    vehicle_height = sensor_configuration_.configuration_vehicle_height;
  }

  if (vehicle_wheelbase < 0.f) {
    RCLCPP_INFO(
      this->get_logger(),
      "Service vehicle_wheelbase is invalid. Falling back to configuration value (%.2f)",
      sensor_configuration_.configuration_vehicle_wheelbase);
    vehicle_wheelbase = sensor_configuration_.configuration_vehicle_wheelbase;
  }

  std::scoped_lock lock(mtx_config_);
  auto result = hw_interface_.SetVehicleParameters(
    vehicle_length, vehicle_width, vehicle_height, vehicle_wheelbase);

  response->success = result == Status::OK;
  response->message = (std::stringstream() << result).str();
}

void ContinentalARS548HwInterfaceRosWrapper::SetRadarParametersRequestCallback(
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetRadarParameters::Request>
    request,
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetRadarParameters::Response>
    response)
{
  std::scoped_lock lock(mtx_config_);
  auto result = hw_interface_.SetRadarParameters(
    request->maximum_distance, request->frequency_slot, request->cycle_time, request->time_slot,
    request->country_code, request->powersave_standstill);

  response->success = result == Status::OK;
  response->message = (std::stringstream() << result).str();
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
     rclcpp::Parameter("multicast_ip", sensor_configuration_.multicast_ip),
     rclcpp::Parameter("base_frame", sensor_configuration_.base_frame),
     rclcpp::Parameter("object_frame", sensor_configuration_.object_frame),
     rclcpp::Parameter("configuration_host_port", sensor_configuration_.configuration_host_port),
     rclcpp::Parameter(
       "configuration_sensor_port", sensor_configuration_.configuration_sensor_port),
     rclcpp::Parameter(
       "configuration_vehicle_length", sensor_configuration_.configuration_vehicle_length),
     rclcpp::Parameter(
       "configuration_vehicle_width", sensor_configuration_.configuration_vehicle_width),
     rclcpp::Parameter(
       "configuration_vehicle_height", sensor_configuration_.configuration_vehicle_height),
     rclcpp::Parameter(
       "configuration_vehicle_wheelbase", sensor_configuration_.configuration_vehicle_wheelbase)});
  RCLCPP_DEBUG_STREAM(this->get_logger(), "updateParameters end");
  return results;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ContinentalARS548HwInterfaceRosWrapper)
}  // namespace ros
}  // namespace nebula
