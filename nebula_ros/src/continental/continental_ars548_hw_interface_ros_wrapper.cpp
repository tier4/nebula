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

  odometry_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "odometry_input", rclcpp::QoS{1},
    std::bind(
      &ContinentalARS548HwInterfaceRosWrapper::OdometryCallback, this, std::placeholders::_1));

  acceleration_sub_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "acceleration_input", rclcpp::QoS{1},
    std::bind(
      &ContinentalARS548HwInterfaceRosWrapper::AccelerationCallback, this, std::placeholders::_1));

  steering_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
    "steering_angle_input", rclcpp::QoS{1},
    std::bind(
      &ContinentalARS548HwInterfaceRosWrapper::SteeringAngleCallback, this, std::placeholders::_1));

  set_new_sensor_ip_service_server_ = this->create_service<std_srvs::srv::Empty>(
    "set_new_sensor_ip", std::bind(
                           &ContinentalARS548HwInterfaceRosWrapper::SetNewSensorIPRequestCallback,
                           this, std::placeholders::_1, std::placeholders::_2));

  set_new_sensor_mounting_service_server_ = this->create_service<std_srvs::srv::Empty>(
    "set_new_sensor_mounting",
    std::bind(
      &ContinentalARS548HwInterfaceRosWrapper::SetNewSensorMountingRequestCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  set_new_vehicle_parameters_service_server_ = this->create_service<std_srvs::srv::Empty>(
    "set_new_vehicle_parameters",
    std::bind(
      &ContinentalARS548HwInterfaceRosWrapper::SetNewVehicleParametersRequestCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  set_new_radar_parameters_service_server_ = this->create_service<std_srvs::srv::Empty>(
    "set_new_radar_parameters",
    std::bind(
      &ContinentalARS548HwInterfaceRosWrapper::SetNewRadarParametersRequestCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  StreamStart();
}

ContinentalARS548HwInterfaceRosWrapper::~ContinentalARS548HwInterfaceRosWrapper()
{
}

Status ContinentalARS548HwInterfaceRosWrapper::StreamStart()
{
  if (Status::OK == interface_status_) {
    interface_status_ = hw_interface_.CloudInterfaceStart();

    std::scoped_lock lock(mtx_config_);
    diagnostics_updater_.add(
      "radar_status(" + sensor_configuration_.frame_id + ")", this,
      &ContinentalARS548HwInterfaceRosWrapper::ContinentalMonitorStatus);
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
    this->declare_parameter<std::string>("new_sensor_ip", descriptor);
    sensor_configuration.new_sensor_ip = this->get_parameter("new_sensor_ip").as_string();
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
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("new_plug_orientation", descriptor);
    sensor_configuration.new_plug_orientation =
      this->get_parameter("new_plug_orientation").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("new_vehicle_length", descriptor);
    sensor_configuration.new_vehicle_length =
      static_cast<float>(this->get_parameter("new_vehicle_length").as_double());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("new_vehicle_width", descriptor);
    sensor_configuration.new_vehicle_width =
      static_cast<float>(this->get_parameter("new_vehicle_width").as_double());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("new_vehicle_height", descriptor);
    sensor_configuration.new_vehicle_height =
      static_cast<float>(this->get_parameter("new_vehicle_height").as_double());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("new_vehicle_wheelbase", descriptor);
    sensor_configuration.new_vehicle_wheelbase =
      static_cast<float>(this->get_parameter("new_vehicle_wheelbase").as_double());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("new_radar_maximum_distance", descriptor);
    sensor_configuration.new_radar_maximum_distance =
      this->get_parameter("new_radar_maximum_distance").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("new_radar_frequency_slot", descriptor);
    sensor_configuration.new_radar_frequency_slot =
      this->get_parameter("new_radar_frequency_slot").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("new_radar_cycle_time", descriptor);
    sensor_configuration.new_radar_cycle_time =
      this->get_parameter("new_radar_cycle_time").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("new_radar_time_slot", descriptor);
    sensor_configuration.new_radar_time_slot = this->get_parameter("new_radar_time_slot").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("new_radar_country_code", descriptor);
    sensor_configuration.new_radar_country_code =
      this->get_parameter("new_radar_country_code").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("new_radar_powersave_standstill", descriptor);
    sensor_configuration.new_radar_powersave_standstill =
      this->get_parameter("new_radar_powersave_standstill").as_int();
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
    get_param(p, "sensor_model", sensor_model_str) | get_param(p, "return_mode", return_mode_str) |
    get_param(p, "host_ip", new_param.host_ip) | get_param(p, "sensor_ip", new_param.sensor_ip) |
    get_param(p, "new_sensor_ip", new_param.new_sensor_ip) |
    get_param(p, "frame_id", new_param.frame_id) | get_param(p, "data_port", new_param.data_port) |
    get_param(p, "configuration_host_port", new_param.configuration_host_port) |
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

void ContinentalARS548HwInterfaceRosWrapper::OdometryCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  std::scoped_lock lock(mtx_config_);
  // hw_interface_.SetCharacteristicSpeed(0.0);
  // hw_interface_.SetSteeringAngleFrontAxle(0.0);

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
    hw_interface_.SetDrivingDirection(msg->twist.twist.linear.x);
  }

  constexpr float ms_to_kmh = 3.6f;
  hw_interface_.SetVelocityVehicle(ms_to_kmh * msg->twist.twist.linear.x);

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
  hw_interface_.SetSteeringAngleFrontAxle(msg->data);
}

void ContinentalARS548HwInterfaceRosWrapper::SetNewSensorIPRequestCallback(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  std::scoped_lock lock(mtx_config_);
  hw_interface_.SetSensorIPAddress(sensor_configuration_.new_sensor_ip);
}

void ContinentalARS548HwInterfaceRosWrapper::SetNewSensorMountingRequestCallback(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  std::scoped_lock lock(mtx_config_);

  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  auto tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  geometry_msgs::msg::TransformStamped autosar_to_sensor_tf;
  try {
    autosar_to_sensor_tf = tf_buffer->lookupTransform(
      std::string("autosar"), sensor_configuration_.frame_id, rclcpp::Time(0),
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not obtain the transform from the autosar frame to %s (%s)",
      sensor_configuration_.frame_id.c_str(), ex.what());
    return;
  }

  const auto & quat = autosar_to_sensor_tf.transform.rotation;
  geometry_msgs::msg::Vector3 rpy;
  tf2::Matrix3x3(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(rpy.x, rpy.y, rpy.z);

  std::cout << "DEBUG TF" << std::endl;
  std::cout << "translation: x=" << autosar_to_sensor_tf.transform.translation.x
            << " y=" << autosar_to_sensor_tf.transform.translation.y
            << " z=" << autosar_to_sensor_tf.transform.translation.z << std::endl;
  std::cout << "orientation: roll=" << rpy.x << " pitch=" << rpy.y << " yaw=" << rpy.z << std::endl;

  hw_interface_.SetSensorMounting(
    autosar_to_sensor_tf.transform.translation.x, autosar_to_sensor_tf.transform.translation.y,
    autosar_to_sensor_tf.transform.translation.z, rpy.z, rpy.y,
    sensor_configuration_.new_plug_orientation);
}

void ContinentalARS548HwInterfaceRosWrapper::SetNewVehicleParametersRequestCallback(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  std::scoped_lock lock(mtx_config_);
  hw_interface_.SetVehicleParameters(
    sensor_configuration_.new_vehicle_length, sensor_configuration_.new_vehicle_width,
    sensor_configuration_.new_vehicle_height, sensor_configuration_.new_vehicle_wheelbase);
}

void ContinentalARS548HwInterfaceRosWrapper::SetNewRadarParametersRequestCallback(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  std::scoped_lock lock(mtx_config_);
  hw_interface_.SetRadarParameters(
    sensor_configuration_.new_radar_maximum_distance,
    sensor_configuration_.new_radar_frequency_slot, sensor_configuration_.new_radar_cycle_time,
    sensor_configuration_.new_radar_time_slot, sensor_configuration_.new_radar_country_code,
    sensor_configuration_.new_radar_powersave_standstill);
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
     rclcpp::Parameter("new_sensor_ip", sensor_configuration_.new_sensor_ip),
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
  diagnostics.add("longitudinal_velocity_status", sensor_status.longitudinal_velocity_status);
  diagnostics.add(
    "longitudinal_acceleration_status", sensor_status.longitudinal_acceleration_status);
  diagnostics.add("lateral_acceleration_status", sensor_status.lateral_acceleration_status);
  diagnostics.add("yaw_rate_status", sensor_status.yaw_rate_status);
  diagnostics.add("steering_angle_status", sensor_status.steering_angle_status);
  diagnostics.add("driving_direction_status", sensor_status.driving_direction_status);
  diagnostics.add("characteristic_speed_status", sensor_status.characteristic_speed_status);
  diagnostics.add("radar_status", sensor_status.radar_status);
  diagnostics.add("voltage_status", sensor_status.voltage_status);
  diagnostics.add("temperature_status", sensor_status.temperature_status);
  diagnostics.add("blockage_status", sensor_status.blockage_status);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ContinentalARS548HwInterfaceRosWrapper)
}  // namespace ros
}  // namespace nebula
