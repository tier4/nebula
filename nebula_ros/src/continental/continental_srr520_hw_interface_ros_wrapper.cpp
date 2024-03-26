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

#include <nebula_ros/continental/continental_srr520_hw_interface_ros_wrapper.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <thread>

namespace nebula
{
namespace ros
{
ContinentalSRR520HwInterfaceRosWrapper::ContinentalSRR520HwInterfaceRosWrapper(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("continental_srr520_hw_interface_ros_wrapper", options), hw_interface_()
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
  std::shared_ptr<drivers::continental_srr520::ContinentalSRR520SensorConfiguration>
    sensor_cfg_ptr =
      std::make_shared<drivers::continental_srr520::ContinentalSRR520SensorConfiguration>(
        sensor_configuration_);
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  hw_interface_.RegisterScanCallback(std::bind(
    &ContinentalSRR520HwInterfaceRosWrapper::ReceivePacketsDataCallback, this,
    std::placeholders::_1));
  packets_pub_ = this->create_publisher<nebula_msgs::msg::NebulaPackets>(
    "nebula_packets", rclcpp::SensorDataQoS());

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ContinentalSRR520HwInterfaceRosWrapper::paramCallback, this, std::placeholders::_1));

  StreamStart();
}

ContinentalSRR520HwInterfaceRosWrapper::~ContinentalSRR520HwInterfaceRosWrapper()
{
}

Status ContinentalSRR520HwInterfaceRosWrapper::StreamStart()
{
  using std::chrono_literals::operator""ms;

  if (Status::OK == interface_status_) {
    interface_status_ = hw_interface_.SensorInterfaceStart();
  }

  if (Status::OK == interface_status_) {
    odometry_sub_.subscribe(this, "odometry_input");
    acceleration_sub_.subscribe(this, "acceleration_input");

    sync_ptr_ =
      std::make_shared<ExactTimeSync>(ExactTimeSyncPolicy(10), odometry_sub_, acceleration_sub_);
    sync_ptr_->registerCallback(&ContinentalSRR520HwInterfaceRosWrapper::dynamicsCallback, this);

    configure_sensor_service_server_ =
      this->create_service<continental_srvs::srv::ContinentalSrr520SetRadarParameters>(
        "configure_sensor",
        std::bind(
          &ContinentalSRR520HwInterfaceRosWrapper::ConfigureSensorRequestCallback, this,
          std::placeholders::_1, std::placeholders::_2));

    sync_timer_ = rclcpp::create_timer(
      this, get_clock(), 100ms,
      std::bind(&ContinentalSRR520HwInterfaceRosWrapper::syncTimerCallback, this));
  }

  return interface_status_;
}

Status ContinentalSRR520HwInterfaceRosWrapper::StreamStop()
{
  hw_interface_.SensorInterfaceStop();
  return Status::OK;
}
Status ContinentalSRR520HwInterfaceRosWrapper::Shutdown()
{
  hw_interface_.SensorInterfaceStop();
  return Status::OK;
}

Status ContinentalSRR520HwInterfaceRosWrapper::InitializeHwInterface(  // todo: don't think
                                                                       // this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

Status ContinentalSRR520HwInterfaceRosWrapper::GetParameters(
  drivers::continental_srr520::ContinentalSRR520SensorConfiguration & sensor_configuration)
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
    this->declare_parameter<std::string>("interface", descriptor);
    sensor_configuration.interface = this->get_parameter("interface").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("receiver_timeout_sec", descriptor);
    sensor_configuration.receiver_timeout_sec =
      this->get_parameter("receiver_timeout_sec").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("sender_timeout_sec", descriptor);
    sensor_configuration.sender_timeout_sec = this->get_parameter("sender_timeout_sec").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("filters", descriptor);
    sensor_configuration.filters = this->get_parameter("filters").as_string();
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
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<bool>("use_bus_time", descriptor);
    sensor_configuration.use_bus_time = this->get_parameter("use_bus_time").as_bool();
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

  if (sensor_configuration.frame_id.empty()) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void ContinentalSRR520HwInterfaceRosWrapper::ReceivePacketsDataCallback(
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> scan_buffer)
{
  packets_pub_->publish(std::move(scan_buffer));
}

rcl_interfaces::msg::SetParametersResult ContinentalSRR520HwInterfaceRosWrapper::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "add_on_set_parameters_callback");
  RCLCPP_DEBUG_STREAM(this->get_logger(), p);
  RCLCPP_DEBUG_STREAM(this->get_logger(), sensor_configuration_);
  RCLCPP_INFO_STREAM(this->get_logger(), p);

  drivers::continental_srr520::ContinentalSRR520SensorConfiguration new_param{
    sensor_configuration_};
  RCLCPP_INFO_STREAM(this->get_logger(), new_param);
  std::string sensor_model_str;
  std::string return_mode_str;

  if (
    get_param(p, "sensor_model", sensor_model_str) |
    get_param(p, "interface", new_param.interface) |
    get_param(p, "receiver_timeout_sec", new_param.receiver_timeout_sec) |
    get_param(p, "sender_timeout_sec", new_param.sender_timeout_sec) |
    get_param(p, "frame_id", new_param.frame_id) |
    get_param(p, "base_frame", new_param.base_frame) | get_param(p, "filters", new_param.filters) |
    get_param(p, "use_bus_time", new_param.use_bus_time) |
    get_param(p, "configuration_vehicle_wheelbase", new_param.configuration_vehicle_wheelbase)) {
    if (0 < sensor_model_str.length())
      new_param.sensor_model = nebula::drivers::SensorModelFromString(sensor_model_str);

    sensor_configuration_ = new_param;
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
      std::make_shared<drivers::continental_srr520::ContinentalSRR520SensorConfiguration>(
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

void ContinentalSRR520HwInterfaceRosWrapper::dynamicsCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr & odometry_msg,
  const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr & acceleration_msg)
{
  constexpr float speed_to_standstill = 0.5f;
  constexpr float speed_to_moving = 2.f;

  if (standstill_ && std::abs(odometry_msg->twist.twist.linear.x) > speed_to_moving) {
    standstill_ = false;
  } else if (!standstill_ && std::abs(odometry_msg->twist.twist.linear.x) < speed_to_standstill) {
    standstill_ = true;
  }

  hw_interface_.SetVehicleDynamics(
    acceleration_msg->accel.accel.linear.x, acceleration_msg->accel.accel.linear.y,
    odometry_msg->twist.twist.angular.z, odometry_msg->twist.twist.linear.x, standstill_);
}

void ContinentalSRR520HwInterfaceRosWrapper::ConfigureSensorRequestCallback(
  const std::shared_ptr<continental_srvs::srv::ContinentalSrr520SetRadarParameters::Request>
    request,
  const std::shared_ptr<continental_srvs::srv::ContinentalSrr520SetRadarParameters::Response>
    response)
{
  std::scoped_lock lock(mtx_config_);

  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  auto tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  float longitudinal = request->longitudinal;
  float lateral = request->lateral;
  float vertical = request->vertical;
  float yaw = request->yaw;
  float vehicle_wheelbase = request->vehicle_wheelbase;

  if (vehicle_wheelbase < 0.f) {
    RCLCPP_INFO(
      this->get_logger(),
      "Service vehicle_wheelbase is invalid. Falling back to configuration value (%.2f)",
      sensor_configuration_.configuration_vehicle_wheelbase);
    vehicle_wheelbase = sensor_configuration_.configuration_vehicle_wheelbase;
  }

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

    longitudinal = base_to_sensor_tf.transform.translation.x - vehicle_wheelbase;
    lateral = base_to_sensor_tf.transform.translation.y;
    vertical = base_to_sensor_tf.transform.translation.z;
    yaw = rpy.z;
  }

  yaw = std::min<float>(std::max<float>(yaw, -3.14159f), 3.14159f);

  auto result = hw_interface_.ConfigureSensor(
    request->sensor_id, longitudinal, lateral, vertical, yaw,
    longitudinal + 0.5 * vehicle_wheelbase, vehicle_wheelbase, request->cover_damping,
    request->plug_bottom, request->reset_sensor_configuration);

  response->success = result == Status::OK;
  response->message = (std::stringstream() << result).str();
}

void ContinentalSRR520HwInterfaceRosWrapper::syncTimerCallback()
{
  hw_interface_.SensorSync();
}

std::vector<rcl_interfaces::msg::SetParametersResult>
ContinentalSRR520HwInterfaceRosWrapper::updateParameters()
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "updateParameters start");
  std::ostringstream os_sensor_model;
  os_sensor_model << sensor_configuration_.sensor_model;
  RCLCPP_INFO_STREAM(this->get_logger(), "set_parameters");

  auto results = set_parameters(
    {rclcpp::Parameter("sensor_model", os_sensor_model.str()),
     rclcpp::Parameter("interface", sensor_configuration_.interface),
     rclcpp::Parameter("receiver_timeout_sec", sensor_configuration_.receiver_timeout_sec),
     rclcpp::Parameter("sender_timeout_sec", sensor_configuration_.sender_timeout_sec),
     rclcpp::Parameter("frame_id", sensor_configuration_.frame_id),
     rclcpp::Parameter("base_frame", sensor_configuration_.base_frame),
     rclcpp::Parameter("filters", sensor_configuration_.filters),
     rclcpp::Parameter("use_bus_time", sensor_configuration_.use_bus_time),
     rclcpp::Parameter(
       "configuration_vehicle_wheelbase", sensor_configuration_.configuration_vehicle_wheelbase)});

  RCLCPP_DEBUG_STREAM(this->get_logger(), "updateParameters end");
  return results;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ContinentalSRR520HwInterfaceRosWrapper)
}  // namespace ros
}  // namespace nebula
