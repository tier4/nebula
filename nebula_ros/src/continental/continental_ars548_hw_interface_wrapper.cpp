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

#include "nebula_ros/continental/continental_ars548_hw_interface_wrapper.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace nebula
{
namespace ros
{

ContinentalARS548HwInterfaceWrapper::ContinentalARS548HwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration> &
    config_ptr)
: parent_node_(parent_node),
  hw_interface_(
    std::make_shared<nebula::drivers::continental_ars548::ContinentalARS548HwInterface>()),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(Status::NOT_INITIALIZED),
  config_ptr_(config_ptr)
{
  hw_interface_->set_logger(
    std::make_shared<rclcpp::Logger>(parent_node->get_logger().get_child("HwInterface")));
  status_ = hw_interface_->set_sensor_configuration(config_ptr);

  if (status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Could not initialize HW interface: " << status_).str());
  }

  status_ = Status::OK;
}

void ContinentalARS548HwInterfaceWrapper::sensor_interface_start()
{
  if (Status::OK == status_) {
    hw_interface_->sensor_interface_start();
  }

  if (Status::OK == status_) {
    odometry_sub_ =
      parent_node_->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "odometry_input", rclcpp::QoS{1},
        std::bind(
          &ContinentalARS548HwInterfaceWrapper::odometry_callback, this, std::placeholders::_1));

    acceleration_sub_ =
      parent_node_->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
        "acceleration_input", rclcpp::QoS{1},
        std::bind(
          &ContinentalARS548HwInterfaceWrapper::acceleration_callback, this,
          std::placeholders::_1));

    steering_angle_sub_ = parent_node_->create_subscription<std_msgs::msg::Float32>(
      "steering_angle_input", rclcpp::SensorDataQoS(),
      std::bind(
        &ContinentalARS548HwInterfaceWrapper::steering_angle_callback, this,
        std::placeholders::_1));

    set_network_configuration_service_server_ =
      parent_node_->create_service<continental_srvs::srv::ContinentalArs548SetNetworkConfiguration>(
        "set_network_configuration",
        std::bind(
          &ContinentalARS548HwInterfaceWrapper::set_network_configuration_request_callback, this,
          std::placeholders::_1, std::placeholders::_2));

    set_sensor_mounting_service_server_ =
      parent_node_->create_service<continental_srvs::srv::ContinentalArs548SetSensorMounting>(
        "set_sensor_mounting",
        std::bind(
          &ContinentalARS548HwInterfaceWrapper::set_sensor_mounting_request_callback, this,
          std::placeholders::_1, std::placeholders::_2));

    set_vehicle_parameters_service_server_ =
      parent_node_->create_service<continental_srvs::srv::ContinentalArs548SetVehicleParameters>(
        "set_vehicle_parameters",
        std::bind(
          &ContinentalARS548HwInterfaceWrapper::set_vehicle_parameters_request_callback, this,
          std::placeholders::_1, std::placeholders::_2));

    set_radar_parameters_service_server_ =
      parent_node_->create_service<continental_srvs::srv::ContinentalArs548SetRadarParameters>(
        "set_radar_parameters",
        std::bind(
          &ContinentalARS548HwInterfaceWrapper::set_radar_parameters_request_callback, this,
          std::placeholders::_1, std::placeholders::_2));
  }
}

void ContinentalARS548HwInterfaceWrapper::on_config_change(
  const std::shared_ptr<
    const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration> &
    new_config_ptr_ptr)
{
  hw_interface_->set_sensor_configuration(new_config_ptr_ptr);
  config_ptr_ = new_config_ptr_ptr;
}

Status ContinentalARS548HwInterfaceWrapper::status()
{
  return status_;
}

std::shared_ptr<drivers::continental_ars548::ContinentalARS548HwInterface>
ContinentalARS548HwInterfaceWrapper::hw_interface() const
{
  return hw_interface_;
}

void ContinentalARS548HwInterfaceWrapper::odometry_callback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  constexpr float speed_to_standstill = 0.5f;
  constexpr float speed_to_moving = 2.f;

  if (standstill_ && std::abs(msg->twist.twist.linear.x) > speed_to_moving) {
    standstill_ = false;
  } else if (!standstill_ && std::abs(msg->twist.twist.linear.x) < speed_to_standstill) {
    standstill_ = true;
  }

  if (standstill_) {
    hw_interface_->set_driving_direction(0);
  } else {
    hw_interface_->set_driving_direction(msg->twist.twist.linear.x > 0.f ? 1 : -1);
  }

  constexpr float ms_to_kmh = 3.6f;
  hw_interface_->set_velocity_vehicle(ms_to_kmh * std::abs(msg->twist.twist.linear.x));

  constexpr float rad_to_deg = 180.f / M_PI;
  hw_interface_->set_yaw_rate(rad_to_deg * msg->twist.twist.angular.z);
}

void ContinentalARS548HwInterfaceWrapper::acceleration_callback(
  const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  hw_interface_->set_acceleration_lateral_cog(msg->accel.accel.linear.y);
  hw_interface_->set_acceleration_longitudinal_cog(msg->accel.accel.linear.x);
}

void ContinentalARS548HwInterfaceWrapper::steering_angle_callback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  constexpr float rad_to_deg = 180.f / M_PI;
  hw_interface_->set_steering_angle_front_axle(rad_to_deg * msg->data);
}

void ContinentalARS548HwInterfaceWrapper::set_network_configuration_request_callback(
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetNetworkConfiguration::Request>
    request,
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetNetworkConfiguration::Response>
    response)
{
  auto result = hw_interface_->set_sensor_ip_address(request->sensor_ip.data);
  response->success = result == Status::OK;
  response->message = (std::stringstream() << result).str();
}

void ContinentalARS548HwInterfaceWrapper::set_sensor_mounting_request_callback(
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetSensorMounting::Request> request,
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetSensorMounting::Response>
    response)
{
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(parent_node_->get_clock());
  auto tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  float longitudinal = request->longitudinal;
  float lateral = request->lateral;
  float vertical = request->vertical;
  float yaw = request->yaw;
  float pitch = request->pitch;

  if (request->autoconfigure_extrinsics) {
    RCLCPP_INFO(
      logger_,
      "autoconfigure_extrinsics was set to true, so the mounting position will be derived from the "
      "tfs");

    geometry_msgs::msg::TransformStamped base_to_sensor_tf;
    try {
      base_to_sensor_tf = tf_buffer->lookupTransform(
        config_ptr_->base_frame, config_ptr_->frame_id, rclcpp::Time(0),
        rclcpp::Duration::from_seconds(0.5));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_, "Could not obtain the transform from the base frame to %s (%s)",
        config_ptr_->frame_id.c_str(), ex.what());
      response->success = false;
      response->message = ex.what();
      return;
    }

    const auto & quat = base_to_sensor_tf.transform.rotation;
    geometry_msgs::msg::Vector3 rpy;
    tf2::Matrix3x3(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(rpy.x, rpy.y, rpy.z);

    longitudinal =
      base_to_sensor_tf.transform.translation.x - config_ptr_->configuration_vehicle_wheelbase;
    lateral = base_to_sensor_tf.transform.translation.y;
    vertical = base_to_sensor_tf.transform.translation.z;
    yaw = rpy.z;
    pitch = rpy.y;
  }

  auto result = hw_interface_->set_sensor_mounting(
    longitudinal, lateral, vertical, yaw, pitch, request->plug_orientation);

  response->success = result == Status::OK;
  response->message = (std::stringstream() << result).str();
}

void ContinentalARS548HwInterfaceWrapper::set_vehicle_parameters_request_callback(
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
      logger_, "Service vehicle_length is invalid. Falling back to configuration value (%.2f)",
      config_ptr_->configuration_vehicle_length);
    vehicle_length = config_ptr_->configuration_vehicle_length;
  }

  if (vehicle_width < 0.f) {
    RCLCPP_INFO(
      logger_, "Service vehicle_width is invalid. Falling back to configuration value (%.2f)",
      config_ptr_->configuration_vehicle_width);
    vehicle_width = config_ptr_->configuration_vehicle_width;
  }

  if (vehicle_height < 0.f) {
    RCLCPP_INFO(
      logger_, "Service vehicle_height is invalid. Falling back to configuration value (%.2f)",
      config_ptr_->configuration_vehicle_height);
    vehicle_height = config_ptr_->configuration_vehicle_height;
  }

  if (vehicle_wheelbase < 0.f) {
    RCLCPP_INFO(
      logger_, "Service vehicle_wheelbase is invalid. Falling back to configuration value (%.2f)",
      config_ptr_->configuration_vehicle_wheelbase);
    vehicle_wheelbase = config_ptr_->configuration_vehicle_wheelbase;
  }

  auto result = hw_interface_->set_vehicle_parameters(
    vehicle_length, vehicle_width, vehicle_height, vehicle_wheelbase);

  response->success = result == Status::OK;
  response->message = (std::stringstream() << result).str();
}

void ContinentalARS548HwInterfaceWrapper::set_radar_parameters_request_callback(
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetRadarParameters::Request>
    request,
  const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetRadarParameters::Response>
    response)
{
  auto result = hw_interface_->set_radar_parameters(
    request->maximum_distance, request->frequency_slot, request->cycle_time, request->time_slot,
    request->country_code, request->powersave_standstill);

  response->success = result == Status::OK;
  response->message = (std::stringstream() << result).str();
}

}  // namespace ros
}  // namespace nebula
