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

#include "nebula_ros/continental/continental_srr520_hw_interface_wrapper.hpp"

#include "nebula_ros/common/rclcpp_logger.hpp"

#include <nebula_common/util/string_conversions.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>

namespace nebula::ros
{

ContinentalSRR520HwInterfaceWrapper::ContinentalSRR520HwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration> &
    config_ptr)
: parent_node_(parent_node),
  hw_interface_(
    std::make_shared<drivers::continental_srr520::ContinentalSRR520HwInterface>(
      drivers::loggers::RclcppLogger(parent_node->get_logger()).child("HwInterface"))),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(Status::NOT_INITIALIZED),
  config_ptr_(config_ptr)
{
  status_ = hw_interface_->set_sensor_configuration(config_ptr_);

  if (status_ != Status::OK) {
    throw std::runtime_error("Could not initialize HW interface: " + util::to_string(status_));
  }

  status_ = Status::OK;
}

void ContinentalSRR520HwInterfaceWrapper::sensor_interface_start()
{
  using std::chrono_literals::operator""ms;

  if (Status::OK == status_) {
    hw_interface_->sensor_interface_start();
  }

  if (Status::OK == status_) {
    odometry_sub_.subscribe(parent_node_, "odometry_input");
    acceleration_sub_.subscribe(parent_node_, "acceleration_input");

    sync_ptr_ =
      std::make_shared<ExactTimeSync>(ExactTimeSyncPolicy(10), odometry_sub_, acceleration_sub_);
    sync_ptr_->registerCallback(&ContinentalSRR520HwInterfaceWrapper::dynamics_callback, this);

    configure_sensor_service_server_ =
      parent_node_->create_service<continental_srvs::srv::ContinentalSrr520SetRadarParameters>(
        "configure_sensor",
        std::bind(
          &ContinentalSRR520HwInterfaceWrapper::configure_sensor_request_callback, this,
          std::placeholders::_1, std::placeholders::_2));

    sync_timer_ = rclcpp::create_timer(
      parent_node_, parent_node_->get_clock(), 100ms,
      std::bind(&ContinentalSRR520HwInterfaceWrapper::sync_timer_callback, this));
  }
}

void ContinentalSRR520HwInterfaceWrapper::on_config_change(
  const std::shared_ptr<
    const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration> &
    new_config_ptr)
{
  hw_interface_->set_sensor_configuration(new_config_ptr);
  config_ptr_ = new_config_ptr;
}

Status ContinentalSRR520HwInterfaceWrapper::status()
{
  return status_;
}

std::shared_ptr<drivers::continental_srr520::ContinentalSRR520HwInterface>
ContinentalSRR520HwInterfaceWrapper::hw_interface() const
{
  return hw_interface_;
}

void ContinentalSRR520HwInterfaceWrapper::dynamics_callback(
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

  hw_interface_->set_vehicle_dynamics(
    acceleration_msg->accel.accel.linear.x, acceleration_msg->accel.accel.linear.y,
    odometry_msg->twist.twist.angular.z, odometry_msg->twist.twist.linear.x, standstill_);
}

void ContinentalSRR520HwInterfaceWrapper::configure_sensor_request_callback(
  const std::shared_ptr<continental_srvs::srv::ContinentalSrr520SetRadarParameters::Request>
    request,
  const std::shared_ptr<continental_srvs::srv::ContinentalSrr520SetRadarParameters::Response>
    response)
{
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(parent_node_->get_clock());
  auto tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  float longitudinal = request->longitudinal;
  float lateral = request->lateral;
  float vertical = request->vertical;
  float yaw = request->yaw;
  float vehicle_wheelbase = request->vehicle_wheelbase;

  if (vehicle_wheelbase < 0.f) {
    RCLCPP_INFO(
      logger_, "Service vehicle_wheelbase is invalid. Falling back to configuration value (%.2f)",
      config_ptr_->configuration_vehicle_wheelbase);
    vehicle_wheelbase = config_ptr_->configuration_vehicle_wheelbase;
  }

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

    longitudinal = base_to_sensor_tf.transform.translation.x - vehicle_wheelbase;
    lateral = base_to_sensor_tf.transform.translation.y;
    vertical = base_to_sensor_tf.transform.translation.z;
    yaw = rpy.z;
  }

  yaw = std::min<float>(std::max<float>(yaw, -3.14159f), 3.14159f);

  auto result = hw_interface_->configure_sensor(
    request->sensor_id, longitudinal, lateral, vertical, yaw,
    longitudinal + 0.5 * vehicle_wheelbase, vehicle_wheelbase, request->cover_damping,
    request->plug_bottom, request->reset_sensor_configuration);

  response->success = result == Status::OK;
  response->message = util::to_string(result);
}

void ContinentalSRR520HwInterfaceWrapper::sync_timer_callback()
{
  hw_interface_->sensor_sync();
}

}  // namespace nebula::ros
