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

#pragma once

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <nebula_common/continental/continental_srr520.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_srr520_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <continental_srvs/srv/continental_srr520_set_radar_parameters.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <memory>

namespace nebula::ros
{
class ContinentalSRR520HwInterfaceWrapper
{
public:
  ContinentalSRR520HwInterfaceWrapper(
    rclcpp::Node * const parent_node,
    std::shared_ptr<const drivers::continental_srr520::ContinentalSRR520SensorConfiguration> &
      config);

  /// @brief Starts the hw interface and subscribes to the input topics
  void sensor_interface_start();

  void on_config_change(
    const std::shared_ptr<const drivers::continental_srr520::ContinentalSRR520SensorConfiguration> &
      new_config);

  /// @brief Get current status of the hw interface
  /// @return Current status
  nebula::Status status();

  std::shared_ptr<drivers::continental_srr520::ContinentalSRR520HwInterface> hw_interface() const;

private:
  /// @brief Callback to send the odometry information to the radar device
  /// @param odometry_msg The odometry message
  /// @param acceleration_msg The acceleration message
  void dynamics_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr & odometry_msg,
    const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr & acceleration_msg);

  /// @brief Service callback to set the new sensor mounting position
  /// @param request Empty service request
  /// @param response Empty service response
  void configure_sensor_request_callback(
    const std::shared_ptr<continental_srvs::srv::ContinentalSrr520SetRadarParameters::Request>
      request,
    const std::shared_ptr<continental_srvs::srv::ContinentalSrr520SetRadarParameters::Response>
      response);

  /// @brief Method periodically called to initiate the sensor synchronization mechanism
  void sync_timer_callback();

  rclcpp::Node * const parent_node_;
  std::shared_ptr<drivers::continental_srr520::ContinentalSRR520HwInterface> hw_interface_{};
  rclcpp::Logger logger_;
  nebula::Status status_{};
  std::shared_ptr<const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration>
    config_ptr_{};

  message_filters::Subscriber<geometry_msgs::msg::TwistWithCovarianceStamped> odometry_sub_;
  message_filters::Subscriber<geometry_msgs::msg::AccelWithCovarianceStamped> acceleration_sub_;

  using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<
    geometry_msgs::msg::TwistWithCovarianceStamped, geometry_msgs::msg::AccelWithCovarianceStamped>;
  using ExactTimeSync = message_filters::Synchronizer<ExactTimeSyncPolicy>;
  std::shared_ptr<ExactTimeSync> sync_ptr_{};
  rclcpp::TimerBase::SharedPtr sync_timer_;

  bool standstill_{true};

  rclcpp::Service<continental_srvs::srv::ContinentalSrr520SetRadarParameters>::SharedPtr
    configure_sensor_service_server_{};
};
}  // namespace nebula::ros
