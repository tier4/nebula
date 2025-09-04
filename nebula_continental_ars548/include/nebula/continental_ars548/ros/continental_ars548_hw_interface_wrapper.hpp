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

#include <nebula_common/continental/continental_ars548.hpp>
#include <nebula_common/util/rate_checker.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_ars548_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <continental_srvs/srv/continental_ars548_set_network_configuration.hpp>
#include <continental_srvs/srv/continental_ars548_set_radar_parameters.hpp>
#include <continental_srvs/srv/continental_ars548_set_sensor_mounting.hpp>
#include <continental_srvs/srv/continental_ars548_set_vehicle_parameters.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <memory>

namespace nebula::ros
{
class ContinentalARS548HwInterfaceWrapper
{
public:
  ContinentalARS548HwInterfaceWrapper(
    rclcpp::Node * const parent_node,
    std::shared_ptr<const drivers::continental_ars548::ContinentalARS548SensorConfiguration> &
      config);

  /// @brief Starts the hw interface and subscribes to the input topics
  Status sensor_interface_start();

  void on_config_change(
    const std::shared_ptr<const drivers::continental_ars548::ContinentalARS548SensorConfiguration> &
      new_config_ptr);

  /// @brief Get current status of the hw interface
  /// @return Current status
  nebula::Status status();

  std::shared_ptr<drivers::continental_ars548::ContinentalARS548HwInterface> hw_interface() const;

private:
  /// @brief Callback to send the odometry information to the radar device
  /// @param msg The odometry message
  void odometry_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  /// @brief Callback to send the acceleration information to the radar device
  /// @param msg The acceleration message
  void acceleration_callback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);

  /// @brief Callback to send the steering angle information to the radar device
  /// @param msg The steering angle message
  void steering_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);

  /// @brief Service callback to set the new sensor ip
  /// @param request service request
  /// @param response service response
  void set_network_configuration_request_callback(
    const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetNetworkConfiguration::Request>
      request,
    const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetNetworkConfiguration::Response>
      response);

  /// @brief Service callback to set the new sensor mounting position
  /// @param request service request
  /// @param response service response
  void set_sensor_mounting_request_callback(
    const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetSensorMounting::Request>
      request,
    const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetSensorMounting::Response>
      response);

  /// @brief Service callback to set the new vehicle parameters
  /// @param request service request
  /// @param response service response
  void set_vehicle_parameters_request_callback(
    const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetVehicleParameters::Request>
      request,
    const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetVehicleParameters::Response>
      response);

  /// @brief Service callback to set the new radar parameters
  /// @param request service request
  /// @param response service response
  void set_radar_parameters_request_callback(
    const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetRadarParameters::Request>
      request,
    const std::shared_ptr<continental_srvs::srv::ContinentalArs548SetRadarParameters::Response>
      response);

  rclcpp::Node * const parent_node_;
  std::shared_ptr<drivers::continental_ars548::ContinentalARS548HwInterface> hw_interface_{};
  rclcpp::Logger logger_;
  nebula::Status status_{};
  std::shared_ptr<const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration>
    config_ptr_{};

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr odometry_sub_{};
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
    acceleration_sub_{};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_angle_sub_{};

  nebula::util::RateChecker odometry_rate_checker_;
  nebula::util::RateChecker acceleration_rate_checker_;
  nebula::util::RateChecker steering_angle_rate_checker_;

  bool standstill_{true};

  rclcpp::Service<continental_srvs::srv::ContinentalArs548SetNetworkConfiguration>::SharedPtr
    set_network_configuration_service_server_{};
  rclcpp::Service<continental_srvs::srv::ContinentalArs548SetSensorMounting>::SharedPtr
    set_sensor_mounting_service_server_{};
  rclcpp::Service<continental_srvs::srv::ContinentalArs548SetVehicleParameters>::SharedPtr
    set_vehicle_parameters_service_server_{};
  rclcpp::Service<continental_srvs::srv::ContinentalArs548SetRadarParameters>::SharedPtr
    set_radar_parameters_service_server_{};
};
}  // namespace nebula::ros
