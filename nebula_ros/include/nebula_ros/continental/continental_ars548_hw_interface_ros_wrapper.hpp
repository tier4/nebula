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

#ifndef NEBULA_ContinentalARS548HwInterfaceRosWrapper_H
#define NEBULA_ContinentalARS548HwInterfaceRosWrapper_H

#include <ament_index_cpp/get_package_prefix.hpp>
#include <boost_tcp_driver/tcp_driver.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/continental/continental_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_ars548_hw_interface.hpp>
#include <nebula_ros/common/nebula_hw_interface_ros_wrapper_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>

#include <boost/asio.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace nebula
{
namespace ros
{
/// @brief Get parameter from rclcpp::Parameter
/// @tparam T
/// @param p Parameter from rclcpp parameter callback
/// @param name Target parameter name
/// @param value Corresponding value
/// @return Whether the target name existed
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

/// @brief Hardware interface ros wrapper of continental radar ethernet driver
class ContinentalARS548HwInterfaceRosWrapper final : public rclcpp::Node,
                                                     NebulaHwInterfaceWrapperBase
{
  drivers::continental_ars548::ContinentalARS548HwInterface hw_interface_;
  Status interface_status_;

  drivers::ContinentalARS548SensorConfiguration sensor_configuration_;

  /// @brief Received Continental Radar message publisher
  rclcpp::Publisher<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_angle_sub_;

  bool standstill_{true};

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_new_sensor_ip_service_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_new_sensor_mounting_service_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_new_vehicle_parameters_service_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_new_radar_parameters_service_server_;

  /// @brief Initializing hardware interface ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) override;
  /// @brief Callback for receiving NebulaPackets
  /// @param packets_buffer Received NebulaPackets
  void ReceivePacketsDataCallback(std::unique_ptr<nebula_msgs::msg::NebulaPackets> packets_buffer);

  /// @brief Callback to send the odometry information to the radar device
  /// @param msg The odometry message
  void OdometryCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  /// @brief Callback to send the acceleration information to the radar device
  /// @param msg The acceleration message
  void AccelerationCallback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);

  /// @brief Callback to send the steering angle information to the radar device
  /// @param msg The steering angle message
  void SteeringAngleCallback(const std_msgs::msg::Float32::SharedPtr msg);

  /// @brief Service callback to set the new sensor ip
  /// @param request Empty service request
  /// @param response Empty service response
  void SetNewSensorIPRequestCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /// @brief Service callback to set the new sensor mounting position
  /// @param request Empty service request
  /// @param response Empty service response
  void SetNewSensorMountingRequestCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /// @brief Service callback to set the new vehicle parameters
  /// @param request Empty service request
  /// @param response Empty service response
  void SetNewVehicleParametersRequestCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /// @brief Service callback to set the new radar parameters
  /// @param request Empty service request
  /// @param response Empty service response
  void SetNewRadarParametersRequestCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

public:
  explicit ContinentalARS548HwInterfaceRosWrapper(const rclcpp::NodeOptions & options);
  ~ContinentalARS548HwInterfaceRosWrapper() noexcept override;

  /// @brief Start point cloud streaming (Call CloudInterfaceStart of HwInterface)
  /// @return Resulting status
  Status StreamStart() override;
  /// @brief Stop point cloud streaming (not used)
  /// @return Resulting status
  Status StreamStop() override;
  /// @brief Shutdown (not used)
  /// @return Resulting status
  Status Shutdown() override;
  /// @brief Get configurations from ros parameters
  /// @param sensor_configuration Output of SensorConfiguration
  /// @return Resulting status
  Status GetParameters(drivers::ContinentalARS548SensorConfiguration & sensor_configuration);

private:
  std::mutex mtx_config_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  diagnostic_updater::Updater diagnostics_updater_;

  /// @brief Callback to populate diagnostic messages
  void ContinentalMonitorStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  /// @brief rclcpp parameter callback
  /// @param parameters Received parameters
  /// @return SetParametersResult
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /// @brief Updating rclcpp parameter
  /// @return SetParametersResult
  std::vector<rcl_interfaces::msg::SetParametersResult> updateParameters();
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_ContinentalARS548HwInterfaceRosWrapper_H
