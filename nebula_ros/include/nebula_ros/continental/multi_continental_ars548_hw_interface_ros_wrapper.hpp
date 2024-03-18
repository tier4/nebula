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
#include <nebula_common/nebula_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_continental/multi_continental_ars548_hw_interface.hpp>
#include <nebula_ros/common/nebula_hw_interface_ros_wrapper_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <boost/asio.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
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
/// NOTE: this is a temporary class that acts as a single hw interface for multiple devices
/// The reason behind this is a not-so-efficient multicasting and package processing when having N
/// interfaces for N devices If we end up having problems because of that we may switch to this
/// implementation. Otherwise this implementation will be removed at a later date
class MultiContinentalARS548HwInterfaceRosWrapper final : public rclcpp::Node,
                                                          NebulaHwInterfaceWrapperBase
{
  drivers::continental_ars548::MultiContinentalARS548HwInterface hw_interface_;
  Status interface_status_;

  drivers::continental_ars548::MultiContinentalARS548SensorConfiguration sensor_configuration_;

  /// @brief Received Continental Radar message publisher
  std::unordered_map<std::string, rclcpp::Publisher<nebula_msgs::msg::NebulaPackets>::SharedPtr>
    packets_pub_map_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_angle_sub_;

  bool standstill_{true};

  /// @brief Initializing hardware interface ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) override;
  /// @brief Callback for receiving NebulaPackets
  /// @param packets_buffer Received NebulaPackets
  void ReceivePacketsDataCallback(
    std::unique_ptr<nebula_msgs::msg::NebulaPackets> packets_buffer, const std::string & sensor_ip);

  /// @brief Callback to send the odometry information to the radar device
  /// @param msg The odometry message
  void OdometryCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  /// @brief Callback to send the acceleration information to the radar device
  /// @param msg The acceleration message
  void AccelerationCallback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);

  /// @brief Callback to send the steering angle information to the radar device
  /// @param msg The steering angle message
  void SteeringAngleCallback(const std_msgs::msg::Float32::SharedPtr msg);

public:
  explicit MultiContinentalARS548HwInterfaceRosWrapper(const rclcpp::NodeOptions & options);
  ~MultiContinentalARS548HwInterfaceRosWrapper() noexcept override;

  /// @brief Start point cloud streaming (Call SensorInterfaceStart of HwInterface)
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
  Status GetParameters(
    drivers::continental_ars548::MultiContinentalARS548SensorConfiguration & sensor_configuration);

private:
  std::mutex mtx_config_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

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
