// Copyright 2026 TIER IV, Inc.
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

#ifndef NEBULA_ROS2_CONTROL__NEBULA_SENSOR_INTERFACE_HPP_
#define NEBULA_ROS2_CONTROL__NEBULA_SENSOR_INTERFACE_HPP_

#include "nebula_ros2_control/component_loader.hpp"
#include "nebula_ros2_control/driver_config.hpp"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_components/node_instance_wrapper.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace nebula::ros2_control
{

/// ros2_control `SensorInterface` that runs a Nebula sensor driver inside the controller manager.
///
/// The driver is any rclcpp component (e.g. `HesaiRosWrapper`, `VelodyneRosWrapper`, ...). It is
/// loaded by class name, configured from the `<param>` tags of the `<ros2_control>` URDF block,
/// and spun by a dedicated executor, so it publishes its point clouds / packets / diagnostics
/// exactly as when launched as a standalone node.
///
/// Lifecycle mapping:
/// - on_init:      parse and validate the URDF configuration
/// - on_configure: locate and load the driver library
/// - on_activate:  instantiate the driver node (connects to the sensor) and start spinning it
/// - on_deactivate: stop spinning and destroy the driver node (releases the sensor)
/// - on_cleanup:   unload the driver library
///
/// The `<sensor>` tag can export the state interfaces listed in StateInterfaceKind, which are
/// computed from the driver's lifecycle and from an optional monitored output topic.
class NebulaSensorInterface : public hardware_interface::SensorInterface
{
public:
  NebulaSensorInterface() = default;
  ~NebulaSensorInterface() override;

  // Newer ros2_control releases add an on_init() overload taking a params struct, which forwards
  // to the HardwareInfo one overridden here. Keep it visible instead of hiding it.
  using hardware_interface::SensorInterface::on_init;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// The parsed driver configuration (valid after on_init).
  [[nodiscard]] const DriverConfig & driver_config() const { return config_; }

private:
  /// Written by the monitor subscription (executor thread), read in read() (control thread).
  struct MonitorState
  {
    std::atomic<std::uint64_t> message_count{0};
    std::atomic<std::int64_t> last_receive_ns{0};
    std::atomic<double> header_stamp{std::numeric_limits<double>::quiet_NaN()};
  };

  struct ExportedState
  {
    StateInterfaceKind kind;
    std::string name;
    double value{0.0};
  };

  bool start_driver();
  void stop_driver();

  rclcpp::Logger logger_{rclcpp::get_logger("nebula_ros2_control")};

  DriverConfig config_;
  std::string sensor_name_;
  // Sized once in on_init and never resized afterwards: the exported interfaces point into it.
  std::vector<ExportedState> states_;

  std::unique_ptr<ComponentLoader> loader_;
  std::optional<rclcpp_components::NodeInstanceWrapper> driver_node_;
  rclcpp::Node::SharedPtr monitor_node_;
  rclcpp::GenericSubscription::SharedPtr monitor_subscription_;
  // Created once and never reassigned, so read() can access it without synchronization.
  const std::shared_ptr<MonitorState> monitor_state_{std::make_shared<MonitorState>()};

  std::shared_ptr<rclcpp::Executor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> spin_finished_{true};
  std::atomic<bool> active_{false};

  // Rate estimation window, only touched in read().
  std::int64_t rate_window_start_ns_{0};
  std::uint64_t rate_window_start_count_{0};
  double message_rate_{0.0};
};

}  // namespace nebula::ros2_control

#endif  // NEBULA_ROS2_CONTROL__NEBULA_SENSOR_INTERFACE_HPP_
