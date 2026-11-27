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

#ifndef NEBULA_ROS2_CONTROL__DRIVER_CONFIG_HPP_
#define NEBULA_ROS2_CONTROL__DRIVER_CONFIG_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

/// @file
/// ros2_control-independent helpers of the Nebula hardware interface: turning the `<param>` tags
/// of a `<ros2_control>` URDF block into the configuration of the driver node, and reading the
/// header stamp out of a serialized message.

namespace nebula::ros2_control
{

/// Thrown for invalid or inconsistent URDF parameters.
class ConfigError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// The state interfaces the hardware interface can export for its `<sensor>`.
enum class StateInterfaceKind {
  /// 1.0 while the driver node is running (hardware active), 0.0 otherwise.
  driver_active,
  /// Number of messages received on the monitored topic since activation.
  message_count,
  /// Rate of the monitored topic in Hz, averaged over roughly one second.
  message_rate,
  /// Seconds since the last message was received on the monitored topic (NaN before the first).
  message_age,
  /// `header.stamp` of the last monitored message, in seconds (NaN before the first).
  header_stamp,
};

/// Parse the name of a `<state_interface>` tag. Throws ConfigError for unknown names.
StateInterfaceKind parse_state_interface_kind(const std::string & name);

/// Whether the state interface needs a monitored topic (`monitor_topic`) to be meaningful.
bool requires_monitor_topic(StateInterfaceKind kind);

/// Everything needed to instantiate and run one Nebula driver node.
struct DriverConfig
{
  /// Component class name as registered with rclcpp_components, e.g. `HesaiRosWrapper`.
  std::string plugin;
  /// Package registering `plugin`. If empty, all packages in the ament index are searched.
  std::string package;
  /// Explicit library (absolute path or file name) providing `plugin`. Bypasses the ament index.
  std::string library;

  std::string node_name;
  std::string node_namespace;

  /// ROS parameter files, applied in order.
  std::vector<std::string> params_files;
  /// Resolve launch-style substitutions (`$(find-pkg-share pkg)`, `$(var name)`, `$(env NAME)`)
  /// in the parameter files, like `<param from="..." allow_substs="true"/>` does in launch files.
  bool allow_substs{true};
  /// ROS parameters as `name`/YAML-value pairs, applied after the parameter files.
  std::vector<std::pair<std::string, std::string>> parameters;
  /// Topic/service remappings as `from`/`to` pairs.
  std::vector<std::pair<std::string, std::string>> remappings;

  bool use_intra_process_comms{false};
  /// Number of executor threads spinning the driver node (1: single-threaded executor).
  std::size_t executor_threads{1};

  /// Topic whose messages feed the `message_*` and `header_stamp` state interfaces.
  /// Resolved relative to `node_namespace`. Empty: no monitoring.
  std::string monitor_topic;
  std::string monitor_topic_type{"sensor_msgs/msg/PointCloud2"};
};

/// Prefix of hardware `<param>` names that are forwarded as ROS parameters to the driver node,
/// e.g. `<param name="ros.sensor_model">Pandar64</param>` becomes `sensor_model: Pandar64`.
inline constexpr const char * k_ros_parameter_prefix = "ros.";

/// Build the driver configuration from the `<hardware>` parameters and the parameters of the
/// (optional) `<sensor>` tag.
///
/// @param hardware_name Name of the `<ros2_control>` block. The default node name is
/// `<hardware_name>_driver`.
/// @param hardware_parameters `<param>` tags of `<hardware>`.
/// @param sensor_parameters `<param>` tags of `<sensor>`. Only `frame_id` and `ros.*` are used.
/// @throws ConfigError on unknown or invalid parameters.
DriverConfig parse_driver_config(
  const std::string & hardware_name,
  const std::unordered_map<std::string, std::string> & hardware_parameters,
  const std::unordered_map<std::string, std::string> & sensor_parameters = {});

/// Build the `--ros-args ...` argument list that makes rcl apply the driver configuration
/// (node name, namespace, parameter files, parameters and remappings) to the driver node.
std::vector<std::string> build_ros_arguments(const DriverConfig & config);

/// The class name rclcpp_components registers a component's factory under.
std::string component_factory_class_name(const std::string & plugin);

/// Read `header.stamp` (in seconds) from a CDR/XCDR2-serialized message whose first field is a
/// `std_msgs/msg/Header` (true for `sensor_msgs/msg/PointCloud2`, `nebula_msgs/msg/NebulaPackets`
/// and most sensor messages). Returns std::nullopt if the buffer is too short or the encoding is
/// unknown.
std::optional<double> parse_header_stamp(const std::uint8_t * data, std::size_t size);

/// Resolves one `$(kind argument)` substitution. Throws ConfigError if it cannot be resolved.
using SubstitutionResolver =
  std::function<std::string(const std::string & kind, const std::string & argument)>;

/// Replace every `$(kind argument)` in `text` by `resolver(kind, argument)`.
/// Nested substitutions are not supported. Throws ConfigError for unterminated ones.
std::string apply_substitutions(const std::string & text, const SubstitutionResolver & resolver);

/// Value of `$(var name)`: the ROS parameter `name` given in the URDF (`ros.<name>` or
/// `frame_id`), with surrounding YAML quotes removed. Throws ConfigError if it is not set.
std::string resolve_var_substitution(const DriverConfig & config, const std::string & name);

/// Split `value` on `delimiter`, trimming whitespace and dropping empty items.
std::vector<std::string> split_list(const std::string & value, char delimiter = ';');

}  // namespace nebula::ros2_control

#endif  // NEBULA_ROS2_CONTROL__DRIVER_CONFIG_HPP_
