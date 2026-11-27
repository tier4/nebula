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

#include "nebula_ros2_control/nebula_sensor_interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialized_message.hpp>

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace nebula::ros2_control
{

namespace
{

using hardware_interface::CallbackReturn;

std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

constexpr double k_ns_to_s = 1e-9;
constexpr std::int64_t k_rate_window_ns = 1'000'000'000;

/// Deletes the temporary parameter files when going out of scope.
class TemporaryFiles
{
public:
  TemporaryFiles() = default;
  TemporaryFiles(const TemporaryFiles &) = delete;
  TemporaryFiles & operator=(const TemporaryFiles &) = delete;
  ~TemporaryFiles()
  {
    for (const auto & path : paths_) {
      std::error_code ignored;
      std::filesystem::remove(path, ignored);
    }
  }

  /// Write `content` to a new temporary file and return its path.
  std::string create(const std::string & content)
  {
    std::string path =
      (std::filesystem::temp_directory_path() / "nebula_ros2_control_XXXXXX.yaml").string();
    const int fd = mkstemps(path.data(), 5);
    if (fd < 0) {
      throw std::runtime_error("Failed to create a temporary parameter file");
    }
    close(fd);
    paths_.push_back(path);

    std::ofstream file(path, std::ios::trunc);
    file << content;
    if (!file) {
      throw std::runtime_error("Failed to write temporary parameter file '" + path + "'");
    }
    return path;
  }

private:
  std::vector<std::string> paths_;
};

/// Resolve launch-style substitutions in the parameter files, writing the results to temporary
/// files. Files without substitutions are passed through unchanged.
DriverConfig resolve_params_files(const DriverConfig & config, TemporaryFiles & temporary_files)
{
  DriverConfig resolved = config;
  if (!config.allow_substs) {
    return resolved;
  }

  const SubstitutionResolver resolver = [&config](
                                          const std::string & kind, const std::string & argument) {
    if (kind == "find-pkg-share") {
      try {
        return ament_index_cpp::get_package_share_directory(argument);
      } catch (const std::exception &) {
        throw ConfigError("$(find-pkg-share " + argument + "): package not found");
      }
    }
    if (kind == "var") {
      return resolve_var_substitution(config, argument);
    }
    if (kind == "env") {
      const char * value = std::getenv(argument.c_str());
      if (value == nullptr) {
        throw ConfigError("$(env " + argument + "): environment variable is not set");
      }
      return std::string{value};
    }
    throw ConfigError(
      "Unsupported substitution $(" + kind + " ...). Supported: find-pkg-share, var, env");
  };

  resolved.params_files.clear();
  for (const auto & path : config.params_files) {
    std::ifstream file(path);
    if (!file) {
      throw ConfigError("Cannot read parameter file '" + path + "'");
    }
    std::stringstream content;
    content << file.rdbuf();
    const std::string text = content.str();
    if (text.find("$(") == std::string::npos) {
      resolved.params_files.push_back(path);
      continue;
    }
    try {
      resolved.params_files.push_back(temporary_files.create(apply_substitutions(text, resolver)));
    } catch (const ConfigError & e) {
      throw ConfigError("In parameter file '" + path + "': " + e.what());
    }
  }
  return resolved;
}

}  // namespace

NebulaSensorInterface::~NebulaSensorInterface()
{
  stop_driver();
  loader_.reset();
}

CallbackReturn NebulaSensorInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  // The base implementation only stores the info; it is deprecated on newer ros2_control
  // releases in favor of a params-struct overload that still forwards to this one.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (hardware_interface::SensorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
#pragma GCC diagnostic pop

  logger_ = rclcpp::get_logger("nebula_ros2_control." + info_.name);

  if (info_.sensors.size() > 1) {
    RCLCPP_FATAL(
      logger_,
      "Expected at most one <sensor> in <ros2_control name=\"%s\">, got %zu. Declare one "
      "<ros2_control> block per Nebula sensor.",
      info_.name.c_str(), info_.sensors.size());
    return CallbackReturn::ERROR;
  }
  if (!info_.joints.empty() || !info_.gpios.empty()) {
    RCLCPP_FATAL(
      logger_, "<ros2_control name=\"%s\"> must not contain <joint> or <gpio> tags",
      info_.name.c_str());
    return CallbackReturn::ERROR;
  }

  try {
    config_ = info_.sensors.empty()
                ? parse_driver_config(info_.name, info_.hardware_parameters)
                : parse_driver_config(
                    info_.name, info_.hardware_parameters, info_.sensors.front().parameters);

    states_.clear();
    if (!info_.sensors.empty()) {
      const auto & sensor = info_.sensors.front();
      sensor_name_ = sensor.name;
      if (!sensor.command_interfaces.empty()) {
        throw ConfigError("Nebula sensors do not support command interfaces");
      }
      for (const auto & state_interface : sensor.state_interfaces) {
        const auto kind = parse_state_interface_kind(state_interface.name);
        if (requires_monitor_topic(kind) && config_.monitor_topic.empty()) {
          throw ConfigError(
            "State interface '" + state_interface.name +
            "' requires the hardware parameter 'monitor_topic' (e.g. the point cloud topic)");
        }
        ExportedState state{kind, state_interface.name, 0.0};
        if (kind == StateInterfaceKind::message_age || kind == StateInterfaceKind::header_stamp) {
          state.value = std::numeric_limits<double>::quiet_NaN();
        }
        states_.push_back(state);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Invalid configuration: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_, "Configured driver '%s' as node '%s/%s'", config_.plugin.c_str(),
    config_.node_namespace.c_str(), config_.node_name.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn NebulaSensorInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (loader_) {
    return CallbackReturn::SUCCESS;
  }
  try {
    loader_ = std::make_unique<ComponentLoader>(config_.plugin, config_.package, config_.library);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    logger_, "Loaded driver '%s' from '%s'", config_.plugin.c_str(),
    loader_->library_path().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn NebulaSensorInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_driver();
  loader_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn NebulaSensorInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!loader_) {
    RCLCPP_ERROR(logger_, "Cannot activate: driver library is not loaded");
    return CallbackReturn::ERROR;
  }
  return start_driver() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn NebulaSensorInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_driver();
  return CallbackReturn::SUCCESS;
}

CallbackReturn NebulaSensorInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop_driver();
  loader_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn NebulaSensorInterface::on_error(const rclcpp_lifecycle::State &)
{
  stop_driver();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NebulaSensorInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(states_.size());
  // The pointer-based constructor is deprecated on newer ros2_control releases but is the only one
  // available on all supported distributions (Humble and Jazzy).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  for (auto & state : states_) {
    interfaces.emplace_back(sensor_name_, state.name, &state.value);
  }
#pragma GCC diagnostic pop
  return interfaces;
}

hardware_interface::return_type NebulaSensorInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (states_.empty()) {
    return hardware_interface::return_type::OK;
  }

  // The executor thread exits early only if the driver threw from one of its callbacks.
  const bool active = active_.load() && !spin_finished_.load();
  const std::int64_t now_ns = steady_now_ns();

  const std::uint64_t count = monitor_state_->message_count.load();
  const std::int64_t last_receive_ns = monitor_state_->last_receive_ns.load();
  const double header_stamp = monitor_state_->header_stamp.load();

  if (!active) {
    message_rate_ = 0.0;
    rate_window_start_ns_ = 0;
  } else if (rate_window_start_ns_ == 0) {
    rate_window_start_ns_ = now_ns;
    rate_window_start_count_ = count;
  } else if (now_ns - rate_window_start_ns_ >= k_rate_window_ns) {
    const double elapsed_s = static_cast<double>(now_ns - rate_window_start_ns_) * k_ns_to_s;
    message_rate_ = static_cast<double>(count - rate_window_start_count_) / elapsed_s;
    rate_window_start_ns_ = now_ns;
    rate_window_start_count_ = count;
  }

  for (auto & state : states_) {
    switch (state.kind) {
      case StateInterfaceKind::driver_active:
        state.value = active ? 1.0 : 0.0;
        break;
      case StateInterfaceKind::message_count:
        state.value = static_cast<double>(count);
        break;
      case StateInterfaceKind::message_rate:
        state.value = message_rate_;
        break;
      case StateInterfaceKind::message_age:
        state.value = last_receive_ns == 0
                        ? std::numeric_limits<double>::quiet_NaN()
                        : static_cast<double>(now_ns - last_receive_ns) * k_ns_to_s;
        break;
      case StateInterfaceKind::header_stamp:
        state.value = header_stamp;
        break;
    }
  }
  return hardware_interface::return_type::OK;
}

bool NebulaSensorInterface::start_driver()
{
  if (active_) {
    return true;
  }

  try {
    // rcl parses the parameter files while constructing the node, so the temporary files with
    // resolved substitutions are only needed until then.
    TemporaryFiles temporary_files;
    const DriverConfig resolved_config = resolve_params_files(config_, temporary_files);

    rclcpp::NodeOptions options;
    // Global arguments are those of the controller manager (e.g. its own node name remapping)
    // and must not leak into the driver node.
    options.use_global_arguments(false);
    options.arguments(build_ros_arguments(resolved_config));
    options.use_intra_process_comms(config_.use_intra_process_comms);

    driver_node_ = loader_->create_node(options);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to start driver '%s': %s", config_.plugin.c_str(), e.what());
    driver_node_.reset();
    return false;
  }

  monitor_state_->message_count = 0;
  monitor_state_->last_receive_ns = 0;
  monitor_state_->header_stamp = std::numeric_limits<double>::quiet_NaN();
  rate_window_start_ns_ = 0;
  message_rate_ = 0.0;

  if (!config_.monitor_topic.empty()) {
    try {
      rclcpp::NodeOptions monitor_options;
      monitor_options.use_global_arguments(false);
      monitor_options.start_parameter_services(false);
      monitor_options.start_parameter_event_publisher(false);
      monitor_node_ = std::make_shared<rclcpp::Node>(
        config_.node_name + "_ros2_control_monitor", config_.node_namespace, monitor_options);

      std::weak_ptr<MonitorState> weak_state = monitor_state_;
      std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback =
        [weak_state](std::shared_ptr<rclcpp::SerializedMessage> message) {
          auto state = weak_state.lock();
          if (!state || !message) {
            return;
          }
          const auto & raw = message->get_rcl_serialized_message();
          if (auto stamp = parse_header_stamp(raw.buffer, raw.buffer_length)) {
            state->header_stamp.store(*stamp);
          }
          state->last_receive_ns.store(steady_now_ns());
          state->message_count.fetch_add(1);
        };

      monitor_subscription_ = monitor_node_->create_generic_subscription(
        config_.monitor_topic, config_.monitor_topic_type, rclcpp::SensorDataQoS(),
        std::move(callback));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Failed to monitor topic '%s' (%s): %s", config_.monitor_topic.c_str(),
        config_.monitor_topic_type.c_str(), e.what());
      monitor_subscription_.reset();
      monitor_node_.reset();
      driver_node_.reset();
      return false;
    }
  }

  if (config_.executor_threads > 1) {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), config_.executor_threads);
  } else {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }
  executor_->add_node(driver_node_->get_node_base_interface());
  if (monitor_node_) {
    executor_->add_node(monitor_node_->get_node_base_interface());
  }

  spin_finished_ = false;
  spin_thread_ = std::thread([this]() {
    try {
      executor_->spin();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Driver executor stopped with an exception: %s", e.what());
    }
    spin_finished_ = true;
  });

  active_ = true;
  RCLCPP_INFO(
    logger_, "Driver '%s' running as '%s'", config_.plugin.c_str(),
    driver_node_->get_node_base_interface()->get_fully_qualified_name());
  return true;
}

void NebulaSensorInterface::stop_driver()
{
  active_ = false;

  if (spin_thread_.joinable()) {
    // cancel() is a no-op if called before spin() has started, so repeat until the thread is out.
    while (!spin_finished_) {
      executor_->cancel();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    spin_thread_.join();
  }

  if (executor_) {
    if (monitor_node_) {
      executor_->remove_node(monitor_node_->get_node_base_interface());
    }
    if (driver_node_) {
      executor_->remove_node(driver_node_->get_node_base_interface());
    }
    executor_.reset();
  }

  monitor_subscription_.reset();
  monitor_node_.reset();
  // Destroying the driver node disconnects from the sensor. It must happen before the library
  // providing it is unloaded (loader_).
  driver_node_.reset();
}

}  // namespace nebula::ros2_control

PLUGINLIB_EXPORT_CLASS(
  nebula::ros2_control::NebulaSensorInterface, hardware_interface::SensorInterface)
