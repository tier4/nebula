// Copyright 2025 TIER IV, Inc.
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

#include "nebula_sample/sample_ros_wrapper.hpp"

#include <nebula_core_common/util/expected.hpp>
#include <nebula_core_ros/parameter_descriptors.hpp>
#include <nebula_sample_common/sample_configuration.hpp>
#include <nebula_sample_hw_interfaces/sample_hw_interface.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/float64.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebula::ros
{

namespace
{
constexpr double k_ns_to_ms = 1e-6;

uint64_t current_system_time_ns()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count());
}

template <typename T>
util::expected<T, ConfigError> declare_required_parameter(
  rclcpp::Node & node, const std::string & name,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  try {
    return node.declare_parameter<T>(name, descriptor);
  } catch (const std::exception & e) {
    return ConfigError{
      ConfigError::Code::PARAMETER_DECLARATION_FAILED,
      "Failed to declare/read parameter '" + name + "': " + e.what()};
  }
}
}  // namespace

util::expected<drivers::SampleSensorConfiguration, ConfigError> load_config_from_ros_parameters(
  rclcpp::Node & node)
{
  drivers::SampleSensorConfiguration config{};

  const auto host_ip =
    declare_required_parameter<std::string>(node, "connection.host_ip", param_read_only());
  if (!host_ip.has_value()) {
    return host_ip.error();
  }
  config.connection.host_ip = host_ip.value();
  if (!drivers::connections::parse_ip(config.connection.host_ip).has_value()) {
    return ConfigError{
      ConfigError::Code::PARAMETER_VALIDATION_FAILED,
      "Parameter 'connection.host_ip' must be a valid IPv4 address, got '" +
        config.connection.host_ip + "'"};
  }

  const auto sensor_ip =
    declare_required_parameter<std::string>(node, "connection.sensor_ip", param_read_only());
  if (!sensor_ip.has_value()) {
    return sensor_ip.error();
  }
  config.connection.sensor_ip = sensor_ip.value();
  if (!drivers::connections::parse_ip(config.connection.sensor_ip).has_value()) {
    return ConfigError{
      ConfigError::Code::PARAMETER_VALIDATION_FAILED,
      "Parameter 'connection.sensor_ip' must be a valid IPv4 address, got '" +
        config.connection.sensor_ip + "'"};
  }

  const auto data_port =
    declare_required_parameter<int64_t>(node, "connection.data_port", param_read_only());
  if (!data_port.has_value()) {
    return data_port.error();
  }
  if (data_port.value() <= 0 || data_port.value() > 65535) {
    return ConfigError{
      ConfigError::Code::PARAMETER_VALIDATION_FAILED,
      "Parameter 'connection.data_port' must be in [1, 65535], got " +
        std::to_string(data_port.value())};
  }
  config.connection.data_port = static_cast<uint16_t>(data_port.value());

  const auto azimuth_min =
    declare_required_parameter<double>(node, "fov.azimuth.min_deg", param_read_write());
  if (!azimuth_min.has_value()) {
    return azimuth_min.error();
  }
  const auto azimuth_max =
    declare_required_parameter<double>(node, "fov.azimuth.max_deg", param_read_write());
  if (!azimuth_max.has_value()) {
    return azimuth_max.error();
  }
  const auto elevation_min =
    declare_required_parameter<double>(node, "fov.elevation.min_deg", param_read_write());
  if (!elevation_min.has_value()) {
    return elevation_min.error();
  }
  const auto elevation_max =
    declare_required_parameter<double>(node, "fov.elevation.max_deg", param_read_write());
  if (!elevation_max.has_value()) {
    return elevation_max.error();
  }

  config.fov.azimuth.start = static_cast<float>(azimuth_min.value());
  config.fov.azimuth.end = static_cast<float>(azimuth_max.value());
  config.fov.elevation.start = static_cast<float>(elevation_min.value());
  config.fov.elevation.end = static_cast<float>(elevation_max.value());

  return config;
}

SampleRosWrapper::SampleRosWrapper(const rclcpp::NodeOptions & options)
: Node("nebula_sample_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  diagnostics_(this),
  runtime_mode_(std::monostate{})
{
  const bool launch_hw = declare_parameter<bool>("launch_hw", true, param_read_only());
  declare_parameter<std::string>("sensor_model", "SampleSensor", param_read_only());
  frame_id_ = declare_parameter<std::string>("frame_id", "sample_lidar", param_read_write());

  const auto config_or_error = load_config_from_ros_parameters(*this);
  if (!config_or_error.has_value()) {
    throw std::runtime_error(
      "Invalid sample sensor configuration: " + config_or_error.error().message);
  }
  config_ = config_or_error.value();

  publishers_.points =
    create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
  publishers_.receive_duration_ms =
    create_publisher<std_msgs::msg::Float64>("debug/receive_duration_ms", 10);
  publishers_.decode_duration_ms =
    create_publisher<std_msgs::msg::Float64>("debug/decode_duration_ms", 10);
  publishers_.publish_duration_ms =
    create_publisher<std_msgs::msg::Float64>("debug/publish_duration_ms", 10);

  initialize_diagnostics();

  decoder_.emplace(
    config_.fov, [this](const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s) {
      publish_pointcloud_callback(pointcloud, timestamp_s);
    });

  if (launch_hw) {
    runtime_mode_.emplace<OnlineMode>(config_.connection);
    auto & online_mode = std::get<OnlineMode>(runtime_mode_);
    online_mode.packets_pub =
      create_publisher<nebula_msgs::msg::NebulaPackets>("packets", rclcpp::SensorDataQoS());
    const auto callback_result = online_mode.hw_interface.register_scan_callback(
      [this](
        const std::vector<uint8_t> & raw_packet,
        const drivers::connections::UdpSocket::RxMetadata & metadata) {
        receive_cloud_packet_callback(raw_packet, metadata);
      });
    if (!callback_result.has_value()) {
      const auto error = callback_result.error();
      throw std::runtime_error(
        "Failed to register sample sensor packet callback: " + error.message);
    }

    const auto stream_start_result = online_mode.hw_interface.sensor_interface_start();
    if (!stream_start_result.has_value()) {
      const auto error = stream_start_result.error();
      throw std::runtime_error("Failed to start sample sensor stream: " + error.message);
    }
  } else {
    runtime_mode_.emplace<OfflineMode>();
    auto & offline_mode = std::get<OfflineMode>(runtime_mode_);
    offline_mode.packets_sub = create_subscription<nebula_msgs::msg::NebulaPackets>(
      "packets", rclcpp::SensorDataQoS(),
      [this](std::unique_ptr<nebula_msgs::msg::NebulaPackets> packets_msg) {
        receive_packets_message_callback(std::move(packets_msg));
      });
    RCLCPP_INFO(
      get_logger(), "Hardware connection disabled, listening for packets on %s",
      offline_mode.packets_sub->get_topic_name());
  }
}

SampleRosWrapper::~SampleRosWrapper()
{
  auto * online_mode = std::get_if<OnlineMode>(&runtime_mode_);
  if (!online_mode) {
    return;
  }

  const auto stop_result = online_mode->hw_interface.sensor_interface_stop();
  if (!stop_result.has_value()) {
    const auto error = stop_result.error();
    RCLCPP_WARN(
      get_logger(), "Failed to stop sample sensor stream cleanly: %s", error.message.c_str());
  }
}

void SampleRosWrapper::initialize_diagnostics()
{
  const double min_ok_hz = declare_parameter<double>(
    "diagnostics.pointcloud_publish_rate.frequency_ok.min_hz", param_read_only());
  const double max_ok_hz = declare_parameter<double>(
    "diagnostics.pointcloud_publish_rate.frequency_ok.max_hz", param_read_only());
  const double min_warn_hz = declare_parameter<double>(
    "diagnostics.pointcloud_publish_rate.frequency_warn.min_hz", param_read_only());
  const double max_warn_hz = declare_parameter<double>(
    "diagnostics.pointcloud_publish_rate.frequency_warn.max_hz", param_read_only());
  const auto num_frame_transition = static_cast<size_t>(declare_parameter<int64_t>(
    "diagnostics.pointcloud_publish_rate.num_frame_transition", param_read_only()));
  const auto packet_liveness_timeout_ms =
    declare_parameter<int64_t>("diagnostics.packet_liveness.timeout_ms", param_read_only());

  if (min_warn_hz >= min_ok_hz) {
    throw std::runtime_error(
      "Invalid diagnostics config: frequency_warn.min_hz must be smaller than "
      "frequency_ok.min_hz.");
  }
  if (max_warn_hz <= max_ok_hz) {
    throw std::runtime_error(
      "Invalid diagnostics config: frequency_warn.max_hz must be greater than "
      "frequency_ok.max_hz.");
  }
  if (packet_liveness_timeout_ms <= 0) {
    throw std::runtime_error(
      "Invalid diagnostics config: packet_liveness.timeout_ms must be positive.");
  }
  const auto packet_liveness_timeout = std::chrono::milliseconds(packet_liveness_timeout_ms);

  diagnostics_.publish_rate.emplace(
    this, custom_diagnostic_tasks::RateBoundStatusParam{min_ok_hz, max_ok_hz},
    custom_diagnostic_tasks::RateBoundStatusParam{min_warn_hz, max_warn_hz},
    std::max<size_t>(1, num_frame_transition), false, true, "pointcloud publish rate");
  diagnostics_.packet_liveness.emplace(
    "packet_receive", this,
    rclcpp::Duration::from_seconds(
      std::chrono::duration<double, std::milli>(packet_liveness_timeout).count() * 1e-3));

  diagnostics_.updater.setHardwareID(frame_id_);
  diagnostics_.updater.add(diagnostics_.publish_rate.value());
  diagnostics_.updater.add(diagnostics_.packet_liveness.value());
  diagnostics_.updater.force_update();
}

void SampleRosWrapper::publish_pointcloud_callback(
  const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s)
{
  if (!pointcloud) {
    return;
  }

  if (publishers_.points->get_subscription_count() > 0) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_s * 1e9));
    ros_pc_msg_ptr->header.frame_id = frame_id_;
    publishers_.points->publish(std::move(ros_pc_msg_ptr));
  }

  diagnostics_.publish_rate->tick();
}

void SampleRosWrapper::receive_cloud_packet_callback(
  const std::vector<uint8_t> & packet, const drivers::connections::UdpSocket::RxMetadata & metadata)
{
  diagnostics_.packet_liveness->tick();
  auto * online_mode = std::get_if<OnlineMode>(&runtime_mode_);
  if (online_mode && online_mode->packets_pub) {
    if (!online_mode->current_scan_packets_msg) {
      online_mode->current_scan_packets_msg = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    }

    const auto packet_timestamp_ns = metadata.timestamp_ns.value_or(current_system_time_ns());
    nebula_msgs::msg::NebulaPacket packet_msg{};
    packet_msg.stamp.sec = static_cast<int32_t>(packet_timestamp_ns / 1'000'000'000ULL);
    packet_msg.stamp.nanosec = static_cast<uint32_t>(packet_timestamp_ns % 1'000'000'000ULL);
    packet_msg.data = packet;
    if (online_mode->current_scan_packets_msg->packets.empty()) {
      online_mode->current_scan_packets_msg->header.stamp = packet_msg.stamp;
      online_mode->current_scan_packets_msg->header.frame_id = frame_id_;
    }
    online_mode->current_scan_packets_msg->packets.emplace_back(std::move(packet_msg));
  }

  process_packet(packet, metadata.packet_perf_counters.receive_duration_ns);
}

void SampleRosWrapper::receive_packets_message_callback(
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> packets_msg)
{
  if (!packets_msg) {
    return;
  }

  if (!std::holds_alternative<OfflineMode>(runtime_mode_)) {
    RCLCPP_ERROR_ONCE(
      get_logger(),
      "Ignoring NebulaPackets. Launch with launch_hw:=false to enable NebulaPackets replay.");
    return;
  }

  for (const auto & packet_msg : packets_msg->packets) {
    diagnostics_.packet_liveness->tick();
    process_packet(packet_msg.data, 0U);
  }
}

void SampleRosWrapper::process_packet(
  const std::vector<uint8_t> & packet, const uint64_t receive_duration_ns)
{
  if (!decoder_) {
    return;
  }

  const auto decode_result = decoder_->unpack(packet);
  publish_debug_durations(
    receive_duration_ns, decode_result.performance_counters.decode_time_ns,
    decode_result.performance_counters.callback_time_ns);

  if (!decode_result.metadata_or_error.has_value()) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000, "Packet decode failed: %s.",
      drivers::to_cstr(decode_result.metadata_or_error.error()));
    return;
  }

  auto * online_mode = std::get_if<OnlineMode>(&runtime_mode_);
  if (
    online_mode && online_mode->packets_pub &&
    decode_result.metadata_or_error.value().did_scan_complete &&
    online_mode->current_scan_packets_msg &&
    !online_mode->current_scan_packets_msg->packets.empty()) {
    online_mode->packets_pub->publish(std::move(online_mode->current_scan_packets_msg));
    online_mode->current_scan_packets_msg = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  }
}

void SampleRosWrapper::publish_debug_durations(
  uint64_t receive_duration_ns, uint64_t decode_duration_ns, uint64_t publish_duration_ns) const
{
  const auto publish_metric = [](const auto & publisher, const uint64_t duration_ns) {
    if (!publisher) {
      return;
    }
    if (
      publisher->get_subscription_count() == 0 &&
      publisher->get_intra_process_subscription_count() == 0) {
      return;
    }

    std_msgs::msg::Float64 msg;
    msg.data = static_cast<double>(duration_ns) * k_ns_to_ms;
    publisher->publish(msg);
  };

  publish_metric(publishers_.receive_duration_ms, receive_duration_ns);
  publish_metric(publishers_.decode_duration_ms, decode_duration_ns);
  publish_metric(publishers_.publish_duration_ms, publish_duration_ns);
}

const char * SampleRosWrapper::to_cstr(const Error::Code code)
{
  switch (code) {
    case Error::Code::HW_INTERFACE_NOT_INITIALIZED:
      return "hardware interface not initialized";
    case Error::Code::HW_STREAM_START_FAILED:
      return "hardware stream start failed";
    default:
      return "unknown wrapper error";
  }
}

}  // namespace nebula::ros

RCLCPP_COMPONENTS_REGISTER_NODE(nebula::ros::SampleRosWrapper)
