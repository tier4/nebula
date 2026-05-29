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

#include "nebula_ouster/ouster_ros_wrapper.hpp"

#include "nebula_ouster_decoders/ouster_metadata.hpp"
#include "nebula_ouster_decoders/ouster_packet.hpp"

#include <nebula_core_common/util/expected.hpp>
#include <nebula_core_ros/parameter_descriptors.hpp>
#include <nebula_core_ros/point_cloud_conversions.hpp>
#include <nebula_ouster_common/ouster_configuration.hpp>
#include <nebula_ouster_hw_interfaces/ouster_hw_interface.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
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

util::expected<std::monostate, ConfigError> validate_fov(
  const drivers::FieldOfView<float, drivers::Degrees> & fov)
{
  if (fov.azimuth.start < 0.0F || fov.azimuth.start >= 360.0F) {
    return ConfigError{
      ConfigError::Code::PARAMETER_VALIDATION_FAILED,
      "Parameter 'fov.azimuth.min_deg' must be in [0, 360), got " +
        std::to_string(fov.azimuth.start)};
  }
  if (fov.azimuth.end <= fov.azimuth.start || fov.azimuth.end > 360.0F) {
    return ConfigError{
      ConfigError::Code::PARAMETER_VALIDATION_FAILED,
      "Parameter 'fov.azimuth.max_deg' must be in (" + std::to_string(fov.azimuth.start) +
        ", 360], got " + std::to_string(fov.azimuth.end)};
  }
  if (fov.elevation.start < -90.0F || fov.elevation.start >= 90.0F) {
    return ConfigError{
      ConfigError::Code::PARAMETER_VALIDATION_FAILED,
      "Parameter 'fov.elevation.min_deg' must be in [-90, 90), got " +
        std::to_string(fov.elevation.start)};
  }
  if (fov.elevation.end <= fov.elevation.start || fov.elevation.end > 90.0F) {
    return ConfigError{
      ConfigError::Code::PARAMETER_VALIDATION_FAILED,
      "Parameter 'fov.elevation.max_deg' must be in (" + std::to_string(fov.elevation.start) +
        ", 90], got " + std::to_string(fov.elevation.end)};
  }
  return std::monostate{};
}

std::string load_metadata_from_file(const std::string & path)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("Cannot open metadata_file for reading: " + path);
  }
  std::stringstream buffer;
  buffer << ifs.rdbuf();
  return buffer.str();
}

void save_metadata_to_file(const std::string & path, const std::string & json)
{
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    // Non-fatal: log at wrapper level instead.
    return;
  }
  ofs << json;
}
}  // namespace

util::expected<drivers::OusterSensorConfiguration, ConfigError> load_config_from_ros_parameters(
  rclcpp::Node & node)
{
  drivers::OusterSensorConfiguration config{};

  const auto host_ip =
    declare_required_parameter<std::string>(node, "connection.host_ip", param_read_only());
  if (!host_ip.has_value()) return host_ip.error();
  config.connection.host_ip = host_ip.value();
  if (!drivers::connections::parse_ip(config.connection.host_ip).has_value()) {
    return ConfigError{
      ConfigError::Code::PARAMETER_VALIDATION_FAILED,
      "Parameter 'connection.host_ip' must be a valid IPv4 address, got '" +
        config.connection.host_ip + "'"};
  }

  const auto sensor_ip =
    declare_required_parameter<std::string>(node, "connection.sensor_ip", param_read_only());
  if (!sensor_ip.has_value()) return sensor_ip.error();
  config.connection.sensor_ip = sensor_ip.value();
  if (!drivers::connections::parse_ip(config.connection.sensor_ip).has_value()) {
    return ConfigError{
      ConfigError::Code::PARAMETER_VALIDATION_FAILED,
      "Parameter 'connection.sensor_ip' must be a valid IPv4 address, got '" +
        config.connection.sensor_ip + "'"};
  }

  const auto data_port =
    declare_required_parameter<int64_t>(node, "connection.data_port", param_read_only());
  if (!data_port.has_value()) return data_port.error();
  if (data_port.value() <= 0 || data_port.value() > 65535) {
    return ConfigError{
      ConfigError::Code::PARAMETER_VALIDATION_FAILED,
      "Parameter 'connection.data_port' must be in [1, 65535], got " +
        std::to_string(data_port.value())};
  }
  config.connection.data_port = static_cast<uint16_t>(data_port.value());

  const auto imu_port =
    node.declare_parameter<int64_t>("connection.imu_port", 0, param_read_only());
  if (imu_port < 0 || imu_port > 65535) {
    return ConfigError{
      ConfigError::Code::PARAMETER_VALIDATION_FAILED,
      "Parameter 'connection.imu_port' must be in [0, 65535], got " + std::to_string(imu_port)};
  }
  config.connection.imu_port = static_cast<uint16_t>(imu_port);

  config.connection.filter_sender_ip =
    node.declare_parameter<bool>("connection.filter_sender_ip", true, param_read_only());

  const auto azimuth_min =
    declare_required_parameter<double>(node, "fov.azimuth.min_deg", param_read_write());
  if (!azimuth_min.has_value()) return azimuth_min.error();
  const auto azimuth_max =
    declare_required_parameter<double>(node, "fov.azimuth.max_deg", param_read_write());
  if (!azimuth_max.has_value()) return azimuth_max.error();
  const auto elevation_min =
    declare_required_parameter<double>(node, "fov.elevation.min_deg", param_read_write());
  if (!elevation_min.has_value()) return elevation_min.error();
  const auto elevation_max =
    declare_required_parameter<double>(node, "fov.elevation.max_deg", param_read_write());
  if (!elevation_max.has_value()) return elevation_max.error();

  config.fov.azimuth.start = static_cast<float>(azimuth_min.value());
  config.fov.azimuth.end = static_cast<float>(azimuth_max.value());
  config.fov.elevation.start = static_cast<float>(elevation_min.value());
  config.fov.elevation.end = static_cast<float>(elevation_max.value());

  const auto fov_validation = validate_fov(config.fov);
  if (!fov_validation.has_value()) return fov_validation.error();

  return config;
}

OusterRosWrapper::OusterRosWrapper(const rclcpp::NodeOptions & options)
: Node("nebula_ouster_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  diagnostics_(this),
  runtime_mode_(std::monostate{})
{
  const bool launch_hw = declare_parameter<bool>("launch_hw", true, param_read_only());
  declare_parameter<std::string>("sensor_model", "OS-128", param_read_only());
  frame_id_ = declare_parameter<std::string>("frame_id", "ouster_lidar", param_read_write());
  const std::string metadata_file =
    declare_parameter<std::string>("metadata_file", "", param_read_only());

  const auto config_or_error = load_config_from_ros_parameters(*this);
  if (!config_or_error.has_value()) {
    throw std::runtime_error(
      "Invalid ouster sensor configuration: " + config_or_error.error().message);
  }
  config_ = config_or_error.value();

  publishers_.points =
    create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
  publishers_.imu = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
  publishers_.receive_duration_ms =
    create_publisher<std_msgs::msg::Float64>("debug/receive_duration_ms", 10);
  publishers_.decode_duration_ms =
    create_publisher<std_msgs::msg::Float64>("debug/decode_duration_ms", 10);
  publishers_.publish_duration_ms =
    create_publisher<std_msgs::msg::Float64>("debug/publish_duration_ms", 10);

  initialize_diagnostics();

  // Metadata acquisition: file (offline) or HTTP (online, with optional file cache).
  std::string metadata_json;
  if (!metadata_file.empty() && !launch_hw) {
    RCLCPP_INFO(get_logger(), "Loading Ouster metadata from file: %s", metadata_file.c_str());
    metadata_json = load_metadata_from_file(metadata_file);
  } else {
    RCLCPP_INFO(
      get_logger(), "Fetching Ouster metadata via HTTP from %s",
      config_.connection.sensor_ip.c_str());
    metadata_json = drivers::fetch_ouster_metadata_http(config_.connection.sensor_ip);
    if (!metadata_file.empty()) {
      save_metadata_to_file(metadata_file, metadata_json);
      RCLCPP_INFO(get_logger(), "Cached metadata to %s", metadata_file.c_str());
    }
  }
  RCLCPP_INFO(get_logger(), "Got Ouster metadata (%zu bytes)", metadata_json.size());

  auto metadata = drivers::parse_ouster_metadata(metadata_json);
  config_.connection.receiver_mtu_bytes =
    static_cast<uint32_t>(metadata.lidar_packet_size_bytes);

  RCLCPP_INFO(
    get_logger(),
    "Ouster UDP: listening on %s:%u filter_sender_ip=%s (sensor_ip=%s) receiver_mtu=%u "
    "(profile=%d, %ux%u beams, %u columns_per_packet)",
    config_.connection.host_ip.c_str(), config_.connection.data_port,
    config_.connection.filter_sender_ip ? "true" : "false", config_.connection.sensor_ip.c_str(),
    config_.connection.receiver_mtu_bytes, static_cast<int>(metadata.udp_profile_lidar),
    metadata.pixels_per_column, metadata.columns_per_frame, metadata.columns_per_packet);

  decoder_.emplace(
    config_.fov, std::move(metadata),
    [this](const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s) {
      publish_pointcloud_callback(pointcloud, timestamp_s);
    });
  decoder_->set_imu_callback(
    [this](const drivers::OusterImuSample & sample) { publish_imu_callback(sample); });

  if (launch_hw) {
    runtime_mode_.emplace<OnlineMode>(config_.connection);
    auto & online_mode = std::get<OnlineMode>(runtime_mode_);
    online_mode.packets_pub =
      create_publisher<nebula_msgs::msg::NebulaPackets>("packets", rclcpp::SensorDataQoS());
    const auto callback_result = online_mode.hw_interface.register_scan_callback(
      [this](
        std::vector<uint8_t> & raw_packet,
        const drivers::connections::UdpSocket::RxMetadata & metadata) {
        receive_cloud_packet_callback(raw_packet, metadata);
      });
    if (!callback_result.has_value()) {
      throw std::runtime_error(
        "Failed to register ouster sensor packet callback: " + callback_result.error().message);
    }

    // Register a separate callback for the IMU socket (when imu_port is set). This keeps the
    // IMU thread completely isolated from the lidar thread — no shared state, no races.
    if (config_.connection.imu_port != 0 &&
        config_.connection.imu_port != config_.connection.data_port) {
      const auto imu_result = online_mode.hw_interface.register_imu_callback(
        [this](
          std::vector<uint8_t> & raw_packet,
          const drivers::connections::UdpSocket::RxMetadata & /*metadata*/) {
          if (!decoder_) return;
          if (raw_packet.size() != decoder_->metadata().imu_packet_size_bytes) return;
          const auto raw = drivers::ouster_packet::parse_imu_packet(raw_packet.data());
          drivers::OusterImuSample s{};
          s.timestamp_ns = raw.sys_timestamp_ns;
          constexpr float k_g = 9.80665f;
          constexpr float k_deg_to_rad = static_cast<float>(M_PI) / 180.0f;
          s.accel_x = raw.accel_x_g * k_g;
          s.accel_y = raw.accel_y_g * k_g;
          s.accel_z = raw.accel_z_g * k_g;
          s.gyro_x = raw.gyro_x_dps * k_deg_to_rad;
          s.gyro_y = raw.gyro_y_dps * k_deg_to_rad;
          s.gyro_z = raw.gyro_z_dps * k_deg_to_rad;
          publish_imu_callback(s);
        });
      if (!imu_result.has_value()) {
        throw std::runtime_error(
          "Failed to register ouster IMU callback: " + imu_result.error().message);
      }
    }

    const auto stream_start_result = online_mode.hw_interface.sensor_interface_start();
    if (!stream_start_result.has_value()) {
      throw std::runtime_error(
        "Failed to start ouster sensor stream: " + stream_start_result.error().message);
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

OusterRosWrapper::~OusterRosWrapper()
{
  auto * online_mode = std::get_if<OnlineMode>(&runtime_mode_);
  if (!online_mode) return;

  const auto stop_result = online_mode->hw_interface.sensor_interface_stop();
  if (!stop_result.has_value()) {
    RCLCPP_WARN(
      get_logger(), "Failed to stop ouster sensor stream cleanly: %s",
      stop_result.error().message.c_str());
  }
}

void OusterRosWrapper::initialize_diagnostics()
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

void OusterRosWrapper::publish_pointcloud_callback(
  const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s)
{
  if (!pointcloud) return;

  auto ros_pc_msg_ptr =
    std::make_unique<sensor_msgs::msg::PointCloud2>(nebula::ros::to_ros_msg(*pointcloud));
  ros_pc_msg_ptr->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_s * 1e9));
  ros_pc_msg_ptr->header.frame_id = frame_id_;
  publishers_.points->publish(std::move(ros_pc_msg_ptr));

  diagnostics_.publish_rate->tick();
}

void OusterRosWrapper::publish_imu_callback(const drivers::OusterImuSample & sample)
{
  if (!publishers_.imu) return;

  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.stamp = rclcpp::Time(static_cast<int64_t>(sample.timestamp_ns));
  msg->header.frame_id = frame_id_;

  msg->linear_acceleration.x = sample.accel_x;
  msg->linear_acceleration.y = sample.accel_y;
  msg->linear_acceleration.z = sample.accel_z;
  msg->angular_velocity.x = sample.gyro_x;
  msg->angular_velocity.y = sample.gyro_y;
  msg->angular_velocity.z = sample.gyro_z;
  // Orientation is not provided by the Ouster IMU; mark covariance[0] = -1 per ROS convention.
  msg->orientation_covariance[0] = -1.0;

  publishers_.imu->publish(std::move(msg));
}

void OusterRosWrapper::receive_cloud_packet_callback(
  std::vector<uint8_t> & packet, const drivers::connections::UdpSocket::RxMetadata & metadata)
{
  diagnostics_.packet_liveness->tick();

  if (metadata.truncated) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ouster UDP payload was truncated at %zu bytes (kernel/datagram is larger than "
      "connection.receiver_mtu). Decoding is skipped for this datagram.",
      packet.size());
    return;
  }

  auto * online_mode = std::get_if<OnlineMode>(&runtime_mode_);
  if (online_mode && online_mode->packets_pub) {
    // Two socket threads (lidar+imu) can enter this function concurrently — serialize the
    // packets_msg append below so the vector isn't corrupted.
    std::lock_guard<std::mutex> guard(online_mode->packets_mutex);
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

void OusterRosWrapper::receive_packets_message_callback(
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> packets_msg)
{
  if (!packets_msg) return;

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

void OusterRosWrapper::process_packet(
  const std::vector<uint8_t> & packet, const uint64_t receive_duration_ns)
{
  if (!decoder_) return;

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

void OusterRosWrapper::publish_debug_durations(
  uint64_t receive_duration_ns, uint64_t decode_duration_ns, uint64_t publish_duration_ns) const
{
  const auto publish_metric = [](const auto & publisher, const uint64_t duration_ns) {
    if (!publisher) return;
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

const char * OusterRosWrapper::to_cstr(const Error::Code code)
{
  switch (code) {
    case Error::Code::HW_INTERFACE_NOT_INITIALIZED:
      return "hardware interface not initialized";
    case Error::Code::HW_STREAM_START_FAILED:
      return "hardware stream start failed";
  }
  return "unknown wrapper error";
}

}  // namespace nebula::ros

RCLCPP_COMPONENTS_REGISTER_NODE(nebula::ros::OusterRosWrapper)
