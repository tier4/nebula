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

#include "nebula_continental/continental_ars548_decoder_wrapper.hpp"

#include "nebula_continental_common/continental_ars548.hpp"
#include "nebula_core_ros/sync_tooling/sync_tooling_worker.hpp"

#include <nebula_core_common/util/string_conversions.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace nebula::ros
{
using std::chrono_literals::operator""ms;

ContinentalARS548DecoderWrapper::ContinentalARS548DecoderWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration> &
    config_ptr,
  bool launch_hw)
: objects_rate_bound_status_(make_rate_bound_status(parent_node, "Objects rate bound status")),
  detections_rate_bound_status_(
    make_rate_bound_status(parent_node, "Detections rate bound status")),
  blockage_status_(make_hysteresis_status(parent_node, "blockage")),
  internal_status_(make_hysteresis_status(parent_node, "internal")),
  liveness_monitor_("Liveness", parent_node, 100ms),
  objects_diagnostics_updater_(parent_node),
  detections_diagnostics_updater_(parent_node),
  liveness_diagnostics_updater_(parent_node),
  status_(nebula::Status::NOT_INITIALIZED),
  logger_(parent_node->get_logger().get_child("ContinentalARS548Decoder")),
  parent_node_(parent_node),
  config_ptr_(config_ptr)
{
  using std::chrono_literals::operator""us;
  if (!config_ptr) {
    throw std::runtime_error(
      "ContinentalARS548DecoderWrapper cannot be instantiated without a valid config!");
  }

  RCLCPP_INFO(logger_, "Starting Decoder");

  initialize_driver(config_ptr);
  status_ = driver_ptr_->get_status();

  if (Status::OK != status_) {
    throw std::runtime_error("Error instantiating decoder: " + util::to_string(status_));
  }

  initialize_sync_diagnostics(parent_node);

  // Publish packets only if HW interface is connected
  if (launch_hw) {
    packets_pub_ = parent_node->create_publisher<nebula_msgs::msg::NebulaPackets>(
      "nebula_packets", rclcpp::SensorDataQoS());
  }

  detection_list_pub_ =
    parent_node->create_publisher<continental_msgs::msg::ContinentalArs548DetectionList>(
      "continental_detections", rclcpp::SensorDataQoS());
  object_list_pub_ =
    parent_node->create_publisher<continental_msgs::msg::ContinentalArs548ObjectList>(
      "continental_objects", rclcpp::SensorDataQoS());

  detection_pointcloud_pub_ = parent_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "detection_points", rclcpp::SensorDataQoS());
  object_pointcloud_pub_ = parent_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "object_points", rclcpp::SensorDataQoS());

  autoware_objects_pub_ = parent_node->create_publisher<autoware_sensing_msgs::msg::RadarObjects>(
    "radar_objects", rclcpp::SensorDataQoS());

  scan_raw_pub_ =
    parent_node->create_publisher<radar_msgs::msg::RadarScan>("scan_raw", rclcpp::SensorDataQoS());

  objects_raw_pub_ = parent_node->create_publisher<radar_msgs::msg::RadarTracks>(
    "objects_raw", rclcpp::SensorDataQoS());

  objects_markers_pub_ =
    parent_node->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array", 10);

  diagnostics_pub_ =
    parent_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  radar_info_pub_ = parent_node->create_publisher<autoware_sensing_msgs::msg::RadarInfo>(
    "radar_info", rclcpp::SensorDataQoS());

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);

  // Set up diagnostics
  objects_diagnostics_updater_.add(objects_rate_bound_status_);
  detections_diagnostics_updater_.add(detections_rate_bound_status_);
  liveness_diagnostics_updater_.add(liveness_monitor_);
  objects_diagnostics_updater_.setHardwareID(config_ptr_->frame_id);
  detections_diagnostics_updater_.setHardwareID(config_ptr_->frame_id);
  liveness_diagnostics_updater_.setHardwareID(config_ptr_->frame_id);
  objects_diagnostics_updater_.setPeriod(100ms);
  detections_diagnostics_updater_.setPeriod(100ms);
  liveness_diagnostics_updater_.setPeriod(100ms);
  objects_diagnostics_updater_.force_update();
  detections_diagnostics_updater_.force_update();
  liveness_diagnostics_updater_.force_update();

  create_radar_info();
}

Status ContinentalARS548DecoderWrapper::initialize_driver(
  const std::shared_ptr<
    const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration> & config)
{
  driver_ptr_.reset();
  driver_ptr_ = std::make_shared<drivers::continental_ars548::ContinentalARS548Decoder>(config);

  driver_ptr_->register_detection_list_callback(
    std::bind(
      &ContinentalARS548DecoderWrapper::detection_list_callback, this, std::placeholders::_1));
  driver_ptr_->register_object_list_callback(
    std::bind(&ContinentalARS548DecoderWrapper::object_list_callback, this, std::placeholders::_1));
  driver_ptr_->register_sensor_status_callback(
    std::bind(
      &ContinentalARS548DecoderWrapper::sensor_status_callback, this, std::placeholders::_1));
  driver_ptr_->register_packets_callback(
    std::bind(&ContinentalARS548DecoderWrapper::packets_callback, this, std::placeholders::_1));

  return Status::OK;
}

void ContinentalARS548DecoderWrapper::initialize_sync_diagnostics(rclcpp::Node * const parent_node)
{
  std::scoped_lock lock(mtx_config_ptr_, mtx_driver_ptr_);
  if (!config_ptr_->sync_diagnostics_topic) {
    return;
  }

  // ARS548 does not document any domain ID settings, so the default domain 0 is hardcoded.
  // ARS548 is known working with domain 0.
  sync_tooling_plugin_.emplace(
    SyncToolingPlugin{
      std::make_shared<SyncToolingWorker>(
        parent_node, *config_ptr_->sync_diagnostics_topic, config_ptr_->frame_id, 0),
      util::RateLimiter(100ms)});

  driver_ptr_->register_sync_status_callback(
    [this](uint64_t receive_time_ns, uint64_t packet_time_ns, bool sync_ok) {
      sync_tooling_plugin_->rate_limiter.with_rate_limit(
        receive_time_ns, [this, receive_time_ns, packet_time_ns, sync_ok]() {
          int64_t clock_diff =
            static_cast<int64_t>(receive_time_ns) - static_cast<int64_t>(packet_time_ns);

          sync_tooling_plugin_->worker->submit_self_reported_clock_state(
            sync_ok ? SelfReportedClockStateUpdate::LOCKED
                    : SelfReportedClockStateUpdate::UNSYNCHRONIZED);

          sync_tooling_plugin_->worker->submit_clock_diff_measurement(clock_diff);
        });
    });
}

void ContinentalARS548DecoderWrapper::on_config_change(
  const std::shared_ptr<
    const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration> &
    new_config_ptr)
{
  std::scoped_lock lock(mtx_config_ptr_, mtx_driver_ptr_);
  initialize_driver(new_config_ptr);
  config_ptr_ = new_config_ptr;
}

void ContinentalARS548DecoderWrapper::process_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  std::lock_guard lock(mtx_driver_ptr_);
  driver_ptr_->process_packet(std::move(packet_msg));

  liveness_monitor_.tick();
}

void ContinentalARS548DecoderWrapper::detection_list_callback(
  std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList> msg)
{
  if (
    detection_pointcloud_pub_->get_subscription_count() > 0 ||
    detection_pointcloud_pub_->get_intra_process_subscription_count() > 0) {
    const auto detection_pointcloud_ptr = convert_to_pointcloud(*msg);
    auto detection_pointcloud_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*detection_pointcloud_ptr, *detection_pointcloud_msg_ptr);

    detection_pointcloud_msg_ptr->header = msg->header;
    detection_pointcloud_pub_->publish(std::move(detection_pointcloud_msg_ptr));
  }

  if (
    scan_raw_pub_->get_subscription_count() > 0 ||
    scan_raw_pub_->get_intra_process_subscription_count() > 0) {
    auto radar_scan_msg = convert_to_radar_scan(*msg);
    radar_scan_msg.header = msg->header;
    scan_raw_pub_->publish(std::move(radar_scan_msg));
  }

  if (
    detection_list_pub_->get_subscription_count() > 0 ||
    detection_list_pub_->get_intra_process_subscription_count() > 0) {
    detection_list_pub_->publish(std::move(msg));
  }

  std::shared_lock lock_cfg(mtx_config_ptr_);
  if (
    detection_msgs_counter_ % config_ptr_->radar_info_rate_subsample == 0 &&
    (radar_info_pub_->get_subscription_count() > 0 ||
     radar_info_pub_->get_intra_process_subscription_count() > 0)) {
    radar_info_msg_.header.stamp = msg->header.stamp;
    radar_info_pub_->publish(radar_info_msg_);
  }

  detection_msgs_counter_++;
  detections_rate_bound_status_.tick();
}

void ContinentalARS548DecoderWrapper::object_list_callback(
  std::unique_ptr<continental_msgs::msg::ContinentalArs548ObjectList> msg)
{
  if (
    object_pointcloud_pub_->get_subscription_count() > 0 ||
    object_pointcloud_pub_->get_intra_process_subscription_count() > 0) {
    const auto object_pointcloud_ptr = convert_to_pointcloud(*msg);
    auto object_pointcloud_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*object_pointcloud_ptr, *object_pointcloud_msg_ptr);

    object_pointcloud_msg_ptr->header = msg->header;
    object_pointcloud_pub_->publish(std::move(object_pointcloud_msg_ptr));
  }

  if (
    autoware_objects_pub_->get_subscription_count() > 0 ||
    autoware_objects_pub_->get_intra_process_subscription_count() > 0) {
    auto autoware_objects_msg = convert_to_autoware_radar_objects(*msg);
    autoware_objects_pub_->publish(std::move(autoware_objects_msg));
  }

  if (
    objects_raw_pub_->get_subscription_count() > 0 ||
    objects_raw_pub_->get_intra_process_subscription_count() > 0) {
    auto objects_raw_msg = convert_to_radar_tracks(*msg);
    objects_raw_msg.header = msg->header;
    objects_raw_pub_->publish(std::move(objects_raw_msg));
  }

  if (
    objects_markers_pub_->get_subscription_count() > 0 ||
    objects_markers_pub_->get_intra_process_subscription_count() > 0) {
    auto marker_array_msg = convert_to_markers(*msg);
    objects_markers_pub_->publish(std::move(marker_array_msg));
  }

  if (
    object_list_pub_->get_subscription_count() > 0 ||
    object_list_pub_->get_intra_process_subscription_count() > 0) {
    object_list_pub_->publish(std::move(msg));
  }
  objects_rate_bound_status_.tick();
}

void ContinentalARS548DecoderWrapper::sensor_status_callback(
  const drivers::continental_ars548::ContinentalARS548Status & sensor_status)
{
  // Update rate bounds
  if (sensor_status.cycle_time != latest_config_cycle_time_ms_) {
    latest_config_cycle_time_ms_ = sensor_status.cycle_time;
    objects_diagnostics_updater_.setPeriod(latest_config_cycle_time_ms_ * 1.0e-3);
    detections_diagnostics_updater_.setPeriod(latest_config_cycle_time_ms_ * 1.0e-3);
    liveness_diagnostics_updater_.setPeriod(latest_config_cycle_time_ms_ * 1.0e-3);
    objects_diagnostics_updater_.force_update();
    detections_diagnostics_updater_.force_update();
    liveness_diagnostics_updater_.force_update();
  }

  // Init diagnostic array msg
  diagnostic_msgs::msg::DiagnosticArray diagnostic_array_msg;
  diagnostic_array_msg.header.stamp.sec = static_cast<int32_t>(sensor_status.timestamp_seconds);
  diagnostic_array_msg.header.stamp.nanosec = sensor_status.timestamp_nanoseconds;

  auto add_diagnostic = [](
                          diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key,
                          const std::string & value) {
    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = key;
    key_value.value = value;
    status.values.push_back(key_value);
  };

  // Dynamics status
  diagnostic_msgs::msg::DiagnosticStatus dynamics_diagnostic_status;

  add_diagnostic(
    dynamics_diagnostic_status, "Longitudinal velocity status",
    sensor_status.longitudinal_velocity_status);
  add_diagnostic(
    dynamics_diagnostic_status, "Longitudinal acceleration status",
    sensor_status.longitudinal_acceleration_status);
  add_diagnostic(
    dynamics_diagnostic_status, "Lateral acceleration status",
    sensor_status.lateral_acceleration_status);
  add_diagnostic(dynamics_diagnostic_status, "Yaw rate status", sensor_status.yaw_rate_status);
  add_diagnostic(
    dynamics_diagnostic_status, "Steering angle status", sensor_status.steering_angle_status);
  add_diagnostic(
    dynamics_diagnostic_status, "Driving direction status", sensor_status.driving_direction_status);

  std::shared_lock lock_cfg(mtx_config_ptr_);
  dynamics_diagnostic_status.hardware_id = config_ptr_->frame_id;
  dynamics_diagnostic_status.name =
    std::string(parent_node_->get_fully_qualified_name()) + ": Dynamics";
  dynamics_diagnostic_status.level = sensor_status.dynamics_diagnostics_status;
  dynamics_diagnostic_status.message = "ARS548 dynamics status";

  diagnostic_array_msg.status = {dynamics_diagnostic_status};
  diagnostics_pub_->publish(diagnostic_array_msg);

  // Internal status
  diagnostic_msgs::msg::DiagnosticStatus internal_diagnostic_status;

  internal_status_.update_state(sensor_status.internal_diagnostics_status);
  add_diagnostic(internal_diagnostic_status, "Radar status", sensor_status.radar_status);
  add_diagnostic(internal_diagnostic_status, "Voltage status", sensor_status.voltage_status);
  add_diagnostic(
    internal_diagnostic_status, "Temperature status", sensor_status.temperature_status);
  add_diagnostic(
    internal_diagnostic_status, "Effective internal status",
    custom_diagnostic_tasks::get_level_string(internal_status_.get_current_state_level()));
  add_diagnostic(
    internal_diagnostic_status, "Candidate internal status",
    custom_diagnostic_tasks::get_level_string(internal_status_.get_candidate_level()));
  add_diagnostic(
    internal_diagnostic_status, "Candidate status observed frames",
    std::to_string(internal_status_.get_candidate_num_observation()));
  add_diagnostic(
    internal_diagnostic_status, "Observed frames transition threshold",
    std::to_string(internal_status_.get_num_frame_transition()));

  internal_diagnostic_status.hardware_id = config_ptr_->frame_id;
  internal_diagnostic_status.name =
    std::string(parent_node_->get_fully_qualified_name()) + ": Internal";
  internal_diagnostic_status.level = internal_status_.get_current_state_level();
  internal_diagnostic_status.message = "ARS548 internal signals";

  diagnostic_array_msg.status = {internal_diagnostic_status};
  diagnostics_pub_->publish(diagnostic_array_msg);

  // Blockage status
  diagnostic_msgs::msg::DiagnosticStatus blockage_diagnostic_status;

  blockage_status_.update_state(sensor_status.blockage_diagnostics_status);
  add_diagnostic(blockage_diagnostic_status, "Blockage status", sensor_status.blockage_status);
  add_diagnostic(
    blockage_diagnostic_status, "Effective blockage status",
    custom_diagnostic_tasks::get_level_string(blockage_status_.get_current_state_level()));
  add_diagnostic(
    blockage_diagnostic_status, "Candidate blockage status",
    custom_diagnostic_tasks::get_level_string(blockage_status_.get_candidate_level()));
  add_diagnostic(
    blockage_diagnostic_status, "Candidate status observed frames",
    std::to_string(blockage_status_.get_candidate_num_observation()));
  add_diagnostic(
    blockage_diagnostic_status, "Observed frames transition threshold",
    std::to_string(blockage_status_.get_num_frame_transition()));

  blockage_diagnostic_status.hardware_id = config_ptr_->frame_id;
  blockage_diagnostic_status.name =
    std::string(parent_node_->get_fully_qualified_name()) + ": Blockage";
  blockage_diagnostic_status.level = blockage_status_.get_current_state_level();
  blockage_diagnostic_status.message = "ARS548 blockage status";

  diagnostic_array_msg.status = {blockage_diagnostic_status};
  diagnostics_pub_->publish(diagnostic_array_msg);

  // Configuration status
  diagnostic_msgs::msg::DiagnosticStatus configuration_status;

  add_diagnostic(
    configuration_status, "Sensor timestamp",
    util::format_timestamp(sensor_status.timestamp_seconds, sensor_status.timestamp_nanoseconds));
  add_diagnostic(
    configuration_status, "Sensor timestamp sync status", sensor_status.timestamp_sync_status);
  add_diagnostic(
    configuration_status, "Firmware version",
    drivers::continental_ars548::extract_firmware_version(
      sensor_status.sw_version_major, sensor_status.sw_version_minor,
      sensor_status.sw_version_patch));
  add_diagnostic(
    configuration_status, "Longitudinal sensor position",
    std::to_string(sensor_status.longitudinal));
  add_diagnostic(
    configuration_status, "Lateral sensor position", std::to_string(sensor_status.lateral));
  add_diagnostic(
    configuration_status, "Vertical sensor position", std::to_string(sensor_status.vertical));
  add_diagnostic(configuration_status, "Yaw sensor angle", std::to_string(sensor_status.yaw));
  add_diagnostic(configuration_status, "Pitch sensor angle", std::to_string(sensor_status.pitch));
  add_diagnostic(configuration_status, "Plug orientation", sensor_status.plug_orientation);
  add_diagnostic(configuration_status, "Vehicle length", std::to_string(sensor_status.length));
  add_diagnostic(configuration_status, "Vehicle width", std::to_string(sensor_status.width));
  add_diagnostic(configuration_status, "Vehicle height", std::to_string(sensor_status.height));
  add_diagnostic(
    configuration_status, "Vehicle wheelbase", std::to_string(sensor_status.wheel_base));
  add_diagnostic(
    configuration_status, "Maximum detection distance", std::to_string(sensor_status.max_distance));
  add_diagnostic(configuration_status, "Frequency slot", sensor_status.frequency_slot);
  add_diagnostic(configuration_status, "Cycle time", std::to_string(sensor_status.cycle_time));
  add_diagnostic(configuration_status, "Time slot", std::to_string(sensor_status.time_slot));
  add_diagnostic(configuration_status, "HCC", sensor_status.hcc);
  add_diagnostic(
    configuration_status, "Power save standstill", sensor_status.power_save_standstill);
  add_diagnostic(configuration_status, "Sensor IP address", sensor_status.sensor_ip_address0);
  configuration_status.hardware_id = config_ptr_->frame_id;
  configuration_status.name =
    std::string(parent_node_->get_fully_qualified_name()) + ": Configuration";
  configuration_status.message = "ARS548 configuration status";
  configuration_status.level = sensor_status.configuration_diagnostics_status;

  diagnostic_array_msg.status = {configuration_status};
  diagnostics_pub_->publish(diagnostic_array_msg);
}

void ContinentalARS548DecoderWrapper::packets_callback(
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> msg)
{
  if (
    packets_pub_ && (packets_pub_->get_subscription_count() > 0 ||
                     packets_pub_->get_intra_process_subscription_count() > 0)) {
    packets_pub_->publish(std::move(msg));
  }
}

void ContinentalARS548DecoderWrapper::create_radar_info()
{
  namespace ars548 = nebula::drivers::continental_ars548;
  std::shared_lock lock_cfg(mtx_config_ptr_);
  radar_info_msg_.header.frame_id = config_ptr_->frame_id;

  auto make_field_info = [](
                           const std::string & field_name, const ars548::FieldInfo & field_info,
                           std::vector<autoware_sensing_msgs::msg::RadarFieldInfo> & fields_msg) {
    autoware_sensing_msgs::msg::RadarFieldInfo field;
    field.field_name.data = field_name;

    if (field_info.min_value) {
      field.min_value_available = true;
      field.min_value = field_info.min_value.value();
    } else {
      field.min_value_available = false;
      field.min_value = std::numeric_limits<float>::lowest();
    }

    if (field_info.max_value) {
      field.max_value_available = true;
      field.max_value = field_info.max_value.value();
    } else {
      field.max_value_available = false;
      field.max_value = std::numeric_limits<float>::max();
    }

    if (field_info.resolution) {
      field.resolution_available = true;
      field.resolution = field_info.resolution.value();
    } else {
      field.resolution_available = false;
      field.resolution = 0.f;
    }

    fields_msg.push_back(field);
  };

  // Detection field infos
  make_field_info("azimuth", ars548::azimuth_info, radar_info_msg_.detection_fields_info);
  make_field_info("azimuth_std", ars548::azimuth_std_info, radar_info_msg_.detection_fields_info);
  make_field_info("elevation", ars548::elevation_info, radar_info_msg_.detection_fields_info);
  make_field_info(
    "elevation_std", ars548::elevation_std_info, radar_info_msg_.detection_fields_info);
  make_field_info("range", ars548::range_info, radar_info_msg_.detection_fields_info);
  make_field_info("range_std", ars548::range_std_info, radar_info_msg_.detection_fields_info);
  make_field_info("range_rate", ars548::range_rate_info, radar_info_msg_.detection_fields_info);
  make_field_info(
    "range_rate_std", ars548::range_rate_std_info, radar_info_msg_.detection_fields_info);
  make_field_info("rcs", ars548::rcs_info, radar_info_msg_.detection_fields_info);
  make_field_info(
    "measurement_id", ars548::measurement_id_info, radar_info_msg_.detection_fields_info);
  make_field_info(
    "positive_predictive_value", ars548::positive_predictive_value_info,
    radar_info_msg_.detection_fields_info);
  make_field_info(
    "classification", ars548::classification_info, radar_info_msg_.detection_fields_info);
  make_field_info(
    "multi_target_probability", ars548::multi_target_probability_info,
    radar_info_msg_.detection_fields_info);
  make_field_info("object_id", ars548::object_id_info, radar_info_msg_.detection_fields_info);
  make_field_info(
    "ambiguity_flag", ars548::ambiguity_flag_info, radar_info_msg_.detection_fields_info);

  // Object field infos
  make_field_info("object_id", ars548::object_id_info, radar_info_msg_.object_fields_info);
  make_field_info("age", ars548::age_info, radar_info_msg_.object_fields_info);
  make_field_info(
    "measurement_status", ars548::measurement_status_info, radar_info_msg_.object_fields_info);
  make_field_info(
    "movement_status", ars548::movement_status_info, radar_info_msg_.object_fields_info);
  make_field_info("position_x", ars548::position_x_info, radar_info_msg_.object_fields_info);
  make_field_info("position_y", ars548::position_y_info, radar_info_msg_.object_fields_info);
  make_field_info("position_z", ars548::position_z_info, radar_info_msg_.object_fields_info);
  make_field_info("velocity_x", ars548::velocity_x_info, radar_info_msg_.object_fields_info);
  make_field_info("velocity_y", ars548::velocity_y_info, radar_info_msg_.object_fields_info);
  make_field_info(
    "acceleration_x", ars548::acceleration_x_info, radar_info_msg_.object_fields_info);
  make_field_info(
    "acceleration_y", ars548::acceleration_y_info, radar_info_msg_.object_fields_info);
  make_field_info("size_x", ars548::size_x_info, radar_info_msg_.object_fields_info);
  make_field_info("size_y", ars548::size_y_info, radar_info_msg_.object_fields_info);
  make_field_info("orientation", ars548::orientation_info, radar_info_msg_.object_fields_info);
  make_field_info(
    "orientation_std", ars548::orientation_std_info, radar_info_msg_.object_fields_info);
  make_field_info(
    "orientation_rate", ars548::orientation_rate_info, radar_info_msg_.object_fields_info);
  make_field_info(
    "orientation_rate_std", ars548::orientation_rate_std_info, radar_info_msg_.object_fields_info);
  make_field_info(
    "existence_probability", ars548::existence_probability_info,
    radar_info_msg_.object_fields_info);

  radar_info_msg_.available_classes = {
    autoware_sensing_msgs::msg::RadarClassification::UNKNOWN,
    autoware_sensing_msgs::msg::RadarClassification::CAR,
    autoware_sensing_msgs::msg::RadarClassification::TRUCK,
    autoware_sensing_msgs::msg::RadarClassification::MOTORCYCLE,
    autoware_sensing_msgs::msg::RadarClassification::BICYCLE,
    autoware_sensing_msgs::msg::RadarClassification::PEDESTRIAN,
    autoware_sensing_msgs::msg::RadarClassification::ANIMAL,
    autoware_sensing_msgs::msg::RadarClassification::HAZARD};

  radar_info_msg_.absolute_dynamics = true;
}

geometry_msgs::msg::Point ContinentalARS548DecoderWrapper::reference_point_to_center(
  const geometry_msgs::msg::Point & reference_point, double yaw, double length, double width,
  int reference_index)
{
  const double half_length = 0.5 * length;
  const double half_width = 0.5 * width;

  geometry_msgs::msg::Point center;
  center.x = reference_point.x +
             std::cos(yaw) * half_length * reference_to_center_[reference_index][0] -
             std::sin(yaw) * half_width * reference_to_center_[reference_index][1];
  center.y = reference_point.y +
             std::sin(yaw) * half_length * reference_to_center_[reference_index][0] +
             std::cos(yaw) * half_width * reference_to_center_[reference_index][1];
  center.z = reference_point.z;

  return center;
}

autoware_sensing_msgs::msg::RadarObjects
ContinentalARS548DecoderWrapper::convert_to_autoware_radar_objects(
  const continental_msgs::msg::ContinentalArs548ObjectList & msg)
{
  namespace ars548 = nebula::drivers::continental_ars548;
  using autoware_sensing_msgs::msg::RadarClassification;
  using autoware_sensing_msgs::msg::RadarObject;
  using autoware_sensing_msgs::msg::RadarObjects;

  RadarObjects autoware_objects;
  autoware_objects.header = msg.header;
  autoware_objects.objects.reserve(msg.objects.size());

  for (const auto & continental_object : msg.objects) {
    RadarObject autoware_object;
    autoware_object.object_id = continental_object.object_id;
    autoware_object.age = continental_object.age;
    autoware_object.measurement_status = continental_object.status_measurement;
    autoware_object.movement_status = continental_object.status_movement;

    switch (continental_object.status_measurement) {
      case ars548::measurement_status_measured:
        autoware_object.measurement_status = RadarObject::MEASUREMENT_STATUS_MEASURED;
        break;
      case ars548::measurement_status_predicted:
        autoware_object.measurement_status = RadarObject::MEASUREMENT_STATUS_PREDICTED;
        break;
      case ars548::measurement_status_new:
        autoware_object.measurement_status = RadarObject::MEASUREMENT_STATUS_NEW;
        break;
      case ars548::measurement_status_invalid:
        autoware_object.measurement_status = RadarObject::MEASUREMENT_STATUS_INVALID;
        break;
      default:
        autoware_object.measurement_status = RadarObject::MEASUREMENT_STATUS_UNKNOWN;
        break;
    }

    switch (continental_object.status_movement) {
      case ars548::movement_status_dynamic:
        autoware_object.movement_status = RadarObject::MOVEMENT_STATUS_DYNAMIC;
        break;
      case ars548::movement_status_static:
        autoware_object.movement_status = RadarObject::MOVEMENT_STATUS_STATIC;
        break;
      case ars548::movement_status_invalid:
        autoware_object.movement_status = RadarObject::MOVEMENT_STATUS_INVALID;
        break;
      default:
        autoware_object.movement_status = RadarObject::MOVEMENT_STATUS_UNKNOWN;
        break;
    }

    autoware_object.orientation = continental_object.orientation;
    autoware_object.orientation_std = continental_object.orientation_std;
    autoware_object.orientation_rate = continental_object.orientation_rate_mean;
    autoware_object.orientation_rate_std = continental_object.orientation_rate_std;
    autoware_object.existence_probability =
      ars548::normalize_probability(continental_object.raw_existence_probability);

    // Position
    // There are 9 possible reference points. In the case of an invalid reference point, we fall
    // back to the center
    const int reference_index = std::min<int>(continental_object.position_reference, 8);
    const double & yaw = continental_object.orientation;
    autoware_object.position = reference_point_to_center(
      continental_object.position, yaw, continental_object.shape_length_edge_mean,
      continental_object.shape_width_edge_mean, reference_index);

    autoware_object.velocity = continental_object.absolute_velocity;
    autoware_object.acceleration = continental_object.absolute_acceleration;
    autoware_object.size.x = continental_object.shape_length_edge_mean;
    autoware_object.size.y = continental_object.shape_width_edge_mean;
    autoware_object.size.z = 1.f;

    RadarClassification classification;
    autoware_object.classifications.reserve(10);

    classification.label = RadarClassification::UNKNOWN;
    classification.probability =
      ars548::normalize_probability(continental_object.raw_classification_unknown);
    autoware_object.classifications.push_back(classification);

    classification.label = RadarClassification::CAR;
    classification.probability =
      ars548::normalize_probability(continental_object.raw_classification_car);
    autoware_object.classifications.push_back(classification);

    classification.label = RadarClassification::TRUCK;
    classification.probability =
      ars548::normalize_probability(continental_object.raw_classification_truck);
    autoware_object.classifications.push_back(classification);

    classification.label = RadarClassification::MOTORCYCLE;
    classification.probability =
      ars548::normalize_probability(continental_object.raw_classification_motorcycle);
    autoware_object.classifications.push_back(classification);

    classification.label = RadarClassification::BICYCLE;
    classification.probability =
      ars548::normalize_probability(continental_object.raw_classification_bicycle);
    autoware_object.classifications.push_back(classification);

    classification.label = RadarClassification::PEDESTRIAN;
    classification.probability =
      ars548::normalize_probability(continental_object.raw_classification_pedestrian);
    autoware_object.classifications.push_back(classification);

    classification.label = RadarClassification::ANIMAL;
    classification.probability =
      ars548::normalize_probability(continental_object.raw_classification_animal);
    autoware_object.classifications.push_back(classification);

    classification.label = RadarClassification::HAZARD;
    classification.probability =
      ars548::normalize_probability(continental_object.raw_classification_hazard);
    autoware_object.classifications.push_back(classification);

    auto fill_cov_matrix =
      [](const double & x, const double & y, const double & xy, std::array<float, 6> & cov_matrix) {
        cov_matrix[0] = static_cast<float>(x * x);
        cov_matrix[1] = static_cast<float>(xy);
        cov_matrix[2] = RadarObject::INVALID_COV_VALUE;
        cov_matrix[3] = static_cast<float>(y * y);
        cov_matrix[4] = RadarObject::INVALID_COV_VALUE;
        cov_matrix[5] = RadarObject::INVALID_COV_VALUE;
      };

    fill_cov_matrix(
      continental_object.position_std.x, continental_object.position_std.y,
      continental_object.position_covariance_xy, autoware_object.position_covariance);

    fill_cov_matrix(
      continental_object.absolute_velocity_std.x, continental_object.absolute_velocity_std.y,
      continental_object.absolute_velocity_covariance_xy, autoware_object.velocity_covariance);

    fill_cov_matrix(
      continental_object.absolute_acceleration_std.x,
      continental_object.absolute_acceleration_std.y,
      continental_object.absolute_acceleration_covariance_xy,
      autoware_object.acceleration_covariance);

    std::fill(
      autoware_object.size_covariance.begin(), autoware_object.size_covariance.end(),
      RadarObject::INVALID_COV_VALUE);

    autoware_objects.objects.push_back(autoware_object);
  }

  return autoware_objects;
}

pcl::PointCloud<nebula::drivers::continental_ars548::PointARS548Detection>::Ptr
ContinentalARS548DecoderWrapper::convert_to_pointcloud(
  const continental_msgs::msg::ContinentalArs548DetectionList & msg)
{
  namespace ars548 = nebula::drivers::continental_ars548;

  pcl::PointCloud<nebula::drivers::continental_ars548::PointARS548Detection>::Ptr output_pointcloud(
    new pcl::PointCloud<nebula::drivers::continental_ars548::PointARS548Detection>);
  output_pointcloud->reserve(msg.detections.size());

  nebula::drivers::continental_ars548::PointARS548Detection point{};
  for (const auto & detection : msg.detections) {
    point.x =
      std::cos(detection.elevation_angle) * std::cos(detection.azimuth_angle) * detection.range;
    point.y =
      std::cos(detection.elevation_angle) * std::sin(detection.azimuth_angle) * detection.range;
    point.z = std::sin(detection.elevation_angle) * detection.range;

    point.azimuth = detection.azimuth_angle;
    point.azimuth_std = detection.azimuth_angle_std;
    point.elevation = detection.elevation_angle;
    point.elevation_std = detection.elevation_angle_std;
    point.range = detection.range;
    point.range_std = detection.range_std;
    point.range_rate = detection.range_rate;
    point.range_rate_std = detection.range_rate_std;
    point.rcs = detection.rcs;
    point.measurement_id = detection.measurement_id;
    point.positive_predictive_value =
      ars548::normalize_probability(detection.raw_positive_predictive_value);
    point.classification = detection.classification;
    point.multi_target_probability =
      ars548::normalize_probability(detection.raw_multi_target_probability);
    point.object_id = detection.object_id;
    point.ambiguity_flag = ars548::normalize_probability(detection.raw_ambiguity_flag);

    output_pointcloud->points.emplace_back(point);
  }

  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<nebula::drivers::continental_ars548::PointARS548Object>::Ptr
ContinentalARS548DecoderWrapper::convert_to_pointcloud(
  const continental_msgs::msg::ContinentalArs548ObjectList & msg)
{
  namespace ars548 = nebula::drivers::continental_ars548;

  pcl::PointCloud<nebula::drivers::continental_ars548::PointARS548Object>::Ptr output_pointcloud(
    new pcl::PointCloud<nebula::drivers::continental_ars548::PointARS548Object>);
  output_pointcloud->reserve(msg.objects.size());

  nebula::drivers::continental_ars548::PointARS548Object point{};
  for (const auto & object : msg.objects) {
    point.x = static_cast<float>(object.position.x);
    point.y = static_cast<float>(object.position.y);
    point.z = static_cast<float>(object.position.z);

    point.id = object.object_id;
    point.age = object.age;
    point.status_measurement = object.status_measurement;
    point.status_movement = object.status_movement;
    point.position_reference = object.position_reference;
    point.classification_car = ars548::normalize_probability(object.raw_classification_car);
    point.classification_truck = ars548::normalize_probability(object.raw_classification_truck);
    point.classification_motorcycle =
      ars548::normalize_probability(object.raw_classification_motorcycle);
    point.classification_bicycle = ars548::normalize_probability(object.raw_classification_bicycle);
    point.classification_pedestrian =
      ars548::normalize_probability(object.raw_classification_pedestrian);
    point.dynamics_abs_vel_x = static_cast<float>(object.absolute_velocity.x);
    point.dynamics_abs_vel_y = static_cast<float>(object.absolute_velocity.y);
    point.dynamics_rel_vel_x = static_cast<float>(object.relative_velocity.x);
    point.dynamics_rel_vel_y = static_cast<float>(object.relative_velocity.y);
    point.shape_length_edge_mean = object.shape_length_edge_mean;
    point.shape_width_edge_mean = object.shape_width_edge_mean;
    point.dynamics_orientation_rate_mean = object.orientation_rate_mean;

    output_pointcloud->points.emplace_back(point);
  }

  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

radar_msgs::msg::RadarScan ContinentalARS548DecoderWrapper::convert_to_radar_scan(
  const continental_msgs::msg::ContinentalArs548DetectionList & msg)
{
  radar_msgs::msg::RadarScan output_msg;
  output_msg.header = msg.header;
  output_msg.returns.reserve(msg.detections.size());

  radar_msgs::msg::RadarReturn return_msg;
  for (const auto & detection : msg.detections) {
    if (
      detection.invalid_azimuth || detection.invalid_distance || detection.invalid_elevation ||
      detection.invalid_range_rate) {
      continue;
    }

    return_msg.range = detection.range;
    return_msg.azimuth = detection.azimuth_angle;
    return_msg.elevation = detection.elevation_angle;
    return_msg.doppler_velocity = detection.range_rate;
    return_msg.amplitude = detection.rcs;
    output_msg.returns.emplace_back(return_msg);
  }

  return output_msg;
}

radar_msgs::msg::RadarTracks ContinentalARS548DecoderWrapper::convert_to_radar_tracks(
  const continental_msgs::msg::ContinentalArs548ObjectList & msg)
{
  radar_msgs::msg::RadarTracks output_msg;
  output_msg.tracks.reserve(msg.objects.size());
  output_msg.header = msg.header;

  constexpr int16_t unknown_id = 32000;
  constexpr int16_t car_id = 32001;
  constexpr int16_t truck_id = 32002;
  constexpr int16_t motorcycle_id = 32005;
  constexpr int16_t bicycle_id = 32006;
  constexpr int16_t pedestrian_id = 32007;
  constexpr float invalid_covariance = 1e6;

  radar_msgs::msg::RadarTrack track_msg;
  for (const auto & object : msg.objects) {
    track_msg.uuid.uuid[0] = static_cast<uint8_t>(object.object_id & 0xff);
    track_msg.uuid.uuid[1] = static_cast<uint8_t>((object.object_id >> 8) & 0xff);
    track_msg.uuid.uuid[2] = static_cast<uint8_t>((object.object_id >> 16) & 0xff);
    track_msg.uuid.uuid[3] = static_cast<uint8_t>((object.object_id >> 24) & 0xff);

    // There are 9 possible reference points. In the case of an invalid reference point, we fall
    // back to the center
    const int reference_index = std::min<int>(object.position_reference, 8);
    const double & yaw = object.orientation;
    track_msg.position = reference_point_to_center(
      object.position, yaw, object.shape_length_edge_mean, object.shape_width_edge_mean,
      reference_index);

    track_msg.velocity = object.absolute_velocity;
    track_msg.acceleration = object.absolute_acceleration;
    track_msg.size.x = object.shape_length_edge_mean;
    track_msg.size.y = object.shape_width_edge_mean;
    track_msg.size.z = 1.f;

    uint8_t max_score = object.raw_classification_unknown;
    track_msg.classification = unknown_id;

    if (object.raw_classification_car > max_score) {
      max_score = object.raw_classification_car;
      track_msg.classification = car_id;
    }
    if (object.raw_classification_truck > max_score) {
      max_score = object.raw_classification_truck;
      track_msg.classification = truck_id;
    }
    if (object.raw_classification_motorcycle > max_score) {
      max_score = object.raw_classification_motorcycle;
      track_msg.classification = motorcycle_id;
    }
    if (object.raw_classification_bicycle > max_score) {
      max_score = object.raw_classification_bicycle;
      track_msg.classification = bicycle_id;
    }
    if (object.raw_classification_pedestrian > max_score) {
      max_score = object.raw_classification_pedestrian;
      track_msg.classification = pedestrian_id;
    }

    track_msg.position_covariance[0] =
      static_cast<float>(object.position_std.x * object.position_std.x);
    track_msg.position_covariance[1] = object.position_covariance_xy;
    track_msg.position_covariance[2] = 0.f;
    track_msg.position_covariance[3] =
      static_cast<float>(object.position_std.y * object.position_std.y);
    track_msg.position_covariance[4] = 0.f;
    track_msg.position_covariance[5] =
      static_cast<float>(object.position_std.z * object.position_std.z);

    track_msg.velocity_covariance[0] =
      static_cast<float>(object.absolute_velocity_std.x * object.absolute_velocity_std.x);
    track_msg.velocity_covariance[1] = object.absolute_velocity_covariance_xy;
    track_msg.velocity_covariance[2] = 0.f;
    track_msg.velocity_covariance[3] =
      static_cast<float>(object.absolute_velocity_std.y * object.absolute_velocity_std.y);
    track_msg.velocity_covariance[4] = 0.f;
    track_msg.velocity_covariance[5] =
      static_cast<float>(object.absolute_velocity_std.z * object.absolute_velocity_std.z);

    track_msg.acceleration_covariance[0] =
      static_cast<float>(object.absolute_acceleration_std.x * object.absolute_acceleration_std.x);
    track_msg.acceleration_covariance[1] = object.absolute_acceleration_covariance_xy;
    track_msg.acceleration_covariance[2] = 0.f;
    track_msg.acceleration_covariance[3] =
      static_cast<float>(object.absolute_acceleration_std.y * object.absolute_acceleration_std.y);
    track_msg.acceleration_covariance[4] = 0.f;
    track_msg.acceleration_covariance[5] =
      static_cast<float>(object.absolute_acceleration_std.z * object.absolute_acceleration_std.z);

    track_msg.size_covariance[0] = invalid_covariance;
    track_msg.size_covariance[1] = 0.f;
    track_msg.size_covariance[2] = 0.f;
    track_msg.size_covariance[3] = invalid_covariance;
    track_msg.size_covariance[4] = 0.f;
    track_msg.size_covariance[5] = invalid_covariance;

    output_msg.tracks.emplace_back(track_msg);
  }

  return output_msg;
}

visualization_msgs::msg::MarkerArray ContinentalARS548DecoderWrapper::convert_to_markers(
  const continental_msgs::msg::ContinentalArs548ObjectList & msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.reserve(4 * msg.objects.size());

  constexpr int line_strip_corners_num = 17;
  constexpr std::array<std::array<double, 3>, line_strip_corners_num> cube_corners = {
    {{{-1.0, -1.0, -1.0}},
     {{-1.0, -1.0, 1.0}},
     {{-1.0, 1.0, 1.0}},
     {{-1.0, 1.0, -1.0}},
     {{-1.0, -1.0, -1.0}},
     {{1.0, -1.0, -1.0}},
     {{1.0, -1.0, 1.0}},
     {{1.0, 1.0, 1.0}},
     {{1.0, 1.0, -1.0}},
     {{1.0, -1.0, -1.0}},
     {{-1.0, -1.0, -1.0}},
     {{-1.0, -1.0, 1.0}},
     {{1.0, -1.0, 1.0}},
     {{1.0, 1.0, 1.0}},
     {{-1.0, 1.0, 1.0}},
     {{-1.0, 1.0, -1.0}},
     {{1.0, 1.0, -1.0}}}};

  constexpr int palette_size = 32;
  constexpr std::array<std::array<double, 3>, palette_size> color_array = {{
    {{1.0, 0.0, 0.0}},       {{0.0, 1.0, 0.0}},
    {{0.0, 0.0, 1.0}},  // Red, Green, Blue
    {{1.0, 1.0, 0.0}},       {{0.0, 1.0, 1.0}},
    {{1.0, 0.0, 1.0}},  // Yellow, Cyan, Magenta
    {{1.0, 0.647, 0.0}},     {{0.749, 1.0, 0.0}},
    {{0.0, 0.502, 0.502}},  // Orange, Lime, Teal
    {{0.502, 0.0, 0.502}},   {{1.0, 0.753, 0.796}},
    {{0.647, 0.165, 0.165}},  // Purple, Pink, Brown
    {{0.502, 0.0, 0.0}},     {{0.502, 0.502, 0.0}},
    {{0.0, 0.0, 0.502}},  // Maroon, Olive, Navy
    {{0.502, 0.502, 0.502}}, {{1.0, 0.4, 0.4}},
    {{0.4, 1.0, 0.4}},  // Grey, Light Red, Light Green
    {{0.4, 0.4, 1.0}},       {{1.0, 1.0, 0.4}},
    {{0.4, 1.0, 1.0}},  // Light Blue, Light Yellow, Light Cyan
    {{1.0, 0.4, 1.0}},       {{1.0, 0.698, 0.4}},
    {{0.698, 0.4, 1.0}},  // Light Magenta, Light Orange, Light Purple
    {{1.0, 0.6, 0.8}},       {{0.71, 0.396, 0.114}},
    {{0.545, 0.0, 0.0}},  // Light Pink, Light Brown, Dark Red
    {{0.0, 0.392, 0.0}},     {{0.0, 0.0, 0.545}},
    {{0.545, 0.545, 0.0}},                          // Dark Green, Dark Blue, Dark Yellow
    {{0.0, 0.545, 0.545}},   {{0.545, 0.0, 0.545}}  // Dark Cyan, Dark Magenta
  }};

  std::unordered_set<int> current_ids;
  std::shared_lock lock_cfg(mtx_config_ptr_);

  radar_msgs::msg::RadarTrack track_msg;
  for (const auto & object : msg.objects) {
    const double half_length = 0.5 * object.shape_length_edge_mean;
    const double half_width = 0.5 * object.shape_width_edge_mean;
    constexpr double default_half_size = 1.0;
    const int reference_index = std::min<int>(object.position_reference, 8);
    const double & yaw = object.orientation;
    current_ids.emplace(object.object_id);

    visualization_msgs::msg::Marker box_marker;
    box_marker.header.frame_id = config_ptr_->object_frame;
    box_marker.header.stamp = msg.header.stamp;
    box_marker.ns = "boxes";
    box_marker.id = object.object_id;
    box_marker.action = visualization_msgs::msg::Marker::ADD;
    box_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    box_marker.lifetime = rclcpp::Duration::from_seconds(0);
    box_marker.color.r = color_array[object.object_id % palette_size][0];
    box_marker.color.g = color_array[object.object_id % palette_size][1];
    box_marker.color.b = color_array[object.object_id % palette_size][2];
    box_marker.color.a = 1.0;
    box_marker.scale.x = 0.1;

    box_marker.pose.position = reference_point_to_center(
      object.position, yaw, object.shape_length_edge_mean, object.shape_width_edge_mean,
      reference_index);
    box_marker.pose.orientation.w = std::cos(0.5 * yaw);
    box_marker.pose.orientation.z = std::sin(0.5 * yaw);

    for (const auto & corner : cube_corners) {
      geometry_msgs::msg::Point p;
      p.x = half_length * corner[0];
      p.y = half_width * corner[1];
      p.z = default_half_size * corner[2];
      box_marker.points.emplace_back(p);
    }

    marker_array.markers.emplace_back(box_marker);

    visualization_msgs::msg::Marker text_marker = box_marker;
    text_marker.ns = "object_age";
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.scale.x = 0.3;
    text_marker.scale.y = 0.3;
    text_marker.scale.z = 0.3;
    text_marker.pose.position.z += 0.5;
    text_marker.text =
      "ID=" + std::to_string(object.object_id) + " Age=" + std::to_string(object.age) + "ms";

    marker_array.markers.emplace_back(text_marker);

    std::stringstream object_status_ss;
    object_status_ss << std::fixed << std::setprecision(3) << "ID=" << object.object_id << "\n"
                     << static_cast<int>(object.status_measurement) << "/"
                     << static_cast<int>(object.status_movement) << "/"
                     << static_cast<int>(object.position_reference);

    text_marker.ns = "object_status";
    text_marker.text = object_status_ss.str();

    marker_array.markers.emplace_back(text_marker);

    std::stringstream object_dynamics_ss;
    object_dynamics_ss << std::fixed << std::setprecision(3) << "ID=" << object.object_id
                       << "\nyaw=" << object.orientation
                       << "\nyaw_rate=" << object.orientation_rate_mean
                       << "\nvx=" << object.absolute_velocity.x
                       << "\nvy=" << object.absolute_velocity.y
                       << "\nax=" << object.absolute_acceleration.x
                       << "\nay=" << object.absolute_acceleration.y;

    text_marker.ns = "object_dynamics";
    text_marker.text = object_dynamics_ss.str();

    marker_array.markers.emplace_back(text_marker);
  }

  for (const auto & previous_id : previous_ids_) {
    if (current_ids.find(previous_id) != current_ids.end()) {
      continue;
    }

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = config_ptr_->object_frame;
    delete_marker.header.stamp = msg.header.stamp;
    delete_marker.ns = "boxes";
    delete_marker.id = previous_id;
    delete_marker.action = visualization_msgs::msg::Marker::DELETE;

    marker_array.markers.push_back(delete_marker);

    delete_marker.ns = "object_age";
    marker_array.markers.push_back(delete_marker);

    delete_marker.ns = "object_status";
    marker_array.markers.push_back(delete_marker);

    delete_marker.ns = "object_dynamics";
    marker_array.markers.push_back(delete_marker);
  }

  previous_ids_.clear();
  previous_ids_ = current_ids;

  return marker_array;
}

nebula::Status ContinentalARS548DecoderWrapper::status()
{
  std::lock_guard lock(mtx_driver_ptr_);

  if (!driver_ptr_) {
    return nebula::Status::NOT_INITIALIZED;
  }

  return driver_ptr_->get_status();
}
}  // namespace nebula::ros
