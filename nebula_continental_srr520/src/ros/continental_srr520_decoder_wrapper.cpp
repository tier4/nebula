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

#include "nebula_ros/continental/continental_srr520_decoder_wrapper.hpp"

#include <nebula_common/util/string_conversions.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <unordered_set>
#include <utility>

namespace nebula::ros
{
ContinentalSRR520DecoderWrapper::ContinentalSRR520DecoderWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration> &
    config,
  std::shared_ptr<drivers::continental_srr520::ContinentalSRR520HwInterface> hw_interface_ptr)
: parent_node_(parent_node),
  status_(nebula::Status::NOT_INITIALIZED),
  logger_(parent_node->get_logger().get_child("ContinentalSRR520Decoder")),
  sensor_cfg_(config),
  hw_interface_ptr_(hw_interface_ptr)
{
  using std::chrono_literals::operator""us;
  if (!config) {
    throw std::runtime_error(
      "ContinentalSRR520DecoderWrapper cannot be instantiated without a valid config!");
  }

  RCLCPP_INFO(logger_, "Starting Decoder");

  initialize_driver(config);
  status_ = driver_ptr_->get_status();

  if (Status::OK != status_) {
    throw std::runtime_error("Error instantiating decoder: " + util::to_string(status_));
  }

  // Publish packets only if HW interface is connected
  if (hw_interface_ptr_) {
    packets_pub_ = parent_node->create_publisher<nebula_msgs::msg::NebulaPackets>(
      "nebula_packets", rclcpp::SensorDataQoS());
  }

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  near_detection_list_pub_ =
    parent_node->create_publisher<continental_msgs::msg::ContinentalSrr520DetectionList>(
      "near_continental_detections", pointcloud_qos);
  hrr_detection_list_pub_ =
    parent_node->create_publisher<continental_msgs::msg::ContinentalSrr520DetectionList>(
      "hrr_continental_detections", pointcloud_qos);
  object_list_pub_ =
    parent_node->create_publisher<continental_msgs::msg::ContinentalSrr520ObjectList>(
      "continental_objects", pointcloud_qos);
  status_pub_ =
    parent_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  near_detection_pointcloud_pub_ = parent_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "near_detection_points", pointcloud_qos);
  hrr_detection_pointcloud_pub_ = parent_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "hrr_detection_points", pointcloud_qos);
  object_pointcloud_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("object_points", pointcloud_qos);

  near_scan_raw_pub_ =
    parent_node->create_publisher<radar_msgs::msg::RadarScan>("near_scan_raw", pointcloud_qos);
  hrr_scan_raw_pub_ =
    parent_node->create_publisher<radar_msgs::msg::RadarScan>("hrr_scan_raw", pointcloud_qos);

  objects_raw_pub_ =
    parent_node->create_publisher<radar_msgs::msg::RadarTracks>("objects_raw", pointcloud_qos);

  objects_markers_pub_ =
    parent_node->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array", 10);

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);

  watchdog_ =
    std::make_shared<WatchdogTimer>(*parent_node, 100'000us, [this, parent_node](bool ok) {
      if (ok) return;
      RCLCPP_WARN_THROTTLE(
        logger_, *parent_node->get_clock(), 5000, "Missed sensor output deadline");
    });
}

Status ContinentalSRR520DecoderWrapper::initialize_driver(
  const std::shared_ptr<
    const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration> & config)
{
  driver_ptr_.reset();
  driver_ptr_ = std::make_shared<drivers::continental_srr520::ContinentalSRR520Decoder>(config);

  driver_ptr_->register_near_detection_list_callback(
    std::bind(
      &ContinentalSRR520DecoderWrapper::near_detection_list_callback, this, std::placeholders::_1));
  driver_ptr_->register_hrr_detection_list_callback(
    std::bind(
      &ContinentalSRR520DecoderWrapper::hrr_detection_list_callback, this, std::placeholders::_1));
  driver_ptr_->register_object_list_callback(
    std::bind(&ContinentalSRR520DecoderWrapper::object_list_callback, this, std::placeholders::_1));
  driver_ptr_->register_status_callback(
    std::bind(&ContinentalSRR520DecoderWrapper::status_callback, this, std::placeholders::_1));

  if (hw_interface_ptr_) {
    driver_ptr_->register_sync_follow_up_callback(
      std::bind(
        &ContinentalSRR520DecoderWrapper::sync_follow_up_callback, this, std::placeholders::_1));
    driver_ptr_->register_packets_callback(
      std::bind(&ContinentalSRR520DecoderWrapper::packets_callback, this, std::placeholders::_1));
  }

  driver_ptr_->set_logger(
    std::make_shared<rclcpp::Logger>(parent_node_->get_logger().get_child("Driver")));

  return Status::OK;
}

void ContinentalSRR520DecoderWrapper::on_config_change(
  const std::shared_ptr<
    const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  initialize_driver(new_config);
  sensor_cfg_ = new_config;
}

void ContinentalSRR520DecoderWrapper::process_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  driver_ptr_->process_packet(std::move(packet_msg));

  watchdog_->update();
}

void ContinentalSRR520DecoderWrapper::near_detection_list_callback(
  std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg)
{
  if (
    near_detection_pointcloud_pub_->get_subscription_count() > 0 ||
    near_detection_pointcloud_pub_->get_intra_process_subscription_count() > 0) {
    const auto detection_pointcloud_ptr = convert_to_pointcloud(*msg);
    auto detection_pointcloud_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*detection_pointcloud_ptr, *detection_pointcloud_msg_ptr);

    detection_pointcloud_msg_ptr->header = msg->header;
    near_detection_pointcloud_pub_->publish(std::move(detection_pointcloud_msg_ptr));
  }

  if (
    near_scan_raw_pub_->get_subscription_count() > 0 ||
    near_scan_raw_pub_->get_intra_process_subscription_count() > 0) {
    auto radar_scan_msg = convert_to_radar_scan(*msg);
    radar_scan_msg.header = msg->header;
    near_scan_raw_pub_->publish(std::move(radar_scan_msg));
  }

  if (
    near_detection_list_pub_->get_subscription_count() > 0 ||
    near_detection_list_pub_->get_intra_process_subscription_count() > 0) {
    near_detection_list_pub_->publish(std::move(msg));
  }
}

void ContinentalSRR520DecoderWrapper::hrr_detection_list_callback(
  std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList> msg)
{
  if (
    hrr_detection_pointcloud_pub_->get_subscription_count() > 0 ||
    hrr_detection_pointcloud_pub_->get_intra_process_subscription_count() > 0) {
    const auto detection_pointcloud_ptr = convert_to_pointcloud(*msg);
    auto detection_pointcloud_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*detection_pointcloud_ptr, *detection_pointcloud_msg_ptr);

    detection_pointcloud_msg_ptr->header = msg->header;
    hrr_detection_pointcloud_pub_->publish(std::move(detection_pointcloud_msg_ptr));
  }

  if (
    hrr_scan_raw_pub_->get_subscription_count() > 0 ||
    hrr_scan_raw_pub_->get_intra_process_subscription_count() > 0) {
    auto radar_scan_msg = convert_to_radar_scan(*msg);
    radar_scan_msg.header = msg->header;
    hrr_scan_raw_pub_->publish(std::move(radar_scan_msg));
  }

  if (
    hrr_detection_list_pub_->get_subscription_count() > 0 ||
    hrr_detection_list_pub_->get_intra_process_subscription_count() > 0) {
    hrr_detection_list_pub_->publish(std::move(msg));
  }
}

void ContinentalSRR520DecoderWrapper::object_list_callback(
  std::unique_ptr<continental_msgs::msg::ContinentalSrr520ObjectList> msg)
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
}

void ContinentalSRR520DecoderWrapper::status_callback(
  std::unique_ptr<diagnostic_msgs::msg::DiagnosticArray> status_msg_ptr)
{
  status_pub_->publish(std::move(status_msg_ptr));
}

pcl::PointCloud<nebula::drivers::continental_srr520::PointSRR520Detection>::Ptr
ContinentalSRR520DecoderWrapper::convert_to_pointcloud(
  const continental_msgs::msg::ContinentalSrr520DetectionList & msg)
{
  pcl::PointCloud<nebula::drivers::continental_srr520::PointSRR520Detection>::Ptr output_pointcloud(
    new pcl::PointCloud<nebula::drivers::continental_srr520::PointSRR520Detection>);
  output_pointcloud->reserve(msg.detections.size());

  nebula::drivers::continental_srr520::PointSRR520Detection point{};
  for (const auto & detection : msg.detections) {
    point.x = std::cos(detection.azimuth_angle) * detection.range;
    point.y = std::sin(detection.azimuth_angle) * detection.range;
    point.z = 0.f;

    point.azimuth = detection.azimuth_angle;
    point.range = detection.range;
    point.range_rate = detection.range_rate;
    point.rcs = detection.rcs;
    point.snr = detection.snr;
    point.pdh00 = detection.pdh00;
    point.pdh01 = detection.pdh01;
    point.pdh02 = detection.pdh02;
    point.pdh03 = detection.pdh03;
    point.pdh04 = detection.pdh04;
    point.pdh05 = detection.pdh05;

    output_pointcloud->points.emplace_back(point);
  }

  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<nebula::drivers::continental_srr520::PointSRR520Object>::Ptr
ContinentalSRR520DecoderWrapper::convert_to_pointcloud(
  const continental_msgs::msg::ContinentalSrr520ObjectList & msg)
{
  pcl::PointCloud<nebula::drivers::continental_srr520::PointSRR520Object>::Ptr output_pointcloud(
    new pcl::PointCloud<nebula::drivers::continental_srr520::PointSRR520Object>);
  output_pointcloud->reserve(msg.objects.size());

  nebula::drivers::continental_srr520::PointSRR520Object point{};
  for (const auto & object : msg.objects) {
    point.x = object.dist_x;
    point.y = object.dist_y;
    point.z = 0.f;

    point.id = object.object_id;
    point.age = object.life_cycles;

    point.box_length = object.box_length;
    point.box_width = object.box_width;

    point.object_status = object.object_status;
    point.orientation = object.orientation;
    point.rcs = object.rcs;
    point.score = object.score;
    point.dynamics_abs_vel_x = object.v_abs_x;
    point.dynamics_abs_vel_y = object.v_abs_y;
    point.dynamics_abs_acc_x = object.a_abs_x;
    point.dynamics_abs_acc_y = object.a_abs_y;

    output_pointcloud->points.emplace_back(point);
  }

  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

radar_msgs::msg::RadarScan ContinentalSRR520DecoderWrapper::convert_to_radar_scan(
  const continental_msgs::msg::ContinentalSrr520DetectionList & msg)
{
  radar_msgs::msg::RadarScan output_msg;
  output_msg.header = msg.header;
  output_msg.returns.reserve(msg.detections.size());

  radar_msgs::msg::RadarReturn return_msg;
  for (const auto & detection : msg.detections) {
    if (detection.pdh00 > 0 || detection.pdh00 > 1 || detection.pdh02 > 0) {
      continue;
    }

    return_msg.range = detection.range;
    return_msg.azimuth = detection.azimuth_angle;
    return_msg.elevation = 0.f;
    return_msg.doppler_velocity = detection.range_rate;
    return_msg.amplitude = detection.rcs;
    output_msg.returns.emplace_back(return_msg);
  }

  return output_msg;
}

radar_msgs::msg::RadarTracks ContinentalSRR520DecoderWrapper::convert_to_radar_tracks(
  const continental_msgs::msg::ContinentalSrr520ObjectList & msg)
{
  radar_msgs::msg::RadarTracks output_msg;
  output_msg.tracks.reserve(msg.objects.size());
  output_msg.header = msg.header;

  constexpr int16_t unknown_id = 32000;
  constexpr float invalid_covariance = 1e6;

  radar_msgs::msg::RadarTrack track_msg;
  for (const auto & object : msg.objects) {
    if (!object.box_valid || object.object_status == 0) {
      continue;
    }

    track_msg.uuid.uuid[0] = static_cast<uint8_t>(object.object_id & 0xff);
    track_msg.uuid.uuid[1] = static_cast<uint8_t>((object.object_id >> 8) & 0xff);
    track_msg.uuid.uuid[2] = static_cast<uint8_t>((object.object_id >> 16) & 0xff);
    track_msg.uuid.uuid[3] = static_cast<uint8_t>((object.object_id >> 24) & 0xff);

    track_msg.position.x = static_cast<double>(object.dist_x);
    track_msg.position.y = static_cast<double>(object.dist_y);
    track_msg.position.z = 0.0;

    track_msg.velocity.x = static_cast<double>(object.v_abs_x);
    track_msg.velocity.y = static_cast<double>(object.v_abs_y);
    track_msg.velocity.z = 0.0;
    track_msg.acceleration.x = static_cast<double>(object.a_abs_x);
    track_msg.acceleration.y = static_cast<double>(object.a_abs_y);
    track_msg.acceleration.z = 0.0;
    track_msg.size.x = object.box_length;
    track_msg.size.y = object.box_width;
    track_msg.size.z = 1.f;

    track_msg.classification = unknown_id;

    track_msg.position_covariance[0] = object.dist_x_std * object.dist_x_std;
    track_msg.position_covariance[1] = invalid_covariance;
    track_msg.position_covariance[2] = 0.f;
    track_msg.position_covariance[3] = object.dist_y_std * object.dist_y_std;
    track_msg.position_covariance[4] = 0.f;
    track_msg.position_covariance[5] = invalid_covariance;

    track_msg.velocity_covariance[0] = object.v_abs_x_std * object.v_abs_x_std;
    track_msg.velocity_covariance[1] = invalid_covariance;
    track_msg.velocity_covariance[2] = 0.f;
    track_msg.velocity_covariance[3] = object.v_abs_y_std * object.v_abs_y_std;
    track_msg.velocity_covariance[4] = 0.f;
    track_msg.velocity_covariance[5] = invalid_covariance;

    track_msg.acceleration_covariance[0] = object.a_abs_x_std * object.a_abs_x_std;
    track_msg.acceleration_covariance[1] = invalid_covariance;
    track_msg.acceleration_covariance[2] = 0.f;
    track_msg.acceleration_covariance[3] = object.a_abs_y_std * object.a_abs_y_std;
    track_msg.acceleration_covariance[4] = 0.f;
    track_msg.acceleration_covariance[5] = invalid_covariance;

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

visualization_msgs::msg::MarkerArray ContinentalSRR520DecoderWrapper::convert_to_markers(
  const continental_msgs::msg::ContinentalSrr520ObjectList & msg)
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

  for (const auto & object : msg.objects) {
    if (!object.box_valid || object.object_status == 0) {
      continue;
    }

    const double half_length = 0.5 * object.box_length;
    const double half_width = 0.5 * object.box_width;
    constexpr double default_half_size = 1.0;

    const double & yaw = object.orientation;
    current_ids.emplace(object.object_id);

    visualization_msgs::msg::Marker box_marker;
    box_marker.header.frame_id = sensor_cfg_->base_frame;
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

    box_marker.pose.position.x = object.dist_x;
    box_marker.pose.position.y = object.dist_y;
    box_marker.pose.position.z = default_half_size;
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
    text_marker.text = "ID=" + std::to_string(object.object_id) +
                       " Age=" + std::to_string(object.life_cycles) + "ms";

    marker_array.markers.emplace_back(text_marker);

    std::stringstream object_status_ss;
    object_status_ss << std::fixed << std::setprecision(3)
                     << "ID=" << static_cast<uint32_t>(object.object_id) << "\n"
                     << static_cast<int>(object.box_length) << "/"
                     << static_cast<int>(object.object_status);

    text_marker.ns = "object_status";
    text_marker.text = object_status_ss.str();

    marker_array.markers.emplace_back(text_marker);

    std::stringstream object_dynamics_ss;
    object_dynamics_ss << std::fixed << std::setprecision(3)
                       << "ID=" << static_cast<uint32_t>(object.object_id)
                       << "\nyaw=" << object.orientation << "\nvx=" << object.v_abs_x
                       << "\nvy=" << object.v_abs_y << "\nax=" << object.a_abs_x
                       << "\nay=" << object.a_abs_y;

    text_marker.ns = "object_dynamics";
    text_marker.text = object_dynamics_ss.str();

    marker_array.markers.emplace_back(text_marker);
  }

  for (const auto & previous_id : previous_ids_) {
    if (current_ids.find(previous_id) != current_ids.end()) {
      continue;
    }

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = sensor_cfg_->base_frame;
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

void ContinentalSRR520DecoderWrapper::sync_follow_up_callback(builtin_interfaces::msg::Time stamp)
{
  hw_interface_ptr_->sensor_sync_follow_up(stamp);
}

void ContinentalSRR520DecoderWrapper::packets_callback(
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> msg)
{
  if (
    packets_pub_ && (packets_pub_->get_subscription_count() > 0 ||
                     packets_pub_->get_intra_process_subscription_count() > 0)) {
    packets_pub_->publish(std::move(msg));
  }
}

nebula::Status ContinentalSRR520DecoderWrapper::status()
{
  std::lock_guard lock(mtx_driver_ptr_);

  if (!driver_ptr_) {
    return nebula::Status::NOT_INITIALIZED;
  }

  return driver_ptr_->get_status();
}
}  // namespace nebula::ros
