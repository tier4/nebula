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

#include "nebula_decoders/nebula_decoders_continental/decoders/continental_ars548_decoder.hpp"

#include <nebula_common/continental/continental_ars548.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers::continental_ars548
{
ContinentalARS548Decoder::ContinentalARS548Decoder(
  const std::shared_ptr<const continental_ars548::ContinentalARS548SensorConfiguration> &
    sensor_configuration)
{
  config_ptr_ = sensor_configuration;
}

Status ContinentalARS548Decoder::get_status()
{
  return Status::OK;
}

Status ContinentalARS548Decoder::register_detection_list_callback(
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList>)>
    detection_list_callback)
{
  detection_list_callback_ = std::move(detection_list_callback);
  return Status::OK;
}

Status ContinentalARS548Decoder::register_object_list_callback(
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548ObjectList>)>
    object_list_callback)
{
  object_list_callback_ = std::move(object_list_callback);
  return Status::OK;
}

Status ContinentalARS548Decoder::register_sensor_status_callback(
  std::function<void(const ContinentalARS548Status & status)> sensor_status_callback)
{
  sensor_status_callback_ = std::move(sensor_status_callback);
  return Status::OK;
}

Status ContinentalARS548Decoder::register_packets_callback(
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets>)> nebula_packets_callback)
{
  nebula_packets_callback_ = std::move(nebula_packets_callback);
  return Status::OK;
}

bool ContinentalARS548Decoder::process_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  const auto & data = packet_msg->data;

  if (data.size() < sizeof(HeaderPacket)) {
    return false;
  }

  HeaderPacket header{};
  std::memcpy(&header, data.data(), sizeof(HeaderPacket));

  if (header.service_id.value() != 0) {
    return false;
  }

  if (header.method_id.value() == detection_list_method_id) {
    if (
      data.size() != detection_list_udp_payload ||
      header.length.value() != detection_list_pdu_length) {
      return false;
    }

    parse_detections_list_packet(*packet_msg);
  } else if (header.method_id.value() == object_list_method_id) {
    if (data.size() != object_list_udp_payload || header.length.value() != object_list_pdu_length) {
      return false;
    }

    parse_objects_list_packet(*packet_msg);
  } else if (header.method_id.value() == sensor_status_method_id) {
    if (
      data.size() != sensor_status_udp_payload ||
      header.length.value() != sensor_status_pdu_length) {
      return false;
    }

    parse_sensor_status_packet(*packet_msg);
  }

  // Some messages are not parsed but are still sent to the user (e.g., filters)
  if (nebula_packets_callback_) {
    auto packets_msg = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    packets_msg->packets.emplace_back(std::move(*packet_msg));
    packets_msg->header.stamp = packet_msg->stamp;
    packets_msg->header.frame_id = config_ptr_->frame_id;
    nebula_packets_callback_(std::move(packets_msg));
  }

  return true;
}

bool ContinentalARS548Decoder::parse_detections_list_packet(
  const nebula_msgs::msg::NebulaPacket & packet_msg)
{
  auto msg_ptr = std::make_unique<continental_msgs::msg::ContinentalArs548DetectionList>();
  auto & msg = *msg_ptr;

  DetectionListPacket detection_list;
  assert(sizeof(DetectionListPacket) == packet_msg.data.size());

  std::memcpy(&detection_list, packet_msg.data.data(), sizeof(DetectionListPacket));

  msg.header.frame_id = config_ptr_->frame_id;

  if (config_ptr_->use_sensor_time) {
    msg.header.stamp.nanosec = detection_list.stamp.timestamp_nanoseconds.value();
    msg.header.stamp.sec = detection_list.stamp.timestamp_seconds.value();
  } else {
    msg.header.stamp = packet_msg.stamp;
  }

  msg.stamp_sync_status = detection_list.stamp.timestamp_sync_status;
  assert(msg.stamp_sync_status >= 1 && msg.stamp_sync_status <= 3);

  msg.origin_pos.x = detection_list.origin_x_pos.value();
  msg.origin_pos.y = detection_list.origin_y_pos.value();
  msg.origin_pos.z = detection_list.origin_z_pos.value();

  msg.origin_pitch = detection_list.origin_pitch.value();
  msg.origin_pitch_std = detection_list.origin_pitch_std.value();
  msg.origin_yaw = detection_list.origin_yaw.value();
  msg.origin_yaw_std = detection_list.origin_yaw_std.value();

  msg.ambiguity_free_velocity_min = detection_list.list_rad_vel_domain_min.value();
  msg.ambiguity_free_velocity_max = detection_list.list_rad_vel_domain_max.value();

  msg.alignment_azimuth_correction = detection_list.alignment_azimuth_correction.value();
  msg.alignment_elevation_correction = detection_list.alignment_elevation_correction.value();

  msg.alignment_status = detection_list.alignment_status;

  const uint32_t number_of_detections = detection_list.number_of_detections.value();
  msg.detections.resize(number_of_detections);

  // Estimate dropped detections only when the radar is synchronized
  if (radar_status_.timestamp_sync_status == "1:SYNC_OK") {
    if (radar_status_.detection_first_stamp == 0) {
      radar_status_.detection_first_stamp =
        static_cast<uint64_t>(msg.header.stamp.sec) * 1'000'000'000 +
        static_cast<uint64_t>(msg.header.stamp.nanosec);
      radar_status_.detection_last_stamp = radar_status_.detection_first_stamp;
    } else {
      uint64_t stamp = static_cast<uint64_t>(msg.header.stamp.sec) * 1'000'000'000 +
                       static_cast<uint64_t>(msg.header.stamp.nanosec);
      radar_status_.detection_total_count++;
      radar_status_.detection_empty_count += number_of_detections == 0 ? 1 : 0;
      radar_status_.detection_dropped_dt_count +=
        (stamp - radar_status_.detection_last_stamp > 1.75 * radar_status_.cycle_time * 10e6) ? 1
                                                                                              : 0;
      radar_status_.detection_last_stamp = stamp;
    }
  }

  assert(msg.origin_pos.x >= -10.f && msg.origin_pos.x <= 10.f);
  assert(msg.origin_pos.y >= -10.f && msg.origin_pos.y <= 10.f);
  assert(msg.origin_pos.z >= -10.f && msg.origin_pos.z <= 10.f);
  assert(msg.origin_pitch >= -M_PI && msg.origin_pitch <= M_PI);
  assert(msg.origin_yaw >= -M_PI && msg.origin_yaw <= M_PI);
  assert(msg.ambiguity_free_velocity_min >= -100.f && msg.ambiguity_free_velocity_min <= 100.f);
  assert(msg.ambiguity_free_velocity_max >= -100.f && msg.ambiguity_free_velocity_max <= 100.f);
  assert(number_of_detections <= 800);
  assert(msg.alignment_azimuth_correction >= -M_PI && msg.alignment_azimuth_correction <= M_PI);
  assert(msg.alignment_elevation_correction >= -M_PI && msg.alignment_elevation_correction <= M_PI);

  for (std::size_t detection_index = 0; detection_index < number_of_detections; detection_index++) {
    auto & detection_msg = msg.detections[detection_index];
    auto & detection = detection_list.detections[detection_index];

    assert(detection.raw_positive_predictive_value <= 100);
    assert(detection.classification <= 4 || detection.classification == 255);
    assert(detection.raw_multi_target_probability <= 100);
    assert(detection.raw_ambiguity_flag <= 100);

    assert(detection.azimuth_angle.value() >= -M_PI && detection.azimuth_angle.value() <= M_PI);
    assert(
      detection.azimuth_angle_std.value() >= 0.f && detection.azimuth_angle_std.value() <= 1.f);
    assert(detection.elevation_angle.value() >= -M_PI && detection.elevation_angle.value() <= M_PI);
    assert(
      detection.elevation_angle_std.value() >= 0.f && detection.elevation_angle_std.value() <= 1.f);

    assert(detection.range.value() >= 0.f && detection.range.value() <= 1500.f);
    assert(detection.range_std.value() >= 0.f && detection.range_std.value() <= 1.f);
    assert(detection.range_rate_std.value() >= 0.f && detection.range_rate_std.value() <= 1.f);

    detection_msg.invalid_distance = detection.invalid_flags & 0x01;
    detection_msg.invalid_distance_std = detection.invalid_flags & 0x02;
    detection_msg.invalid_azimuth = detection.invalid_flags & 0x04;
    detection_msg.invalid_azimuth_std = detection.invalid_flags & 0x08;
    detection_msg.invalid_elevation = detection.invalid_flags & 0x10;
    detection_msg.invalid_elevation_std = detection.invalid_flags & 0x20;
    detection_msg.invalid_range_rate = detection.invalid_flags & 0x40;
    detection_msg.invalid_range_rate_std = detection.invalid_flags & 0x80;
    detection_msg.rcs = detection.rcs;
    detection_msg.measurement_id = detection.measurement_id.value();
    detection_msg.raw_positive_predictive_value = detection.raw_positive_predictive_value;
    detection_msg.classification = detection.classification;
    detection_msg.raw_multi_target_probability = detection.raw_multi_target_probability;
    detection_msg.object_id = detection.object_id.value();
    detection_msg.raw_ambiguity_flag = detection.raw_ambiguity_flag;

    detection_msg.azimuth_angle = detection.azimuth_angle.value();
    detection_msg.azimuth_angle_std = detection.azimuth_angle_std.value();
    detection_msg.elevation_angle = detection.elevation_angle.value();
    detection_msg.elevation_angle_std = detection.elevation_angle_std.value();

    detection_msg.range = detection.range.value();
    detection_msg.range_std = detection.range_std.value();
    detection_msg.range_rate = detection.range_rate.value();
    detection_msg.range_rate_std = detection.range_rate_std.value();
  }

  if (detection_list_callback_) {
    detection_list_callback_(std::move(msg_ptr));
  }

  return true;
}

bool ContinentalARS548Decoder::parse_objects_list_packet(
  const nebula_msgs::msg::NebulaPacket & packet_msg)
{
  // cSpell:ignore knzo25
  // NOTE(knzo25): In the radar firmware used when developing this driver,
  // corner radars were not supported. When a new firmware addresses this,
  // the driver will be updated.
  if (nebula::drivers::continental_ars548::is_corner_radar(radar_status_.yaw)) {
    return true;
  }

  auto msg_ptr = std::make_unique<continental_msgs::msg::ContinentalArs548ObjectList>();
  auto & msg = *msg_ptr;

  ObjectListPacket object_list;
  assert(sizeof(ObjectListPacket) == packet_msg.data.size());

  std::memcpy(&object_list, packet_msg.data.data(), sizeof(object_list));

  msg.header.frame_id = config_ptr_->object_frame;

  if (config_ptr_->use_sensor_time) {
    msg.header.stamp.nanosec = object_list.stamp.timestamp_nanoseconds.value();
    msg.header.stamp.sec = object_list.stamp.timestamp_seconds.value();
  } else {
    msg.header.stamp = packet_msg.stamp;
  }

  msg.stamp_sync_status = object_list.stamp.timestamp_sync_status;
  assert(msg.stamp_sync_status >= 1 && msg.stamp_sync_status <= 3);

  const uint8_t number_of_objects = object_list.number_of_objects;

  msg.objects.resize(number_of_objects);

  // Estimate dropped objects only when the radar is synchronized
  if (radar_status_.timestamp_sync_status == "1:SYNC_OK") {
    if (radar_status_.object_first_stamp == 0) {
      radar_status_.object_first_stamp =
        static_cast<uint64_t>(msg.header.stamp.sec) * 1'000'000'000 +
        static_cast<uint64_t>(msg.header.stamp.nanosec);
      radar_status_.object_last_stamp = radar_status_.object_first_stamp;
    } else {
      uint64_t stamp = static_cast<uint64_t>(msg.header.stamp.sec) * 1'000'000'000 +
                       static_cast<uint64_t>(msg.header.stamp.nanosec);
      radar_status_.object_total_count++;
      radar_status_.object_empty_count += number_of_objects == 0 ? 1 : 0;
      radar_status_.object_dropped_dt_count +=
        (stamp - radar_status_.object_last_stamp > 1.75 * radar_status_.cycle_time * 10e6) ? 1 : 0;
      radar_status_.object_last_stamp = stamp;
    }
  }

  for (std::size_t object_index = 0; object_index < number_of_objects; object_index++) {
    auto & object_msg = msg.objects[object_index];
    const ObjectPacket & object = object_list.objects[object_index];

    assert(object.status_measurement <= 2 || object.status_measurement == 255);
    assert(object.status_movement <= 1 || object.status_movement == 255);
    assert(object.position_reference <= 7 || object.position_reference == 255);

    assert(object.position_x.value() >= -1600.f && object.position_x.value() <= 1600.f);
    assert(object.position_x_std.value() >= 0.f);
    assert(object.position_y.value() >= -1600.f && object.position_y.value() <= 1600.f);
    assert(object.position_y_std.value() >= 0.f);
    assert(object.position_z.value() >= -1600.f && object.position_z.value() <= 1600.f);
    assert(object.position_z_std.value() >= 0.f);
    assert(
      object.position_orientation.value() >= -M_PI && object.position_orientation.value() <= M_PI);
    assert(object.position_orientation_std.value() >= 0.f);

    assert(object.raw_classification_car <= 100);
    assert(object.raw_classification_truck <= 100);
    assert(object.raw_classification_motorcycle <= 100);
    assert(object.raw_classification_bicycle <= 100);
    assert(object.raw_classification_pedestrian <= 100);
    assert(object.raw_classification_animal <= 100);
    assert(object.raw_classification_hazard <= 100);
    assert(object.raw_classification_unknown <= 100);

    assert(object.dynamics_abs_vel_x_std.value() >= 0.f);
    assert(object.dynamics_abs_vel_y_std.value() >= 0.f);

    assert(object.dynamics_rel_vel_x_std.value() >= 0.f);
    assert(object.dynamics_rel_vel_y_std.value() >= 0.f);

    assert(object.dynamics_abs_accel_x_std.value() >= 0.f);
    assert(object.dynamics_abs_accel_y_std.value() >= 0.f);

    assert(object.dynamics_rel_accel_x_std.value() >= 0.f);
    assert(object.dynamics_rel_accel_y_std.value() >= 0.f);

    object_msg.object_id = object.id.value();
    object_msg.age = object.age.value();
    object_msg.status_measurement = object.status_measurement;
    object_msg.status_movement = object.status_movement;
    object_msg.position_reference = object.position_reference;

    object_msg.position.x = static_cast<double>(object.position_x.value());
    object_msg.position.y = static_cast<double>(object.position_y.value());
    object_msg.position.z = static_cast<double>(object.position_z.value());

    object_msg.position_std.x = static_cast<double>(object.position_x_std.value());
    object_msg.position_std.y = static_cast<double>(object.position_y_std.value());
    object_msg.position_std.z = static_cast<double>(object.position_z_std.value());

    object_msg.position_covariance_xy = object.position_covariance_xy.value();

    object_msg.orientation = object.position_orientation.value();
    object_msg.orientation_std = object.position_orientation_std.value();

    object_msg.raw_existence_probability = object.raw_existence_probability.value();
    object_msg.raw_classification_car = object.raw_classification_car;
    object_msg.raw_classification_truck = object.raw_classification_truck;
    object_msg.raw_classification_motorcycle = object.raw_classification_motorcycle;
    object_msg.raw_classification_bicycle = object.raw_classification_bicycle;
    object_msg.raw_classification_pedestrian = object.raw_classification_pedestrian;
    object_msg.raw_classification_animal = object.raw_classification_animal;
    object_msg.raw_classification_hazard = object.raw_classification_hazard;
    object_msg.raw_classification_unknown = object.raw_classification_unknown;

    object_msg.absolute_velocity.x = static_cast<double>(object.dynamics_abs_vel_x.value());
    object_msg.absolute_velocity.y = static_cast<double>(object.dynamics_abs_vel_y.value());
    object_msg.absolute_velocity_std.x = static_cast<double>(object.dynamics_abs_vel_x_std.value());
    object_msg.absolute_velocity_std.y = static_cast<double>(object.dynamics_abs_vel_y_std.value());
    object_msg.absolute_velocity_covariance_xy = object.dynamics_abs_vel_covariance_xy.value();

    object_msg.relative_velocity.x = static_cast<double>(object.dynamics_rel_vel_x.value());
    object_msg.relative_velocity.y = static_cast<double>(object.dynamics_rel_vel_y.value());
    object_msg.relative_velocity_std.x = static_cast<double>(object.dynamics_rel_vel_x_std.value());
    object_msg.relative_velocity_std.y = static_cast<double>(object.dynamics_rel_vel_y_std.value());
    object_msg.relative_velocity_covariance_xy = object.dynamics_rel_vel_covariance_xy.value();

    object_msg.absolute_acceleration.x = static_cast<double>(object.dynamics_abs_accel_x.value());
    object_msg.absolute_acceleration.y = static_cast<double>(object.dynamics_abs_accel_y.value());
    object_msg.absolute_acceleration_std.x =
      static_cast<double>(object.dynamics_abs_accel_x_std.value());
    object_msg.absolute_acceleration_std.y =
      static_cast<double>(object.dynamics_abs_accel_y_std.value());
    object_msg.absolute_acceleration_covariance_xy =
      object.dynamics_abs_accel_covariance_xy.value();

    object_msg.relative_velocity.x = object.dynamics_rel_accel_x.value();
    object_msg.relative_velocity.y = object.dynamics_rel_accel_y.value();
    object_msg.relative_velocity_std.x = object.dynamics_rel_accel_x_std.value();
    object_msg.relative_velocity_std.y = object.dynamics_rel_accel_y_std.value();
    object_msg.relative_velocity_covariance_xy = object.dynamics_rel_accel_covariance_xy.value();

    object_msg.orientation_rate_mean = object.dynamics_orientation_rate_mean.value();
    object_msg.orientation_rate_std = object.dynamics_orientation_rate_std.value();

    object_msg.shape_length_edge_mean = object.shape_length_edge_mean.value();
    object_msg.shape_width_edge_mean = object.shape_width_edge_mean.value();
  }

  if (object_list_callback_) {
    object_list_callback_(std::move(msg_ptr));
  }

  return true;
}

bool ContinentalARS548Decoder::parse_sensor_status_packet(
  const nebula_msgs::msg::NebulaPacket & packet_msg)
{
  SensorStatusPacket sensor_status_packet;
  std::memcpy(&sensor_status_packet, packet_msg.data.data(), sizeof(SensorStatusPacket));

  radar_status_.timestamp_nanoseconds = sensor_status_packet.stamp.timestamp_nanoseconds.value();
  radar_status_.timestamp_seconds = sensor_status_packet.stamp.timestamp_seconds.value();

  switch (sensor_status_packet.stamp.timestamp_sync_status) {
    case sync_ok:
      radar_status_.timestamp_sync_status = "1:SYNC_OK";
      break;
    case never_sync:
      radar_status_.timestamp_sync_status = "2:NEVER_SYNC";
      break;
    case sync_lost:
      radar_status_.timestamp_sync_status = "3:SYNC_LOST";
      break;
    default:
      radar_status_.timestamp_sync_status =
        std::to_string(sensor_status_packet.stamp.timestamp_sync_status) + ":Invalid";
      break;
  }

  radar_status_.sw_version_major = sensor_status_packet.sw_version_major;
  radar_status_.sw_version_minor = sensor_status_packet.sw_version_minor;
  radar_status_.sw_version_patch = sensor_status_packet.sw_version_patch;

  switch (sensor_status_packet.status.plug_orientation) {
    case plug_right:
      radar_status_.plug_orientation = "0:PLUG_RIGHT";
      break;
    case plug_left:
      radar_status_.plug_orientation = "1:PLUG_LEFT";
      break;
    default:
      radar_status_.plug_orientation =
        std::to_string(sensor_status_packet.status.plug_orientation) + ":Invalid";
      break;
  }

  radar_status_.max_distance = sensor_status_packet.status.maximum_distance.value();

  radar_status_.longitudinal = sensor_status_packet.status.longitudinal.value();
  radar_status_.lateral = sensor_status_packet.status.lateral.value();
  radar_status_.vertical = sensor_status_packet.status.vertical.value();
  radar_status_.yaw = sensor_status_packet.status.yaw.value();

  radar_status_.pitch = sensor_status_packet.status.pitch.value();
  radar_status_.length = sensor_status_packet.status.length.value();
  radar_status_.width = sensor_status_packet.status.width.value();
  radar_status_.height = sensor_status_packet.status.height.value();
  radar_status_.wheel_base = sensor_status_packet.status.wheelbase.value();

  switch (sensor_status_packet.status.frequency_slot) {
    case frequency_slot_low:
      radar_status_.frequency_slot = "0:Low (76.23 GHz)";
      break;
    case frequency_slot_mid:
      radar_status_.frequency_slot = "1:Mid (76.48 GHz)";
      break;
    case frequency_slot_high:
      radar_status_.frequency_slot = "2:High (76.73 GHz)";
      break;
    default:
      radar_status_.frequency_slot =
        std::to_string(sensor_status_packet.status.frequency_slot) + ":Invalid";
      break;
  }

  radar_status_.cycle_time = sensor_status_packet.status.cycle_time;
  radar_status_.time_slot = sensor_status_packet.status.time_slot;

  switch (sensor_status_packet.status.hcc) {
    case hcc_worldwide:
      radar_status_.hcc = "1:Worldwide";
      break;
    case hcc_japan:
      radar_status_.hcc = "2:Japan";
      break;
    default:
      radar_status_.hcc = std::to_string(sensor_status_packet.status.hcc) + ":Invalid hcc";
      break;
  }

  switch (sensor_status_packet.status.powersave_standstill) {
    case powersave_standstill_off:
      radar_status_.power_save_standstill = "0:Off";
      break;
    case powersave_standstill_on:
      radar_status_.power_save_standstill = "1:On";
      break;
    default:
      radar_status_.power_save_standstill =
        std::to_string(sensor_status_packet.status.powersave_standstill) + ":Invalid";
      break;
  }

  std::stringstream ss0, ss1;
  ss0 << std::to_string(sensor_status_packet.status.sensor_ip_address00) << "."
      << std::to_string(sensor_status_packet.status.sensor_ip_address01) << "."
      << std::to_string(sensor_status_packet.status.sensor_ip_address02) << "."
      << std::to_string(sensor_status_packet.status.sensor_ip_address03);
  radar_status_.sensor_ip_address0 = ss0.str();

  ss1 << std::to_string(sensor_status_packet.status.sensor_ip_address10) << "."
      << std::to_string(sensor_status_packet.status.sensor_ip_address11) << "."
      << std::to_string(sensor_status_packet.status.sensor_ip_address12) << "."
      << std::to_string(sensor_status_packet.status.sensor_ip_address13);
  radar_status_.sensor_ip_address1 = ss1.str();

  radar_status_.configuration_counter = sensor_status_packet.configuration_counter;

  auto vdy_value_to_string = [](uint8_t value) -> std::string {
    switch (value) {
      case vdy_ok:
        return "0:VDY_OK";
      case vdy_notok:
        return "1:VDY_NOTOK";
      default:
        return std::to_string(value) + ":Invalid";
    }
  };

  radar_status_.longitudinal_velocity_status =
    vdy_value_to_string(sensor_status_packet.longitudinal_velocity_status);
  radar_status_.longitudinal_acceleration_status =
    vdy_value_to_string(sensor_status_packet.longitudinal_acceleration_status);
  radar_status_.lateral_acceleration_status =
    vdy_value_to_string(sensor_status_packet.lateral_acceleration_status);

  radar_status_.yaw_rate_status = vdy_value_to_string(sensor_status_packet.yaw_rate_status);
  radar_status_.steering_angle_status =
    vdy_value_to_string(sensor_status_packet.steering_angle_status);
  radar_status_.driving_direction_status =
    vdy_value_to_string(sensor_status_packet.driving_direction_status);
  radar_status_.characteristic_speed_status =
    vdy_value_to_string(sensor_status_packet.characteristic_speed_status);

  switch (sensor_status_packet.radar_status) {
    case state_init:
      radar_status_.radar_status = "0:STATE_INIT";
      break;
    case state_ok:
      radar_status_.radar_status = "1:STATE_OK";
      break;
    case state_invalid:
      radar_status_.radar_status = "2:STATE_INVALID";
      break;
    default:
      radar_status_.radar_status = std::to_string(sensor_status_packet.radar_status) + ":Invalid";
      break;
  }

  std::vector<std::string> voltage_status_vector, temperature_status_vector;

  if (sensor_status_packet.voltage_status == 0) {
    voltage_status_vector.push_back("Ok");
  }
  if (sensor_status_packet.voltage_status & 0x01) {
    voltage_status_vector.push_back("Current undervoltage");
  }
  if (sensor_status_packet.voltage_status & 0x02) {
    voltage_status_vector.push_back("Past undervoltage");
  }
  if (sensor_status_packet.voltage_status & 0x04) {
    voltage_status_vector.push_back("Current overvoltage");
  }
  if (sensor_status_packet.voltage_status & 0x08) {
    voltage_status_vector.push_back("Past overvoltage");
  }

  if (sensor_status_packet.temperature_status == 0) {
    temperature_status_vector.push_back("Ok");
  }
  if (sensor_status_packet.temperature_status & 0x01) {
    temperature_status_vector.push_back("Current undertemperature");
  }
  if (sensor_status_packet.temperature_status & 0x02) {
    temperature_status_vector.push_back("Past undertemperature");
  }
  if (sensor_status_packet.temperature_status & 0x04) {
    temperature_status_vector.push_back("Current overtemperature");
  }
  if (sensor_status_packet.temperature_status & 0x08) {
    temperature_status_vector.push_back("Past overtemperature");
  }

  radar_status_.voltage_status = boost::algorithm::join(voltage_status_vector, ", ");
  radar_status_.temperature_status = boost::algorithm::join(temperature_status_vector, ", ");

  const uint8_t & blockage_status0 = sensor_status_packet.blockage_status & 0x0f;
  const uint8_t & blockage_status1 = (sensor_status_packet.blockage_status & 0xf0) >> 4;

  switch (blockage_status0) {
    case blockage_status_blind:
      radar_status_.blockage_status = "0:Blind";
      break;
    case blockage_status_high:
      radar_status_.blockage_status = "1:High";
      break;
    case blockage_status_mid:
      radar_status_.blockage_status = "2:Mid";
      break;
    case blockage_status_low:
      radar_status_.blockage_status = "3:Low";
      break;
    case blockage_status_none:
      radar_status_.blockage_status = "4:None";
      break;
    default:
      radar_status_.blockage_status = std::to_string(blockage_status0) + ":Invalid";
      break;
  }

  switch (blockage_status1) {
    case blockage_test_failed:
      radar_status_.blockage_status += ". 0:Self test failed";
      break;
    case blockage_test_passed:
      radar_status_.blockage_status += ". 1:Self test passed";
      break;
    case blockage_test_ongoing:
      radar_status_.blockage_status += ". 2:Self test ongoing";
      break;
    default:
      radar_status_.blockage_status += std::to_string(blockage_status1) + ":Invalid";
      break;
  }

  radar_status_.status_total_count++;
  radar_status_.radar_invalid_count += sensor_status_packet.radar_status == 2 ? 1 : 0;

  if (sensor_status_callback_) {
    sensor_status_callback_(radar_status_);
  }

  return true;
}

}  // namespace nebula::drivers::continental_ars548
