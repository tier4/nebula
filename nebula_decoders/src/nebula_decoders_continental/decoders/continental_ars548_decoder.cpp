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

#include "nebula_decoders/nebula_decoders_continental/decoders/continental_ars548_decoder.hpp"

#include "nebula_common/continental/continental_ars548.hpp"

#include <cmath>
#include <utility>

namespace nebula
{
namespace drivers
{
namespace continental_ars548
{
ContinentalARS548Decoder::ContinentalARS548Decoder(
  const std::shared_ptr<continental_ars548::ContinentalARS548SensorConfiguration> &
    sensor_configuration)
{
  sensor_configuration_ = sensor_configuration;
}

Status ContinentalARS548Decoder::RegisterDetectionListCallback(
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548DetectionList>)>
    detection_list_callback)
{
  detection_list_callback_ = std::move(detection_list_callback);
  return Status::OK;
}

Status ContinentalARS548Decoder::RegisterObjectListCallback(
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalArs548ObjectList>)>
    object_list_callback)
{
  object_list_callback_ = std::move(object_list_callback);
  return Status::OK;
}

Status ContinentalARS548Decoder::RegisterSensorStatusCallback(
  std::function<void(const ContinentalARS548Status & status)> sensor_status_callback)
{
  sensor_status_callback_ = std::move(sensor_status_callback);
  return Status::OK;
}

bool ContinentalARS548Decoder::ProcessPackets(
  const nebula_msgs::msg::NebulaPackets & nebula_packets)
{
  if (nebula_packets.packets.size() != 1) {
    return false;
  }

  const auto & data = nebula_packets.packets[0].data;

  if (data.size() < sizeof(HeaderPacket)) {
    return false;
  }

  HeaderPacket header{};
  std::memcpy(&header, data.data(), sizeof(HeaderPacket));

  if (header.service_id.value() != 0) {
    return false;
  }

  if (header.method_id.value() == DETECTION_LIST_METHOD_ID) {
    if (
      data.size() != DETECTION_LIST_UDP_PAYLOAD ||
      header.length.value() != DETECTION_LIST_PDU_LENGTH) {
      return false;
    }

    return ParseDetectionsListPacket(data, nebula_packets.header);
  } else if (header.method_id.value() == OBJECT_LIST_METHOD_ID) {
    if (data.size() != OBJECT_LIST_UDP_PAYLOAD || header.length.value() != OBJECT_LIST_PDU_LENGTH) {
      return false;
    }

    return ParseObjectsListPacket(data, nebula_packets.header);
  } else if (header.method_id.value() == SENSOR_STATUS_METHOD_ID) {
    if (
      data.size() != SENSOR_STATUS_UDP_PAYLOAD ||
      header.length.value() != SENSOR_STATUS_PDU_LENGTH) {
      return false;
    }

    return ParseSensorStatusPacket(data, nebula_packets.header);
  }

  return true;
}

bool ContinentalARS548Decoder::ParseDetectionsListPacket(
  const std::vector<uint8_t> & data, const std_msgs::msg::Header & header)
{
  auto msg_ptr = std::make_unique<continental_msgs::msg::ContinentalArs548DetectionList>();
  auto & msg = *msg_ptr;

  DetectionListPacket detection_list;
  assert(sizeof(DetectionListPacket) == data.size());

  std::memcpy(&detection_list, data.data(), sizeof(DetectionListPacket));

  msg.header.frame_id = sensor_configuration_->frame_id;

  if (sensor_configuration_->use_sensor_time) {
    msg.header.stamp.nanosec = detection_list.stamp.timestamp_nanoseconds.value();
    msg.header.stamp.sec = detection_list.stamp.timestamp_seconds.value();
  } else {
    msg.header.stamp = header.stamp;
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

  // Estimate dropped detections only when the radar is synchronzied
  if (radar_status_.timestamp_sync_status == "SYNC_OK") {
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

    assert(detection.positive_predictive_value <= 100);
    assert(detection.classification <= 4 || detection.classification == 255);
    assert(detection.multi_target_probability <= 100);
    assert(detection.ambiguity_flag <= 100);

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
    detection_msg.positive_predictive_value = detection.positive_predictive_value;
    detection_msg.classification = detection.classification;
    detection_msg.multi_target_probability = detection.multi_target_probability;
    detection_msg.object_id = detection.object_id.value();
    detection_msg.ambiguity_flag = detection.ambiguity_flag;

    detection_msg.azimuth_angle = detection.azimuth_angle.value();
    detection_msg.azimuth_angle_std = detection.azimuth_angle_std.value();
    detection_msg.elevation_angle = detection.elevation_angle.value();
    detection_msg.elevation_angle_std = detection.elevation_angle_std.value();

    detection_msg.range = detection.range.value();
    detection_msg.range_std = detection.range_std.value();
    detection_msg.range_rate = detection.range_rate.value();
    detection_msg.range_rate_std = detection.range_rate_std.value();
  }

  detection_list_callback_(std::move(msg_ptr));

  return true;
}

bool ContinentalARS548Decoder::ParseObjectsListPacket(
  const std::vector<uint8_t> & data, const std_msgs::msg::Header & header)
{
  auto msg_ptr = std::make_unique<continental_msgs::msg::ContinentalArs548ObjectList>();
  auto & msg = *msg_ptr;

  ObjectListPacket object_list;
  assert(sizeof(ObjectListPacket) == data.size());

  std::memcpy(&object_list, data.data(), sizeof(object_list));

  msg.header.frame_id = sensor_configuration_->base_frame;

  if (sensor_configuration_->use_sensor_time) {
    msg.header.stamp.nanosec = object_list.stamp.timestamp_nanoseconds.value();
    msg.header.stamp.sec = object_list.stamp.timestamp_seconds.value();
  } else {
    msg.header.stamp = header.stamp;
  }

  msg.stamp_sync_status = object_list.stamp.timestamp_sync_status;
  assert(msg.stamp_sync_status >= 1 && msg.stamp_sync_status <= 3);

  const uint8_t number_of_objects = object_list.number_of_objects;

  msg.objects.resize(number_of_objects);

  // Estimate dropped objects only when the radar is synchronzied
  if (radar_status_.timestamp_sync_status == "SYNC_OK") {
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

    assert(object.classification_car <= 100);
    assert(object.classification_truck <= 100);
    assert(object.classification_motorcycle <= 100);
    assert(object.classification_bicycle <= 100);
    assert(object.classification_pedestrian <= 100);
    assert(object.classification_animal <= 100);
    assert(object.classification_hazard <= 100);
    assert(object.classification_unknown <= 100);

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

    object_msg.existence_probability = object.existence_probability.value();
    object_msg.classification_car = object.classification_car;
    object_msg.classification_truck = object.classification_truck;
    object_msg.classification_motorcycle = object.classification_motorcycle;
    object_msg.classification_bicycle = object.classification_bicycle;
    object_msg.classification_pedestrian = object.classification_pedestrian;
    object_msg.classification_animal = object.classification_animal;
    object_msg.classification_hazard = object.classification_hazard;
    object_msg.classification_unknown = object.classification_unknown;

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

  object_list_callback_(std::move(msg_ptr));

  return true;
}

bool ContinentalARS548Decoder::ParseSensorStatusPacket(
  const std::vector<uint8_t> & data, const std_msgs::msg::Header & header)
{
  SensorStatusPacket sensor_status_packet;
  std::memcpy(&sensor_status_packet, data.data(), sizeof(SensorStatusPacket));

  radar_status_.timestamp_nanoseconds = sensor_status_packet.stamp.timestamp_nanoseconds.value();
  radar_status_.timestamp_seconds = sensor_status_packet.stamp.timestamp_seconds.value();

  if (sensor_status_packet.stamp.timestamp_sync_status == 1) {
    radar_status_.timestamp_sync_status = "SYNC_OK";
  } else if (sensor_status_packet.stamp.timestamp_sync_status == 2) {
    radar_status_.timestamp_sync_status = "NEVER_SYNC";
  } else if (sensor_status_packet.stamp.timestamp_sync_status == 3) {
    radar_status_.timestamp_sync_status = "SYNC_LOST";
  } else {
    radar_status_.timestamp_sync_status = "INVALID_VALUE";
  }

  radar_status_.sw_version_major = sensor_status_packet.sw_version_major;
  radar_status_.sw_version_minor = sensor_status_packet.sw_version_minor;
  radar_status_.sw_version_patch = sensor_status_packet.sw_version_patch;

  radar_status_.plug_orientation = sensor_status_packet.status.plug_orientation == 0 ? "PLUG_RIGHT"
                                   : sensor_status_packet.status.plug_orientation == 1
                                     ? "PLUG_LEFT"
                                     : "INVALID_VALUE";

  radar_status_.max_distance = sensor_status_packet.status.maximum_distance.value();

  radar_status_.longitudinal = sensor_status_packet.status.longitudinal.value();
  radar_status_.lateral = sensor_status_packet.status.lateral.value();
  radar_status_.vertical = sensor_status_packet.status.vertical.value();
  radar_status_.yaw = sensor_status_packet.status.yaw.value();

  radar_status_.pitch = sensor_status_packet.status.pitch.value();
  ;
  radar_status_.length = sensor_status_packet.status.length.value();
  ;
  radar_status_.width = sensor_status_packet.status.width.value();
  radar_status_.height = sensor_status_packet.status.height.value();
  radar_status_.wheel_base = sensor_status_packet.status.wheelbase.value();

  if (sensor_status_packet.status.frequency_slot == 0) {
    radar_status_.frequency_slot = "0:Low (76.23 GHz)";
  } else if (sensor_status_packet.status.frequency_slot == 1) {
    radar_status_.frequency_slot = "1:Mid (76.48 GHz)";
  } else if (sensor_status_packet.status.frequency_slot == 2) {
    radar_status_.frequency_slot = "2:High (76.73 GHz)";
  } else {
    radar_status_.frequency_slot = "INVALID VALUE";
  }

  radar_status_.cycle_time = sensor_status_packet.status.cycle_time;
  radar_status_.time_slot = sensor_status_packet.status.time_slot;

  radar_status_.hcc = sensor_status_packet.status.hcc == 1 ? "Worldwide"
                      : sensor_status_packet.status.hcc == 2
                        ? "Japan"
                        : ("INVALID VALUE=" + std::to_string(sensor_status_packet.status.hcc));

  radar_status_.power_save_standstill =
    sensor_status_packet.status.powersave_standstill == 0   ? "Off"
    : sensor_status_packet.status.powersave_standstill == 1 ? "On"
                                                            : "INVALID VALUE";

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

  radar_status_.longitudinal_velocity_status =
    sensor_status_packet.longitudinal_velocity_status == 0   ? "VDY_OK"
    : sensor_status_packet.longitudinal_velocity_status == 1 ? "VDY_NOTOK"
                                                             : "INVALID VALUE";

  radar_status_.longitudinal_acceleration_status =
    sensor_status_packet.longitudinal_acceleration_status == 0   ? "VDY_OK"
    : sensor_status_packet.longitudinal_acceleration_status == 1 ? "VDY_NOTOK"
                                                                 : "INVALID VALUE";

  radar_status_.lateral_acceleration_status =
    sensor_status_packet.lateral_acceleration_status == 0   ? "VDY_OK"
    : sensor_status_packet.lateral_acceleration_status == 1 ? "VDY_NOTOK"
                                                            : "INVALID VALUE";

  radar_status_.yaw_rate_status = sensor_status_packet.yaw_rate_status == 0   ? "VDY_OK"
                                  : sensor_status_packet.yaw_rate_status == 1 ? "VDY_NOTOK"
                                                                              : "INVALID VALUE";

  radar_status_.steering_angle_status = sensor_status_packet.steering_angle_status == 0 ? "VDY_OK"
                                        : sensor_status_packet.steering_angle_status == 1
                                          ? "VDY_NOTOK"
                                          : "INVALID VALUE";

  radar_status_.driving_direction_status =
    sensor_status_packet.driving_direction_status == 0   ? "VDY_OK"
    : sensor_status_packet.driving_direction_status == 1 ? "VDY_NOTOK"
                                                         : "INVALID VALUE";

  radar_status_.characteristic_speed_status =
    sensor_status_packet.characteristic_speed_status == 0   ? "VDY_OK"
    : sensor_status_packet.characteristic_speed_status == 1 ? "VDY_NOTOK"
                                                            : "INVALID VALUE";

  if (sensor_status_packet.radar_status == 0) {
    radar_status_.radar_status = "STATE_INIT";
  } else if (sensor_status_packet.radar_status == 1) {
    radar_status_.radar_status = "STATE_OK";
  } else if (sensor_status_packet.radar_status == 2) {
    radar_status_.radar_status = "STATE_INVALID";
  } else {
    radar_status_.radar_status = "INVALID VALUE";
  }

  if (sensor_status_packet.voltage_status == 0) {
    radar_status_.voltage_status = "Ok";
  }
  if (sensor_status_packet.voltage_status & 0x01) {
    radar_status_.voltage_status += "Current under voltage";
  }
  if (sensor_status_packet.voltage_status & 0x02) {
    radar_status_.voltage_status = "Past under voltage";
  }
  if (sensor_status_packet.voltage_status & 0x03) {
    radar_status_.voltage_status = "Current over voltage";
  }
  if (sensor_status_packet.voltage_status & 0x04) {
    radar_status_.voltage_status = "Past over voltage";
  }

  if (sensor_status_packet.temperature_status == 0) {
    radar_status_.temperature_status = "Ok";
  }
  if (sensor_status_packet.temperature_status & 0x01) {
    radar_status_.temperature_status += "Current under temperature";
  }
  if (sensor_status_packet.temperature_status & 0x02) {
    radar_status_.temperature_status += "Past under temperature";
  }
  if (sensor_status_packet.temperature_status & 0x03) {
    radar_status_.temperature_status += "Current over temperature";
  }
  if (sensor_status_packet.temperature_status & 0x04) {
    radar_status_.temperature_status += "Past over temperature";
  }

  const uint8_t & blockage_status0 = sensor_status_packet.blockage_status & 0x0f;
  const uint8_t & blockage_status1 = (sensor_status_packet.blockage_status & 0xf0) >> 4;

  if (blockage_status0 == 0) {
    radar_status_.blockage_status = "Blind";
  } else if (blockage_status0 == 1) {
    radar_status_.blockage_status = "High";
  } else if (blockage_status0 == 2) {
    radar_status_.blockage_status = "Mid";
  } else if (blockage_status0 == 3) {
    radar_status_.blockage_status = "Low";
  } else if (blockage_status0 == 4) {
    radar_status_.blockage_status = "None";
  } else {
    radar_status_.blockage_status = "INVALID VALUE";
  }

  if (blockage_status1 == 0) {
    radar_status_.blockage_status += ". Self test failed";
  } else if (blockage_status1 == 1) {
    radar_status_.blockage_status += ". Self test passed";
  } else if (blockage_status1 == 2) {
    radar_status_.blockage_status += ". Self test ongoing";
  } else {
    radar_status_.blockage_status += ". Invalid self test";
  }

  radar_status_.status_total_count++;
  radar_status_.radar_invalid_count += sensor_status_packet.radar_status == 2 ? 1 : 0;

  sensor_status_callback_(radar_status_);

  return true;
}

}  // namespace continental_ars548
}  // namespace drivers
}  // namespace nebula
