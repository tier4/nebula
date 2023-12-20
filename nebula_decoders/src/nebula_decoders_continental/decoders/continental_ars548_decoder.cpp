// Copyright 2023 Tier IV, Inc.
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
#include "nebula_common/continental/continental_common.hpp"

#include <cmath>
#include <utility>

namespace nebula
{
namespace drivers
{
namespace continental_ars548
{
ContinentalARS548Decoder::ContinentalARS548Decoder(
  const std::shared_ptr<drivers::ContinentalARS548SensorConfiguration> & sensor_configuration)
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

    return ParseDetectionsListPacket(data);
  } else if (header.method_id.value() == OBJECT_LIST_METHOD_ID) {
    if (data.size() != OBJECT_LIST_UDP_PAYLOAD || header.length.value() != OBJECT_LIST_PDU_LENGTH) {
      return false;
    }

    return ParseObjectsListPacket(data);
  }

  return true;
}

bool ContinentalARS548Decoder::ParseDetectionsListPacket(const std::vector<uint8_t> & data)
{
  auto msg_ptr = std::make_unique<continental_msgs::msg::ContinentalArs548DetectionList>();
  auto & msg = *msg_ptr;

  DetectionListPacket detection_list;
  assert(sizeof(DetectionListPacket) == data.size());

  std::memcpy(&detection_list, data.data(), sizeof(DetectionListPacket));

  msg.header.frame_id = sensor_configuration_->frame_id;
  msg.header.stamp.nanosec = detection_list.stamp.timestamp_nanoseconds.value();
  msg.header.stamp.sec = detection_list.stamp.timestamp_seconds.value();
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

bool ContinentalARS548Decoder::ParseObjectsListPacket(const std::vector<uint8_t> & data)
{
  auto msg_ptr = std::make_unique<continental_msgs::msg::ContinentalArs548ObjectList>();
  auto & msg = *msg_ptr;

  ObjectListPacket object_list;
  assert(sizeof(ObjectListPacket) == data.size());

  std::memcpy(&object_list, data.data(), sizeof(object_list));

  msg.header.frame_id = sensor_configuration_->frame_id;
  msg.header.stamp.nanosec = object_list.stamp.timestamp_nanoseconds.value();
  msg.header.stamp.sec = object_list.stamp.timestamp_seconds.value();
  msg.stamp_sync_status = object_list.stamp.timestamp_sync_status;
  assert(msg.stamp_sync_status >= 1 && msg.stamp_sync_status <= 3);

  const uint8_t number_of_objects = object_list.number_of_objects;

  msg.objects.resize(number_of_objects);

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
    object_msg.position.y = static_cast<double>(object.position_z.value());

    object_msg.position_std.x = static_cast<double>(object.position_x_std.value());
    object_msg.position_std.y = static_cast<double>(object.position_y_std.value());
    object_msg.position_std.z = static_cast<double>(object.position_z_std.value());

    object_msg.position_covariance_xy = object.position_covariance_xy.value();

    object_msg.orientation = object.position_orientation.value();
    object_msg.orientation_std = object.position_orientation_std.value();

    object_msg.existence_probability = object.existance_probability.value();
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

}  // namespace continental_ars548
}  // namespace drivers
}  // namespace nebula
