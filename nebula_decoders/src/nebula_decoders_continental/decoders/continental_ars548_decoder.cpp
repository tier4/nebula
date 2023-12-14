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

  if (data.size() < LENGTH_BYTE + sizeof(uint32_t)) {
    return false;
  }

  const uint16_t service_id =
    (static_cast<uint16_t>(data[SERVICE_ID_BYTE]) << 8) | data[SERVICE_ID_BYTE + 1];
  const uint16_t method_id =
    (static_cast<uint16_t>(data[METHOD_ID_BYTE]) << 8) | data[METHOD_ID_BYTE + 1];
  const uint32_t length = (static_cast<uint32_t>(data[LENGTH_BYTE]) << 24) |
                          (static_cast<uint32_t>(data[LENGTH_BYTE + 1]) << 16) |
                          (static_cast<uint32_t>(data[LENGTH_BYTE + 2]) << 8) |
                          static_cast<uint32_t>(data[LENGTH_BYTE + 3]);

  if (service_id != 0) {
    return false;
  }

  if (method_id == DETECTION_LIST_METHOD_ID) {
    if (data.size() != DETECTION_LIST_UDP_PAYLOAD || length != DETECTION_LIST_PDU_LENGTH) {
      return false;
    }

    return ParseDetectionsListPacket(data);
  } else if (method_id == OBJECT_LIST_METHOD_ID) {
    if (data.size() != OBJECT_LIST_UDP_PAYLOAD || length != OBJECT_LIST_PDU_LENGTH) {
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

  msg.header.frame_id = sensor_configuration_->frame_id;
  msg.header.stamp.nanosec =
    (static_cast<uint32_t>(data[DETECTION_LIST_TIMESTAMP_NANOSECONDS_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_TIMESTAMP_NANOSECONDS_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_TIMESTAMP_NANOSECONDS_BYTE + 2]) << 8) |
    data[DETECTION_LIST_TIMESTAMP_NANOSECONDS_BYTE + 3];
  msg.header.stamp.sec =
    (static_cast<uint32_t>(data[DETECTION_LIST_TIMESTAMP_SECONDS_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_TIMESTAMP_SECONDS_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_TIMESTAMP_SECONDS_BYTE + 2]) << 8) |
    data[DETECTION_LIST_TIMESTAMP_SECONDS_BYTE + 3];
  msg.stamp_sync_status = data[DETECTION_LIST_TIMESTAMP_SYNC_STATUS_BYTE];
  assert(msg.stamp_sync_status >= 1 && msg.stamp_sync_status <= 3);

  const uint32_t origin_pos_x_u =
    (static_cast<uint32_t>(data[DETECTION_LIST_ORIGIN_X_POS_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_ORIGIN_X_POS_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_ORIGIN_X_POS_BYTE + 2]) << 8) |
    data[DETECTION_LIST_ORIGIN_X_POS_BYTE + 3];
  const uint32_t origin_pos_y_u =
    (static_cast<uint32_t>(data[DETECTION_LIST_ORIGIN_Y_POS_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_ORIGIN_Y_POS_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_ORIGIN_Y_POS_BYTE + 2]) << 8) |
    data[DETECTION_LIST_ORIGIN_Y_POS_BYTE + 3];
  const uint32_t origin_pos_z_u =
    (static_cast<uint32_t>(data[DETECTION_LIST_ORIGIN_Z_POS_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_ORIGIN_Z_POS_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_ORIGIN_Z_POS_BYTE + 2]) << 8) |
    data[DETECTION_LIST_ORIGIN_Z_POS_BYTE + 3];

  float origin_pos_x_f, origin_pos_y_f, origin_pos_z_f;

  std::memcpy(&origin_pos_x_f, &origin_pos_x_u, sizeof(origin_pos_x_u));
  std::memcpy(&origin_pos_y_f, &origin_pos_y_u, sizeof(origin_pos_y_u));
  std::memcpy(&origin_pos_z_f, &origin_pos_z_u, sizeof(origin_pos_z_u));

  msg.origin_pos.x = static_cast<double>(origin_pos_x_f);
  msg.origin_pos.y = static_cast<double>(origin_pos_y_f);
  msg.origin_pos.z = static_cast<double>(origin_pos_z_f);

  const uint32_t origin_pitch_u =
    (static_cast<uint32_t>(data[DETECTION_LIST_PITCH_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_PITCH_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_PITCH_BYTE + 2]) << 8) |
    data[DETECTION_LIST_PITCH_BYTE + 3];
  const uint32_t origin_pitch_std_u =
    (static_cast<uint32_t>(data[DETECTION_LIST_PITCH_STD_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_PITCH_STD_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_PITCH_STD_BYTE + 2]) << 8) |
    data[DETECTION_LIST_PITCH_STD_BYTE + 3];
  const uint32_t origin_yaw_u = (static_cast<uint32_t>(data[DETECTION_LIST_YAW_BYTE]) << 24) |
                                (static_cast<uint32_t>(data[DETECTION_LIST_YAW_BYTE + 1]) << 16) |
                                (static_cast<uint32_t>(data[DETECTION_LIST_YAW_BYTE + 2]) << 8) |
                                data[DETECTION_LIST_YAW_BYTE + 3];
  const uint32_t origin_yaw_std_u =
    (static_cast<uint32_t>(data[DETECTION_LIST_YAW_STD_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_YAW_STD_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_YAW_STD_BYTE + 2]) << 8) |
    data[DETECTION_LIST_YAW_STD_BYTE + 3];

  std::memcpy(&msg.origin_pitch, &origin_pitch_u, sizeof(origin_pitch_u));
  std::memcpy(&msg.origin_pitch_std, &origin_pitch_std_u, sizeof(origin_pitch_std_u));
  std::memcpy(&msg.origin_yaw, &origin_yaw_u, sizeof(origin_yaw_u));
  std::memcpy(&msg.origin_yaw_std, &origin_yaw_std_u, sizeof(origin_yaw_std_u));

  const uint32_t ambiguity_free_velocity_min_u =
    (static_cast<uint32_t>(data[DETECTION_LIST_RAD_VEL_DOMAIN_MIN_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_RAD_VEL_DOMAIN_MIN_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_RAD_VEL_DOMAIN_MIN_BYTE + 2]) << 8) |
    data[DETECTION_LIST_RAD_VEL_DOMAIN_MIN_BYTE + 3];
  const uint32_t ambiguity_free_velocity_max_u =
    (static_cast<uint32_t>(data[DETECTION_LIST_RAD_VEL_DOMAIN_MAX_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_RAD_VEL_DOMAIN_MAX_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_RAD_VEL_DOMAIN_MAX_BYTE + 2]) << 8) |
    data[DETECTION_LIST_RAD_VEL_DOMAIN_MAX_BYTE + 3];

  std::memcpy(
    &msg.ambiguity_free_velocity_min, &ambiguity_free_velocity_min_u,
    sizeof(ambiguity_free_velocity_min_u));
  std::memcpy(
    &msg.ambiguity_free_velocity_max, &ambiguity_free_velocity_max_u,
    sizeof(ambiguity_free_velocity_max_u));

  const uint32_t alignment_azimuth_correction_u =
    (static_cast<uint32_t>(data[DETECTION_LIST_AZIMUTH_CORRECTION_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_AZIMUTH_CORRECTION_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_AZIMUTH_CORRECTION_BYTE + 2]) << 8) |
    data[DETECTION_LIST_AZIMUTH_CORRECTION_BYTE + 3];
  const uint32_t alignment_elevation_correction_u =
    (static_cast<uint32_t>(data[DETECTION_LIST_ELEVATION_CORRECTION_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_ELEVATION_CORRECTION_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_ELEVATION_CORRECTION_BYTE + 2]) << 8) |
    data[DETECTION_LIST_ELEVATION_CORRECTION_BYTE + 3];

  std::memcpy(
    &msg.alignment_azimuth_correction, &alignment_azimuth_correction_u,
    sizeof(alignment_azimuth_correction_u));
  std::memcpy(
    &msg.alignment_elevation_correction, &alignment_elevation_correction_u,
    sizeof(alignment_elevation_correction_u));

  msg.alignment_status = data[DETECTION_LIST_ALIGNMENT_STATUS_BYTE];

  const uint32_t numer_of_detections =
    (static_cast<uint32_t>(data[DETECTION_LIST_NUMBER_OF_DETECTIONS_BYTE]) << 24) |
    (static_cast<uint32_t>(data[DETECTION_LIST_NUMBER_OF_DETECTIONS_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[DETECTION_LIST_NUMBER_OF_DETECTIONS_BYTE + 2]) << 8) |
    data[DETECTION_LIST_NUMBER_OF_DETECTIONS_BYTE + 3];

  assert(origin_pos_x_f >= -10.f && origin_pos_x_f <= 10.f);
  assert(origin_pos_y_f >= -10.f && origin_pos_y_f <= 10.f);
  assert(origin_pos_z_f >= -10.f && origin_pos_z_f <= 10.f);
  assert(msg.origin_pitch >= -M_PI && msg.origin_pitch <= M_PI);
  assert(msg.origin_yaw >= -M_PI && msg.origin_yaw <= M_PI);
  assert(msg.ambiguity_free_velocity_min >= -100.f && msg.ambiguity_free_velocity_min <= 100.f);
  assert(msg.ambiguity_free_velocity_max >= -100.f && msg.ambiguity_free_velocity_max <= 100.f);
  assert(numer_of_detections <= 800);
  assert(msg.alignment_azimuth_correction >= -M_PI && msg.alignment_azimuth_correction <= M_PI);
  assert(msg.alignment_elevation_correction >= -M_PI && msg.alignment_elevation_correction <= M_PI);

  msg.detections.resize(numer_of_detections);

  for (std::size_t detection_index = 0; detection_index < numer_of_detections; detection_index++) {
    auto & detection_msg = msg.detections[detection_index];

    const int CURRENT_DETECTION_START_BYTE =
      DETECTION_LIST_ARRAY_BYTE + detection_index * DETECTION_STRUCT_SIZE;
    const int CURRENT_DETECTION_AZIMUTH_ANGLE_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_AZIMUTH_ANGLE_BYTE;
    const int CURRENT_DETECTION_AZIMUTH_ANGLE_STD_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_AZIMUTH_ANGLE_STD_BYTE;
    const int CURRENT_DETECTION_INVALID_FLAGS_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_INVALID_FLAGS_BYTE;
    const int CURRENT_DETECTION_ELEVATION_ANGLE_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_ELEVATION_ANGLE_BYTE;
    const int CURRENT_DETECTION_ELEVATION_ANGLE_STD_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_ELEVATION_ANGLE_STD_BYTE;
    const int CURRENT_DETECTION_RANGE_BYTE = CURRENT_DETECTION_START_BYTE + DETECTION_RANGE_BYTE;
    const int CURRENT_DETECTION_RANGE_STD_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_RANGE_STD_BYTE;
    const int CURRENT_DETECTION_RANGE_RATE_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_RANGE_RATE_BYTE;
    const int CURRENT_DETECTION_RANGE_RATE_STD_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_RANGE_RATE_STD_BYTE;
    const int CURRENT_DETECTION_RCS_BYTE = CURRENT_DETECTION_START_BYTE + DETECTION_RCS_BYTE;
    const int CURRENT_DETECTION_MEASUREMENT_ID_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_MEASUREMENT_ID_BYTE;
    const int CURRENT_DETECTION_POSITIVE_PREDICTIVE_VALUE_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_POSITIVE_PREDICTIVE_VALUE_BYTE;
    const int CURRENT_DETECTION_CLASSIFICATION_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_CLASSIFICATION_BYTE;
    const int CURRENT_DETECTION_MULT_TARGET_PROBABILITY_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_MULT_TARGET_PROBABILITY_BYTE;
    const int CURRENT_DETECTION_OBJECT_ID_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_OBJECT_ID_BYTE;
    const int CURRENT_DETECTION_AMBIGUITY_FLAG_BYTE =
      CURRENT_DETECTION_START_BYTE + DETECTION_AMBIGUITY_FLAG_BYTE;

    const uint32_t azimuth_angle_u =
      (static_cast<uint32_t>(data[CURRENT_DETECTION_AZIMUTH_ANGLE_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_AZIMUTH_ANGLE_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_AZIMUTH_ANGLE_BYTE + 2]) << 8) |
      data[CURRENT_DETECTION_AZIMUTH_ANGLE_BYTE + 3];
    const uint32_t azimuth_angle_std_u =
      (static_cast<uint32_t>(data[CURRENT_DETECTION_AZIMUTH_ANGLE_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_AZIMUTH_ANGLE_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_AZIMUTH_ANGLE_STD_BYTE + 2]) << 8) |
      data[CURRENT_DETECTION_AZIMUTH_ANGLE_STD_BYTE + 3];
    const uint8_t invalid_flags_u =
      (static_cast<uint32_t>(data[CURRENT_DETECTION_INVALID_FLAGS_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_INVALID_FLAGS_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_INVALID_FLAGS_BYTE + 2]) << 8) |
      data[CURRENT_DETECTION_INVALID_FLAGS_BYTE + 3];
    const uint32_t elevation_angle_u =
      (static_cast<uint32_t>(data[CURRENT_DETECTION_ELEVATION_ANGLE_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_ELEVATION_ANGLE_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_ELEVATION_ANGLE_BYTE + 2]) << 8) |
      data[CURRENT_DETECTION_ELEVATION_ANGLE_BYTE + 3];
    const uint32_t elevation_angle_std_u =
      (static_cast<uint32_t>(data[CURRENT_DETECTION_ELEVATION_ANGLE_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_ELEVATION_ANGLE_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_ELEVATION_ANGLE_STD_BYTE + 2]) << 8) |
      data[CURRENT_DETECTION_ELEVATION_ANGLE_STD_BYTE + 3];
    const uint32_t range_u = (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_BYTE]) << 24) |
                             (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_BYTE + 1]) << 16) |
                             (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_BYTE + 2]) << 8) |
                             data[CURRENT_DETECTION_RANGE_BYTE + 3];
    const uint32_t range_std_u =
      (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_STD_BYTE + 2]) << 8) |
      data[CURRENT_DETECTION_RANGE_STD_BYTE + 3];
    const uint32_t range_rate_u =
      (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_RATE_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_RATE_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_RATE_BYTE + 2]) << 8) |
      data[CURRENT_DETECTION_RANGE_RATE_BYTE + 3];
    const uint32_t range_rate_std_u =
      (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_RATE_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_RATE_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_DETECTION_RANGE_RATE_STD_BYTE + 2]) << 8) |
      data[CURRENT_DETECTION_RANGE_RATE_STD_BYTE + 3];
    const uint8_t rcs_u = data[CURRENT_DETECTION_RCS_BYTE];
    const uint16_t measurement_id_u =
      (static_cast<uint16_t>(data[CURRENT_DETECTION_MEASUREMENT_ID_BYTE]) << 8) |
      static_cast<uint16_t>(data[CURRENT_DETECTION_MEASUREMENT_ID_BYTE + 1]);
    const uint8_t positive_predictive_value_u =
      data[CURRENT_DETECTION_POSITIVE_PREDICTIVE_VALUE_BYTE];
    const uint8_t classification_u = data[CURRENT_DETECTION_CLASSIFICATION_BYTE];
    const uint8_t multi_target_probability_u = data[CURRENT_DETECTION_MULT_TARGET_PROBABILITY_BYTE];
    const uint16_t object_id_u =
      (static_cast<uint16_t>(data[CURRENT_DETECTION_OBJECT_ID_BYTE]) << 8) |
      static_cast<uint16_t>(data[CURRENT_DETECTION_OBJECT_ID_BYTE + 1]);
    const uint8_t ambiguity_flag_u = data[CURRENT_DETECTION_AMBIGUITY_FLAG_BYTE];

    assert(positive_predictive_value_u <= 100);
    assert(classification_u <= 4 || classification_u == 255);
    assert(multi_target_probability_u <= 100);
    assert(ambiguity_flag_u <= 100);

    detection_msg.invalid_distance = invalid_flags_u & 0x01;
    detection_msg.invalid_distance_std = invalid_flags_u & 0x01;
    detection_msg.invalid_azimuth = invalid_flags_u & 0x04;
    detection_msg.invalid_azimuth_std = invalid_flags_u & 0x08;
    detection_msg.invalid_elevation = invalid_flags_u & 0x10;
    detection_msg.invalid_elevation_std = invalid_flags_u & 0x20;
    detection_msg.invalid_range_rate = invalid_flags_u & 0x40;
    detection_msg.invalid_range_rate_std = invalid_flags_u & 0x01;
    detection_msg.rcs = static_cast<int8_t>(rcs_u);
    detection_msg.measurement_id = measurement_id_u;
    detection_msg.positive_predictive_value = positive_predictive_value_u;
    detection_msg.classification = classification_u;
    detection_msg.multi_target_probability = multi_target_probability_u;
    detection_msg.object_id = object_id_u;
    detection_msg.ambiguity_flag = ambiguity_flag_u;
    std::memcpy(&detection_msg.azimuth_angle, &azimuth_angle_u, sizeof(azimuth_angle_u));
    std::memcpy(
      &detection_msg.azimuth_angle_std, &azimuth_angle_std_u, sizeof(azimuth_angle_std_u));
    std::memcpy(&detection_msg.elevation_angle, &elevation_angle_u, sizeof(elevation_angle_u));
    std::memcpy(
      &detection_msg.elevation_angle_std, &elevation_angle_std_u, sizeof(elevation_angle_std_u));
    std::memcpy(&detection_msg.range, &range_u, sizeof(range_u));
    std::memcpy(&detection_msg.range_std, &range_std_u, sizeof(range_std_u));
    std::memcpy(&detection_msg.range_rate, &range_rate_u, sizeof(range_rate_u));
    std::memcpy(&detection_msg.range_rate_std, &range_rate_std_u, sizeof(range_rate_std_u));

    assert(detection_msg.azimuth_angle >= -M_PI && detection_msg.azimuth_angle <= M_PI);
    assert(detection_msg.azimuth_angle_std >= 0.f && detection_msg.azimuth_angle_std <= 1.f);
    assert(detection_msg.elevation_angle >= -M_PI && detection_msg.elevation_angle <= M_PI);
    assert(detection_msg.elevation_angle_std >= 0.f && detection_msg.elevation_angle_std <= 1.f);

    assert(detection_msg.range >= 0.f && detection_msg.range <= 1500.f);
    assert(detection_msg.range_std >= 0.f && detection_msg.range_std <= 1.f);
    assert(detection_msg.range_rate_std >= 0.f && detection_msg.range_rate_std <= 1.f);
  }

  detection_list_callback_(std::move(msg_ptr));

  return true;
}

bool ContinentalARS548Decoder::ParseObjectsListPacket(const std::vector<uint8_t> & data)
{
  auto msg_ptr = std::make_unique<continental_msgs::msg::ContinentalArs548ObjectList>();
  auto & msg = *msg_ptr;
  msg.header.frame_id = sensor_configuration_->frame_id;
  msg.header.stamp.nanosec =
    (static_cast<uint32_t>(data[OBJECT_LIST_TIMESTAMP_NANOSECONDS_BYTE]) << 24) |
    (static_cast<uint32_t>(data[OBJECT_LIST_TIMESTAMP_NANOSECONDS_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[OBJECT_LIST_TIMESTAMP_NANOSECONDS_BYTE + 2]) << 8) |
    data[OBJECT_LIST_TIMESTAMP_NANOSECONDS_BYTE + 3];
  msg.header.stamp.sec =
    (static_cast<uint32_t>(data[OBJECT_LIST_TIMESTAMP_SECONDS_BYTE]) << 24) |
    (static_cast<uint32_t>(data[OBJECT_LIST_TIMESTAMP_SECONDS_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(data[OBJECT_LIST_TIMESTAMP_SECONDS_BYTE + 2]) << 8) |
    data[OBJECT_LIST_TIMESTAMP_SECONDS_BYTE + 3];
  msg.stamp_sync_status = data[OBJECT_LIST_TIMESTAMP_SYNC_STATUS_BYTE];
  assert(msg.stamp_sync_status >= 1 && msg.stamp_sync_status <= 3);

  const uint8_t numer_of_objects = data[OBJECT_LIST_NUMBER_OF_OBJECTS_BYTE];

  msg.objects.resize(numer_of_objects);

  for (std::size_t object_index = 0; object_index < numer_of_objects; object_index++) {
    auto & object_msg = msg.objects[object_index];

    const int CURRENT_OBJECT_START_BYTE =
      OBJECT_LIST_ARRAY_BYTE + object_index * OBJECT_STRUCT_SIZE;
    const int CURRENT_OBJECT_ID_BYTE = CURRENT_OBJECT_START_BYTE + OBJECT_ID_BYTE;
    const int CURRENT_OBJECT_AGE_BYTE = CURRENT_OBJECT_START_BYTE + OBJECT_AGE_BYTE;

    const int CURRENT_OBJECT_STATUS_MEASUREMENT_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_STATUS_MEASUREMENT_BYTE;
    const int CURRENT_OBJECT_STATUS_MOVEMENT_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_STATUS_MOVEMENT_BYTE;
    const int CURRENT_OBJECT_POSITION_REFERENCE_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_POSITION_REFERENCE_BYTE;
    const int CURRENT_OBJECT_POSITION_X_BYTE = CURRENT_OBJECT_START_BYTE + OBJECT_POSITION_X_BYTE;
    const int CURRENT_OBJECT_POSITION_X_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_POSITION_X_STD_BYTE;
    const int CURRENT_OBJECT_POSITION_Y_BYTE = CURRENT_OBJECT_START_BYTE + OBJECT_POSITION_Y_BYTE;
    const int CURRENT_OBJECT_POSITION_Y_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_POSITION_Y_STD_BYTE;
    const int CURRENT_OBJECT_POSITION_Z_BYTE = CURRENT_OBJECT_START_BYTE + OBJECT_POSITION_Z_BYTE;
    const int CURRENT_OBJECT_POSITION_Z_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_POSITION_Z_STD_BYTE;

    const int CURRENT_OBJECT_POSITION_COVARIANCE_XY_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_POSITION_COVARIANCE_XY_BYTE;
    const int CURRENT_OBJECT_ORIENTATION_BYTE = CURRENT_OBJECT_START_BYTE + OBJECT_ORIENTATION_BYTE;
    const int CURRENT_OBJECT_ORIENTATION_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_ORIENTATION_STD_BYTE;

    const int CURRENT_OBJECT_EXISTENCE_PROBABILITY_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_EXISTENCE_PROBABILITY_BYTE;

    const int CURRENT_OBJECT_CLASSIFICATION_CAR_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_CLASSIFICATION_CAR_BYTE;
    const int CURRENT_OBJECT_CLASSIFICATION_TRUCK_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_CLASSIFICATION_TRUCK_BYTE;
    const int CURRENT_OBJECT_CLASSIFICATION_MOTORCYCLE_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_CLASSIFICATION_MOTORCYCLE_BYTE;
    const int CURRENT_OBJECT_CLASSIFICATION_BICYCLE_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_CLASSIFICATION_BICYCLE_BYTE;
    const int CURRENT_OBJECT_CLASSIFICATION_PEDESTRIAN_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_CLASSIFICATION_PEDESTRIAN_BYTE;
    const int CURRENT_OBJECT_CLASSIFICATION_ANIMAL_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_CLASSIFICATION_ANIMAL_BYTE;
    const int CURRENT_OBJECT_CLASSIFICATION_HAZARD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_CLASSIFICATION_HAZARD_BYTE;
    const int CURRENT_OBJECT_CLASSIFICATION_UNKNOWN_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_CLASSIFICATION_UNKNOWN_BYTE;

    const int CURRENT_OBJECT_DYNAMICS_ABS_VEL_X_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ABS_VEL_X_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_ABS_VEL_X_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ABS_VEL_X_STD_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_ABS_VEL_Y_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ABS_VEL_Y_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_ABS_VEL_Y_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ABS_VEL_Y_STD_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_ABS_VEL_COVARIANCE_XY_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ABS_VEL_COVARIANCE_XY_BYTE;

    const int CURRENT_OBJECT_DYNAMICS_REL_VEL_X_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_REL_VEL_X_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_REL_VEL_X_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_REL_VEL_X_STD_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_REL_VEL_Y_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_REL_VEL_Y_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_REL_VEL_Y_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_REL_VEL_Y_STD_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_REL_VEL_COVARIANCE_XY_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_REL_VEL_COVARIANCE_XY_BYTE;

    const int CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_X_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ABS_ACCEL_X_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_X_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ABS_ACCEL_X_STD_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_Y_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ABS_ACCEL_Y_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_Y_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ABS_ACCEL_Y_STD_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_COVARIANCE_XY_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ABS_ACCEL_COVARIANCE_XY_BYTE;

    const int CURRENT_OBJECT_DYNAMICS_REL_ACCEL_X_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_REL_ACCEL_X_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_REL_ACCEL_X_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_REL_ACCEL_X_STD_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_REL_ACCEL_Y_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_REL_ACCEL_Y_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_REL_ACCEL_Y_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_REL_ACCEL_Y_STD_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_REL_ACCEL_COVARIANCE_XY_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_REL_ACCEL_COVARIANCE_XY_BYTE;

    const int CURRENT_OBJECT_DYNAMICS_ORIENTATION_RATE_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ORIENTATION_RATE_BYTE;
    const int CURRENT_OBJECT_DYNAMICS_ORIENTATION_RATE_STD_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_DYNAMICS_ORIENTATION_RATE_STD_BYTE;

    const int CURRENT_OBJECT_SHAPE_LENGTH_EDGE_MEAN_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_SHAPE_LENGTH_EDGE_MEAN_BYTE;
    const int CURRENT_OBJECT_SHAPE_WIDTH_EDGE_MEAN_BYTE =
      CURRENT_OBJECT_START_BYTE + OBJECT_SHAPE_WIDTH_EDGE_MEAN_BYTE;

    const uint32_t object_id_u = (static_cast<uint32_t>(data[CURRENT_OBJECT_ID_BYTE]) << 24) |
                                 (static_cast<uint32_t>(data[CURRENT_OBJECT_ID_BYTE + 1]) << 16) |
                                 (static_cast<uint32_t>(data[CURRENT_OBJECT_ID_BYTE + 2]) << 8) |
                                 data[CURRENT_OBJECT_ID_BYTE + 3];
    const uint16_t object_age_u = (static_cast<uint16_t>(data[CURRENT_OBJECT_AGE_BYTE]) << 8) |
                                  static_cast<uint16_t>(data[CURRENT_OBJECT_AGE_BYTE + 1]);
    const uint8_t status_measurement_u = data[CURRENT_OBJECT_STATUS_MEASUREMENT_BYTE];
    const uint8_t status_movement_u = data[CURRENT_OBJECT_STATUS_MOVEMENT_BYTE];
    const uint8_t position_reference_u = data[CURRENT_OBJECT_POSITION_REFERENCE_BYTE];

    const uint32_t position_x_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_X_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_X_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_X_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_POSITION_X_BYTE + 3];
    const uint32_t position_x_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_X_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_X_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_X_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_POSITION_X_STD_BYTE + 3];
    const uint32_t position_y_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Y_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Y_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Y_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_POSITION_Y_BYTE + 3];
    const uint32_t position_y_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Y_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Y_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Y_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_POSITION_Y_STD_BYTE + 3];
    const uint32_t position_z_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Z_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Z_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Z_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_POSITION_Z_BYTE + 3];
    const uint32_t position_z_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Z_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Z_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_Z_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_POSITION_Z_STD_BYTE + 3];
    const uint32_t position_xy_covariance_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_COVARIANCE_XY_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_COVARIANCE_XY_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_POSITION_COVARIANCE_XY_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_POSITION_COVARIANCE_XY_BYTE + 3];
    const uint32_t orientation_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_ORIENTATION_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_ORIENTATION_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_ORIENTATION_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_ORIENTATION_BYTE + 3];
    const uint32_t orientation_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_ORIENTATION_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_ORIENTATION_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_ORIENTATION_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_ORIENTATION_STD_BYTE + 3];
    const uint32_t existence_probability_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_EXISTENCE_PROBABILITY_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_EXISTENCE_PROBABILITY_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_EXISTENCE_PROBABILITY_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_EXISTENCE_PROBABILITY_BYTE + 3];

    float position_x_f;
    float position_x_std_f;
    float position_y_f;
    float position_y_std_f;
    float position_z_f;
    float position_z_std_f;
    float position_xy_covariance_f;
    float orientation_f;
    float orientation_std_f;
    float existence_probability_f;

    std::memcpy(&position_x_f, &position_x_u, sizeof(position_x_u));
    std::memcpy(&position_x_std_f, &position_x_std_u, sizeof(position_x_std_u));
    std::memcpy(&position_y_f, &position_y_u, sizeof(position_y_u));
    std::memcpy(&position_y_std_f, &position_y_std_u, sizeof(position_y_std_u));
    std::memcpy(&position_z_f, &position_z_u, sizeof(position_z_u));
    std::memcpy(&position_z_std_f, &position_z_std_u, sizeof(position_z_std_u));
    std::memcpy(
      &position_xy_covariance_f, &position_xy_covariance_u, sizeof(position_xy_covariance_u));
    std::memcpy(&orientation_f, &orientation_u, sizeof(orientation_u));
    std::memcpy(&orientation_std_f, &orientation_std_u, sizeof(orientation_std_u));
    std::memcpy(
      &existence_probability_f, &existence_probability_u, sizeof(existence_probability_u));

    const uint8_t classification_car_u = data[CURRENT_OBJECT_CLASSIFICATION_CAR_BYTE];
    const uint8_t classification_truck_u = data[CURRENT_OBJECT_CLASSIFICATION_TRUCK_BYTE];
    const uint8_t classification_motorcycle_u = data[CURRENT_OBJECT_CLASSIFICATION_MOTORCYCLE_BYTE];
    const uint8_t classification_bicycle_u = data[CURRENT_OBJECT_CLASSIFICATION_BICYCLE_BYTE];
    const uint8_t classification_pedestrian_u = data[CURRENT_OBJECT_CLASSIFICATION_PEDESTRIAN_BYTE];
    const uint8_t classification_animal_u = data[CURRENT_OBJECT_CLASSIFICATION_ANIMAL_BYTE];
    const uint8_t classification_hazard_u = data[CURRENT_OBJECT_CLASSIFICATION_HAZARD_BYTE];
    const uint8_t classification_unknown_u = data[CURRENT_OBJECT_CLASSIFICATION_UNKNOWN_BYTE];

    const uint32_t dynamics_abs_vel_x_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_X_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_X_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_X_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_X_BYTE + 3];
    const uint32_t dynamics_abs_vel_x_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_X_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_X_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_X_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_X_STD_BYTE + 3];
    const uint32_t dynamics_abs_vel_y_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_Y_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_Y_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_Y_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_Y_BYTE + 3];
    const uint32_t dynamics_abs_vel_y_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_Y_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_Y_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_Y_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_Y_STD_BYTE + 3];
    const uint32_t dynamics_abs_vel_covariance_xy_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_COVARIANCE_XY_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_COVARIANCE_XY_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_COVARIANCE_XY_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ABS_VEL_COVARIANCE_XY_BYTE + 3];

    const uint32_t dynamics_rel_vel_x_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_X_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_X_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_X_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_REL_VEL_X_BYTE + 3];
    const uint32_t dynamics_rel_vel_x_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_X_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_X_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_X_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_REL_VEL_X_STD_BYTE + 3];
    const uint32_t dynamics_rel_vel_y_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_Y_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_Y_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_Y_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_REL_VEL_Y_BYTE + 3];
    const uint32_t dynamics_rel_vel_y_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_Y_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_Y_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_Y_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_REL_VEL_Y_STD_BYTE + 3];
    const uint32_t dynamics_rel_vel_covariance_xy_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_COVARIANCE_XY_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_COVARIANCE_XY_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_VEL_COVARIANCE_XY_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_REL_VEL_COVARIANCE_XY_BYTE + 3];

    const uint32_t dynamics_abs_accel_x_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_X_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_X_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_X_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_X_BYTE + 3];
    const uint32_t dynamics_abs_accel_x_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_X_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_X_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_X_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_X_STD_BYTE + 3];
    const uint32_t dynamics_abs_accel_y_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_Y_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_Y_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_Y_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_Y_BYTE + 3];
    const uint32_t dynamics_abs_accel_y_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_Y_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_Y_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_Y_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_Y_STD_BYTE + 3];
    const uint32_t dynamics_abs_accel_covariance_xy_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_COVARIANCE_XY_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_COVARIANCE_XY_BYTE + 1])
       << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_COVARIANCE_XY_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ABS_ACCEL_COVARIANCE_XY_BYTE + 3];

    const uint32_t dynamics_rel_accel_x_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_X_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_X_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_X_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_X_BYTE + 3];
    const uint32_t dynamics_rel_accel_x_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_X_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_X_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_X_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_X_STD_BYTE + 3];
    const uint32_t dynamics_rel_accel_y_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_Y_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_Y_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_Y_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_Y_BYTE + 3];
    ;
    const uint32_t dynamics_rel_accel_y_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_Y_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_Y_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_Y_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_Y_STD_BYTE + 3];
    const uint32_t dynamics_rel_accel_covariance_xy_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_COVARIANCE_XY_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_COVARIANCE_XY_BYTE + 1])
       << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_COVARIANCE_XY_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_REL_ACCEL_COVARIANCE_XY_BYTE + 3];

    float dynamics_abs_vel_x_f;
    float dynamics_abs_vel_x_std_f;
    float dynamics_abs_vel_y_f;
    float dynamics_abs_vel_y_std_f;
    float dynamics_abs_vel_covariance_xy_f;

    float dynamics_rel_vel_x_f;
    float dynamics_rel_vel_x_std_f;
    float dynamics_rel_vel_y_f;
    float dynamics_rel_vel_y_std_f;
    float dynamics_rel_vel_covariance_xy_f;

    float dynamics_abs_accel_x_f;
    float dynamics_abs_accel_x_std_f;
    float dynamics_abs_accel_y_f;
    float dynamics_abs_accel_y_std_f;
    float dynamics_abs_accel_covariance_xy_f;

    float dynamics_rel_accel_x_f;
    float dynamics_rel_accel_x_std_f;
    float dynamics_rel_accel_y_f;
    float dynamics_rel_accel_y_std_f;
    float dynamics_rel_accel_covariance_xy_f;

    std::memcpy(&dynamics_abs_vel_x_f, &dynamics_abs_vel_x_u, sizeof(dynamics_abs_vel_x_u));
    std::memcpy(
      &dynamics_abs_vel_x_std_f, &dynamics_abs_vel_x_std_u, sizeof(dynamics_abs_vel_x_std_u));
    std::memcpy(&dynamics_abs_vel_y_f, &dynamics_abs_vel_y_u, sizeof(dynamics_abs_vel_y_u));
    std::memcpy(
      &dynamics_abs_vel_y_std_f, &dynamics_abs_vel_y_std_u, sizeof(dynamics_abs_vel_y_std_u));
    std::memcpy(
      &dynamics_abs_vel_covariance_xy_f, &dynamics_abs_vel_covariance_xy_u,
      sizeof(dynamics_abs_vel_covariance_xy_u));

    std::memcpy(&dynamics_rel_vel_x_f, &dynamics_rel_vel_x_u, sizeof(dynamics_rel_vel_x_u));
    std::memcpy(
      &dynamics_rel_vel_x_std_f, &dynamics_rel_vel_x_std_u, sizeof(dynamics_rel_vel_x_std_u));
    std::memcpy(&dynamics_rel_vel_y_f, &dynamics_rel_vel_y_u, sizeof(dynamics_rel_vel_y_u));
    std::memcpy(
      &dynamics_rel_vel_y_std_f, &dynamics_rel_vel_y_std_u, sizeof(dynamics_rel_vel_y_std_u));
    std::memcpy(
      &dynamics_rel_vel_covariance_xy_f, &dynamics_rel_vel_covariance_xy_u,
      sizeof(dynamics_rel_vel_covariance_xy_u));

    std::memcpy(&dynamics_abs_accel_x_f, &dynamics_abs_accel_x_u, sizeof(dynamics_abs_accel_x_u));
    std::memcpy(
      &dynamics_abs_accel_x_std_f, &dynamics_abs_accel_x_std_u, sizeof(dynamics_abs_accel_x_std_u));
    std::memcpy(&dynamics_abs_accel_y_f, &dynamics_abs_accel_y_u, sizeof(dynamics_abs_accel_y_u));
    std::memcpy(
      &dynamics_abs_accel_y_std_f, &dynamics_abs_accel_y_std_u, sizeof(dynamics_abs_accel_y_std_u));
    std::memcpy(
      &dynamics_abs_accel_covariance_xy_f, &dynamics_abs_accel_covariance_xy_u,
      sizeof(dynamics_abs_accel_covariance_xy_u));

    std::memcpy(&dynamics_rel_accel_x_f, &dynamics_rel_accel_x_u, sizeof(dynamics_rel_accel_x_u));
    std::memcpy(
      &dynamics_rel_accel_x_std_f, &dynamics_rel_accel_x_std_u, sizeof(dynamics_rel_accel_x_std_u));
    std::memcpy(&dynamics_rel_accel_y_f, &dynamics_rel_accel_y_u, sizeof(dynamics_rel_accel_y_u));
    std::memcpy(
      &dynamics_rel_accel_y_std_f, &dynamics_rel_accel_y_std_u, sizeof(dynamics_rel_accel_y_std_u));
    std::memcpy(
      &dynamics_rel_accel_covariance_xy_f, &dynamics_rel_accel_covariance_xy_u,
      sizeof(dynamics_rel_accel_covariance_xy_u));

    const uint32_t dynamics_orientation_rate_mean_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ORIENTATION_RATE_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ORIENTATION_RATE_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ORIENTATION_RATE_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ORIENTATION_RATE_BYTE + 3];
    const uint32_t dynamics_orientation_rate_std_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ORIENTATION_RATE_STD_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ORIENTATION_RATE_STD_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_DYNAMICS_ORIENTATION_RATE_STD_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_DYNAMICS_ORIENTATION_RATE_STD_BYTE + 3];

    const uint32_t shape_length_edge_mean_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_SHAPE_LENGTH_EDGE_MEAN_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_SHAPE_LENGTH_EDGE_MEAN_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_SHAPE_LENGTH_EDGE_MEAN_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_SHAPE_LENGTH_EDGE_MEAN_BYTE + 3];
    const uint32_t shape_width_edge_mean_u =
      (static_cast<uint32_t>(data[CURRENT_OBJECT_SHAPE_WIDTH_EDGE_MEAN_BYTE]) << 24) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_SHAPE_WIDTH_EDGE_MEAN_BYTE + 1]) << 16) |
      (static_cast<uint32_t>(data[CURRENT_OBJECT_SHAPE_WIDTH_EDGE_MEAN_BYTE + 2]) << 8) |
      data[CURRENT_OBJECT_SHAPE_WIDTH_EDGE_MEAN_BYTE + 3];

    float dynamics_orientation_rate_mean_f;
    float dynamics_orientation_rate_std_f;

    float shape_length_edge_mean_f;
    float shape_width_edge_mean_f;

    std::memcpy(
      &dynamics_orientation_rate_mean_f, &dynamics_orientation_rate_mean_u,
      sizeof(dynamics_orientation_rate_mean_u));
    std::memcpy(
      &dynamics_orientation_rate_std_f, &dynamics_orientation_rate_std_u,
      sizeof(dynamics_orientation_rate_std_u));
    std::memcpy(
      &shape_length_edge_mean_f, &shape_length_edge_mean_u, sizeof(shape_length_edge_mean_u));
    std::memcpy(
      &shape_width_edge_mean_f, &shape_width_edge_mean_u, sizeof(shape_width_edge_mean_u));

    assert(status_measurement_u <= 2 || status_measurement_u == 255);
    assert(status_movement_u <= 1 || status_movement_u == 255);
    assert(position_reference_u <= 7 || position_reference_u == 255);

    assert(position_x_f >= -1600.f && position_x_f <= 1600.f);
    assert(position_x_std_f >= 0.f);
    assert(position_y_f >= -1600.f && position_y_f <= 1600.f);
    assert(position_y_std_f >= 0.f);
    assert(position_z_f >= -1600.f && position_z_f <= 1600.f);
    assert(position_z_std_f >= 0.f);
    assert(orientation_f >= -M_PI && orientation_f <= M_PI);
    assert(orientation_std_f >= 0.f);

    assert(classification_car_u <= 100);
    assert(classification_truck_u <= 100);
    assert(classification_motorcycle_u <= 100);
    assert(classification_bicycle_u <= 100);
    assert(classification_pedestrian_u <= 100);
    assert(classification_animal_u <= 100);
    assert(classification_hazard_u <= 100);
    assert(classification_unknown_u <= 100);

    assert(dynamics_abs_vel_x_std_f >= 0.f);
    assert(dynamics_abs_vel_y_std_f >= 0.f);

    assert(dynamics_rel_vel_x_std_f >= 0.f);
    assert(dynamics_rel_vel_y_std_f >= 0.f);

    assert(dynamics_abs_accel_x_std_f >= 0.f);
    assert(dynamics_abs_accel_y_std_f >= 0.f);

    assert(dynamics_rel_accel_x_std_f >= 0.f);
    assert(dynamics_rel_accel_y_std_f >= 0.f);

    object_msg.object_id = object_id_u;
    object_msg.age = object_age_u;
    object_msg.status_measurement = status_measurement_u;
    object_msg.status_movement = status_movement_u;
    object_msg.position_reference = position_reference_u;

    object_msg.position.x = static_cast<double>(position_x_f);
    object_msg.position.y = static_cast<double>(position_y_f);
    object_msg.position.y = static_cast<double>(position_z_f);

    object_msg.position_std.x = static_cast<double>(position_x_std_f);
    object_msg.position_std.y = static_cast<double>(position_y_std_f);
    object_msg.position_std.z = static_cast<double>(position_z_std_f);

    object_msg.position_covariance_xy = position_xy_covariance_f;

    object_msg.orientation = orientation_f;
    object_msg.orientation_std = orientation_std_f;

    object_msg.existence_probability = existence_probability_f;
    object_msg.classification_car = classification_car_u;
    object_msg.classification_truck = classification_truck_u;
    object_msg.classification_motorcycle = classification_motorcycle_u;
    object_msg.classification_bicycle = classification_bicycle_u;
    object_msg.classification_pedestrian = classification_pedestrian_u;
    object_msg.classification_animal = classification_animal_u;
    object_msg.classification_hazard = classification_hazard_u;
    object_msg.classification_unknown = classification_unknown_u;

    object_msg.absolute_velocity.x = static_cast<double>(dynamics_abs_vel_x_f);
    object_msg.absolute_velocity.y = static_cast<double>(dynamics_abs_vel_y_f);
    object_msg.absolute_velocity_std.x = static_cast<double>(dynamics_abs_vel_x_std_f);
    object_msg.absolute_velocity_std.y = static_cast<double>(dynamics_abs_vel_y_std_f);
    object_msg.absolute_velocity_covariance_xy = dynamics_abs_vel_covariance_xy_f;

    object_msg.relative_velocity.x = static_cast<double>(dynamics_rel_vel_x_f);
    object_msg.relative_velocity.y = static_cast<double>(dynamics_rel_vel_y_f);
    object_msg.relative_velocity_std.x = static_cast<double>(dynamics_rel_vel_x_std_f);
    object_msg.relative_velocity_std.y = static_cast<double>(dynamics_rel_vel_y_std_f);
    object_msg.relative_velocity_covariance_xy = dynamics_rel_vel_covariance_xy_f;

    object_msg.absolute_acceleration.x = static_cast<double>(dynamics_abs_accel_x_f);
    object_msg.absolute_acceleration.y = static_cast<double>(dynamics_abs_accel_y_f);
    object_msg.absolute_acceleration_std.x = static_cast<double>(dynamics_abs_accel_x_std_f);
    object_msg.absolute_acceleration_std.y = static_cast<double>(dynamics_abs_accel_y_std_f);
    object_msg.absolute_acceleration_covariance_xy = dynamics_abs_accel_covariance_xy_f;

    object_msg.relative_velocity.x = dynamics_rel_accel_x_f;
    object_msg.relative_velocity.y = dynamics_rel_accel_y_f;
    object_msg.relative_velocity_std.x = dynamics_rel_accel_x_std_f;
    object_msg.relative_velocity_std.y = dynamics_rel_accel_y_std_f;
    object_msg.relative_velocity_covariance_xy = dynamics_rel_accel_covariance_xy_f;

    object_msg.orientation_rate_mean = dynamics_orientation_rate_mean_f;
    object_msg.orientation_rate_std = dynamics_orientation_rate_std_f;

    object_msg.shape_length_edge_mean = shape_length_edge_mean_f;
    object_msg.shape_width_edge_mean = shape_width_edge_mean_f;
  }

  object_list_callback_(std::move(msg_ptr));

  return true;
}

}  // namespace continental_ars548
}  // namespace drivers
}  // namespace nebula
