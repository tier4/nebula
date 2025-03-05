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

#include "nebula_decoders/nebula_decoders_continental/decoders/continental_srr520_decoder.hpp"

#include <nebula_common/continental/continental_srr520.hpp>
#include <nebula_common/continental/crc.hpp>

#include <boost/algorithm/string/join.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers::continental_srr520
{
ContinentalSRR520Decoder::ContinentalSRR520Decoder(
  const std::shared_ptr<const continental_srr520::ContinentalSRR520SensorConfiguration> &
    sensor_configuration)
{
  sensor_configuration_ = sensor_configuration;

  rdi_near_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  near_detection_list_ptr_ =
    std::make_unique<continental_msgs::msg::ContinentalSrr520DetectionList>();

  rdi_hrr_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  hrr_detection_list_ptr_ =
    std::make_unique<continental_msgs::msg::ContinentalSrr520DetectionList>();

  object_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  object_list_ptr_ = std::make_unique<continental_msgs::msg::ContinentalSrr520ObjectList>();
}

Status ContinentalSRR520Decoder::get_status()
{
  return Status::OK;
}

Status ContinentalSRR520Decoder::register_near_detection_list_callback(
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList>)>
    detection_list_callback)
{
  near_detection_list_callback_ = std::move(detection_list_callback);
  return Status::OK;
}

Status ContinentalSRR520Decoder::register_hrr_detection_list_callback(
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList>)>
    detection_list_callback)
{
  hrr_detection_list_callback_ = std::move(detection_list_callback);
  return Status::OK;
}

Status ContinentalSRR520Decoder::register_object_list_callback(
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520ObjectList>)>
    object_list_callback)
{
  object_list_callback_ = std::move(object_list_callback);
  return Status::OK;
}

Status ContinentalSRR520Decoder::register_status_callback(
  std::function<void(std::unique_ptr<diagnostic_msgs::msg::DiagnosticArray>)> status_callback)
{
  status_callback_ = std::move(status_callback);
  return Status::OK;
}

Status ContinentalSRR520Decoder::register_sync_follow_up_callback(
  std::function<void(builtin_interfaces::msg::Time)> sync_follow_up_callback)
{
  sync_follow_up_callback_ = std::move(sync_follow_up_callback);
  return Status::OK;
}

Status ContinentalSRR520Decoder::register_packets_callback(
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets>)> nebula_packets_callback)
{
  nebula_packets_callback_ = std::move(nebula_packets_callback);
  return Status::OK;
}

bool ContinentalSRR520Decoder::process_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  const uint32_t can_message_id = (static_cast<uint32_t>(packet_msg->data[0]) << 24) |
                                  (static_cast<uint32_t>(packet_msg->data[1]) << 16) |
                                  (static_cast<uint32_t>(packet_msg->data[2]) << 8) |
                                  static_cast<uint32_t>(packet_msg->data[3]);

  std::size_t payload_size = packet_msg->data.size() - 4;

  if (can_message_id == rdi_near_header_can_message_id) {
    if (payload_size != rdi_near_header_packet_size) {
      print_error("rdi_near_header_can_message_id message with invalid size");
      return false;
    }
    process_near_header_packet(std::move(packet_msg));
  } else if (can_message_id == rdi_near_element_can_message_id) {
    if (payload_size != rdi_near_element_packet_size) {
      print_error("rdi_near_element_can_message_id message with invalid size");
      return false;
    }

    process_near_element_packet(std::move(packet_msg));
  } else if (can_message_id == rdi_hrr_header_can_message_id) {
    if (payload_size != rdi_hrr_header_packet_size) {
      print_error("rdi_hrr_header_can_message_id message with invalid size");
      return false;
    }
    process_hrr_header_packet(std::move(packet_msg));
  } else if (can_message_id == rdi_hrr_element_can_message_id) {
    if (payload_size != rdi_hrr_element_packet_size) {
      print_error("rdi_hrr_element_can_message_id message with invalid size");
      return false;
    }

    process_hrr_element_packet(std::move(packet_msg));
  } else if (can_message_id == object_header_can_message_id) {
    if (payload_size != object_header_packet_size) {
      print_error("object_header_can_message_id message with invalid size");
      return false;
    }
    process_object_header_packet(std::move(packet_msg));
  } else if (can_message_id == object_can_message_id) {
    if (payload_size != object_packet_size) {
      print_error("object_element_can_message_id message with invalid size");
      return false;
    }

    process_object_element_packet(std::move(packet_msg));
  } else if (can_message_id == crc_list_can_message_id) {
    if (payload_size != crc_list_packet_size) {
      print_error("crc_list_can_message_id message with invalid size");
      return false;
    }

    process_crc_list_packet(std::move(packet_msg));
  } else if (can_message_id == status_can_message_id) {
    if (payload_size != status_packet_size) {
      print_error("crc_list_can_message_id message with invalid size");
      return false;
    }

    process_sensor_status_packet(std::move(packet_msg));
  } else if (can_message_id == sync_follow_up_can_message_id) {
    if (payload_size != sync_follow_up_can_packet_size) {
      print_error("sync_follow_up_can_message_id message with invalid size");
      return false;
    }

    process_sync_follow_up_packet(std::move(packet_msg));
  } else if (
    can_message_id != veh_dyn_can_message_id && can_message_id != sensor_config_can_message_id) {
    print_error("Unrecognized message ID=" + std::to_string(can_message_id));
    return false;
  }

  return true;
}

void ContinentalSRR520Decoder::process_near_header_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  constexpr float v_ambiguous_resolution = 0.003051851f;
  constexpr float v_ambiguous_min_value = -100.f;
  constexpr float max_range_resolution = 0.1f;

  first_rdi_near_packet_ = false;

  static_assert(sizeof(ScanHeaderPacket) == rdi_near_header_packet_size);
  static_assert(sizeof(DetectionPacket) == rdi_near_element_packet_size);
  assert(packet_msg->data.size() == rdi_near_header_packet_size + 4);

  std::memcpy(
    &rdi_near_header_packet_, packet_msg->data.data() + 4 * sizeof(uint8_t),
    sizeof(ScanHeaderPacket));

  assert(
    rdi_near_header_packet_.u_global_time_stamp_sync_status >= 1 &&
    rdi_near_header_packet_.u_global_time_stamp_sync_status <= 3);

  near_detection_list_ptr_->header.frame_id = sensor_configuration_->frame_id;

  if (rdi_near_header_packet_.u_global_time_stamp_sync_status == 1) {
    near_detection_list_ptr_->header.stamp.sec =
      rdi_near_header_packet_.u_global_time_stamp_sec.value();
    near_detection_list_ptr_->header.stamp.nanosec =
      rdi_near_header_packet_.u_global_time_stamp_nsec.value();
  } else {
    near_detection_list_ptr_->header.stamp = packet_msg->stamp;
  }

  rdi_near_packets_ptr_->header.stamp = packet_msg->stamp;
  rdi_near_packets_ptr_->header.frame_id = sensor_configuration_->frame_id;

  near_detection_list_ptr_->internal_time_stamp_usec = rdi_near_header_packet_.u_time_stamp.value();
  near_detection_list_ptr_->global_time_stamp_sync_status =
    rdi_near_header_packet_.u_global_time_stamp_sync_status;
  near_detection_list_ptr_->signal_status = rdi_near_header_packet_.u_signal_status;
  near_detection_list_ptr_->sequence_counter = rdi_near_header_packet_.u_sequence_counter;
  near_detection_list_ptr_->cycle_counter = rdi_near_header_packet_.u_cycle_counter.value();
  near_detection_list_ptr_->v_ambiguous =
    v_ambiguous_resolution * rdi_near_header_packet_.u_v_ambiguous.value() + v_ambiguous_min_value;
  near_detection_list_ptr_->max_range =
    max_range_resolution * rdi_near_header_packet_.u_max_range.value();

  near_detection_list_ptr_->detections.reserve(
    rdi_near_header_packet_.u_number_of_detections.value());

  rdi_near_packets_ptr_->packets.emplace_back(std::move(*packet_msg));
}

void ContinentalSRR520Decoder::process_near_element_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  constexpr auto range_resolution = 0.024420024;
  constexpr auto azimuth_resolution = 0.006159986;
  constexpr auto range_rate_resolution = 0.014662757;
  constexpr auto rcs_resolution = 0.476190476;
  constexpr auto snr_resolution = 1.7;

  constexpr auto azimuth_min_value = -1.570796327;
  constexpr auto range_rate_min_value = -15.f;
  constexpr auto rcs_min_value = -40.f;
  constexpr auto snr_min_value = 11.f;

  if (rdi_near_packets_ptr_->packets.size() == 0) {
    if (!first_rdi_near_packet_) {
      print_error("Near element before header. This can happen during the first iteration");
    }
    return;
  }

  int parsed_detections = near_detection_list_ptr_->detections.size();

  if (
    near_detection_list_ptr_->detections.size() >=
    rdi_near_header_packet_.u_number_of_detections.value()) {
    rdi_near_packets_ptr_->packets.emplace_back(std::move(*packet_msg));
    return;
  }

  DetectionPacket detection_packet;
  std::memcpy(
    &detection_packet, packet_msg->data.data() + 4 * sizeof(uint8_t), sizeof(DetectionPacket));

  static_assert(sizeof(DetectionPacket) == rdi_near_element_packet_size);
  assert(packet_msg->data.size() == rdi_near_element_packet_size + 4);
  assert(rdi_near_header_packet_.u_sequence_counter == detection_packet.u_sequence_counter);
  assert(
    rdi_near_packets_ptr_->packets.size() ==
    static_cast<std::size_t>(detection_packet.u_message_counter + 1));

  for (const auto & fragment : detection_packet.fragments) {
    continental_msgs::msg::ContinentalSrr520Detection detection_msg;
    const auto & data = fragment.data;

    if (parsed_detections >= rdi_near_header_packet_.u_number_of_detections.value()) {
      break;
    }

    uint16_t u_range =
      (static_cast<uint16_t>(data[0]) << 4) | (static_cast<uint16_t>(data[1] & 0xF0) >> 4);
    assert(u_range <= 4095);
    detection_msg.range = range_resolution * u_range;

    uint16_t u_azimuth =
      (static_cast<uint16_t>(data[1] & 0x0f) << 5) | (static_cast<uint16_t>(data[2] & 0xF8) >> 3);
    assert(u_azimuth <= 510);
    detection_msg.azimuth_angle = azimuth_resolution * u_azimuth + azimuth_min_value;

    uint16_t u_range_rate =
      (static_cast<uint16_t>(data[2] & 0x07) << 8) | static_cast<uint16_t>(data[3]);
    assert(u_range_rate <= 2046);
    detection_msg.range_rate = range_rate_resolution * u_range_rate + range_rate_min_value;

    uint16_t u_rcs = (data[4] & 0xFE) >> 1;
    assert(u_rcs <= 126);
    detection_msg.rcs = rcs_resolution * u_rcs + rcs_min_value;

    detection_msg.pdh00 = 100 * (data[4] & 0x01);
    detection_msg.pdh01 = 100 * ((data[5] & 0x80) >> 7);
    detection_msg.pdh02 = 100 * ((data[5] & 0x40) >> 6);
    detection_msg.pdh03 = 100 * ((data[5] & 0x20) >> 5);
    detection_msg.pdh04 = 100 * ((data[5] & 0x10) >> 4);

    uint8_t u_snr = data[5] & 0x0f;
    detection_msg.snr = snr_resolution * u_snr + snr_min_value;

    near_detection_list_ptr_->detections.push_back(detection_msg);
    parsed_detections++;
  }

  rdi_near_packets_ptr_->packets.emplace_back(std::move(*packet_msg));
}

void ContinentalSRR520Decoder::process_hrr_header_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  constexpr float V_AMBIGUOUS_RESOLUTION = 0.003051851f;
  constexpr float V_AMBIGUOUS_MIN_VALUE = -100.f;
  constexpr float MAX_RANGE_RESOLUTION = 0.1f;

  first_rdi_hrr_packet_ = false;

  static_assert(sizeof(ScanHeaderPacket) == rdi_hrr_header_packet_size);
  assert(packet_msg->data.size() == rdi_hrr_header_packet_size + 4);

  std::memcpy(
    &rdi_hrr_header_packet_, packet_msg->data.data() + 4 * sizeof(uint8_t),
    sizeof(ScanHeaderPacket));

  assert(
    rdi_hrr_header_packet_.u_global_time_stamp_sync_status >= 1 &&
    rdi_hrr_header_packet_.u_global_time_stamp_sync_status <= 3);

  hrr_detection_list_ptr_->header.frame_id = sensor_configuration_->frame_id;

  if (rdi_hrr_header_packet_.u_global_time_stamp_sync_status == 1) {
    hrr_detection_list_ptr_->header.stamp.sec =
      rdi_hrr_header_packet_.u_global_time_stamp_sec.value();
    hrr_detection_list_ptr_->header.stamp.nanosec =
      rdi_hrr_header_packet_.u_global_time_stamp_nsec.value();
  } else {
    hrr_detection_list_ptr_->header.stamp = packet_msg->stamp;
  }

  rdi_hrr_packets_ptr_->header.stamp = packet_msg->stamp;
  rdi_hrr_packets_ptr_->header.frame_id = sensor_configuration_->frame_id;

  hrr_detection_list_ptr_->internal_time_stamp_usec = rdi_hrr_header_packet_.u_time_stamp.value();
  hrr_detection_list_ptr_->global_time_stamp_sync_status =
    rdi_hrr_header_packet_.u_global_time_stamp_sync_status;
  hrr_detection_list_ptr_->signal_status = rdi_hrr_header_packet_.u_signal_status;
  hrr_detection_list_ptr_->sequence_counter = rdi_hrr_header_packet_.u_sequence_counter;
  hrr_detection_list_ptr_->cycle_counter = rdi_hrr_header_packet_.u_cycle_counter.value();
  hrr_detection_list_ptr_->v_ambiguous =
    V_AMBIGUOUS_RESOLUTION * rdi_hrr_header_packet_.u_v_ambiguous.value() + V_AMBIGUOUS_MIN_VALUE;
  hrr_detection_list_ptr_->max_range =
    MAX_RANGE_RESOLUTION * rdi_hrr_header_packet_.u_max_range.value();

  hrr_detection_list_ptr_->detections.reserve(
    rdi_hrr_header_packet_.u_number_of_detections.value());

  rdi_hrr_packets_ptr_->packets.emplace_back(std::move(*packet_msg));
}

void ContinentalSRR520Decoder::process_hrr_element_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  constexpr auto RANGE_RESOLUTION = 0.024420024;
  constexpr auto AZIMUTH_RESOLUTION = 0.006159986;
  constexpr auto RANGE_RATE_RESOLUTION = 0.014662757;
  constexpr auto RCS_RESOLUTION = 0.476190476;
  constexpr auto SNR_RESOLUTION = 1.7f;

  constexpr auto AZIMUTH_MIN_VALUE = -1.570796327;
  constexpr auto RANGE_RATE_MIN_VALUE = -15.f;
  constexpr auto RCS_MIN_VALUE = -40.f;
  constexpr auto SNR_MIN_VALUE = 11.f;

  if (rdi_hrr_packets_ptr_->packets.size() == 0) {
    if (!first_rdi_hrr_packet_) {
      print_error("HRR element before header. This can happen during the first iteration");
    }
    return;
  }

  int parsed_detections = hrr_detection_list_ptr_->detections.size();

  if (
    hrr_detection_list_ptr_->detections.size() >=
    rdi_hrr_header_packet_.u_number_of_detections.value()) {
    rdi_hrr_packets_ptr_->packets.emplace_back(std::move(*packet_msg));
    return;
  }

  DetectionPacket detection_packet;
  std::memcpy(
    &detection_packet, packet_msg->data.data() + 4 * sizeof(uint8_t), sizeof(DetectionPacket));

  static_assert(sizeof(DetectionPacket) == rdi_hrr_element_packet_size);
  assert(packet_msg->data.size() == rdi_hrr_element_packet_size + 4);
  assert(rdi_hrr_header_packet_.u_sequence_counter == detection_packet.u_sequence_counter);
  assert(
    rdi_hrr_packets_ptr_->packets.size() ==
    static_cast<std::size_t>(detection_packet.u_message_counter + 1));

  for (const auto & fragment : detection_packet.fragments) {
    continental_msgs::msg::ContinentalSrr520Detection detection_msg;
    const auto & data = fragment.data;

    if (parsed_detections >= rdi_hrr_header_packet_.u_number_of_detections.value()) {
      break;
    }

    uint16_t u_range =
      (static_cast<uint16_t>(data[0]) << 4) | (static_cast<uint16_t>(data[1] & 0xF0) >> 4);
    assert(u_range <= 4095);
    detection_msg.range = RANGE_RESOLUTION * u_range;

    uint16_t u_azimuth =
      (static_cast<uint16_t>(data[1] & 0x0f) << 5) | (static_cast<uint16_t>(data[2] & 0xF8) >> 3);
    assert(u_azimuth <= 510);
    detection_msg.azimuth_angle = AZIMUTH_RESOLUTION * u_azimuth + AZIMUTH_MIN_VALUE;

    uint16_t u_range_rate =
      (static_cast<uint16_t>(data[2] & 0x07) << 8) | static_cast<uint16_t>(data[3]);
    assert(u_range_rate <= 2046);
    detection_msg.range_rate = RANGE_RATE_RESOLUTION * u_range_rate + RANGE_RATE_MIN_VALUE;

    uint16_t u_rcs = (data[4] & 0xFE) >> 1;
    assert(u_rcs <= 126);
    detection_msg.rcs = RCS_RESOLUTION * u_rcs + RCS_MIN_VALUE;

    detection_msg.pdh00 = 100 * (data[4] & 0x01);
    detection_msg.pdh01 = 100 * ((data[5] & 0x80) >> 7);
    detection_msg.pdh02 = 100 * ((data[5] & 0x40) >> 6);
    detection_msg.pdh03 = 100 * ((data[5] & 0x20) >> 5);
    detection_msg.pdh04 = 100 * ((data[5] & 0x10) >> 4);

    uint8_t u_snr = data[5] & 0x0f;
    detection_msg.snr = SNR_RESOLUTION * u_snr + SNR_MIN_VALUE;

    hrr_detection_list_ptr_->detections.push_back(detection_msg);
    parsed_detections++;
  }

  rdi_hrr_packets_ptr_->packets.emplace_back(std::move(*packet_msg));
}

void ContinentalSRR520Decoder::process_object_header_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  constexpr auto VX_RESOLUTION = 0.003051851;
  constexpr auto VX_MIN_VALUE = -100.f;
  constexpr auto YAW_RATE_RESOLUTION = 9.58766e-05;
  constexpr auto YAW_RATE_MIN_VALUE = -3.14159;

  first_object_packet_ = false;

  static_assert(sizeof(ObjectHeaderPacket) == object_header_packet_size);
  assert(packet_msg->data.size() == object_header_packet_size + 4);

  std::memcpy(
    &object_header_packet_, packet_msg->data.data() + 4 * sizeof(uint8_t),
    sizeof(ObjectHeaderPacket));

  assert(
    object_header_packet_.u_global_time_stamp_sync_status >= 1 &&
    object_header_packet_.u_global_time_stamp_sync_status <= 3);

  object_list_ptr_->header.frame_id = sensor_configuration_->base_frame;

  if (object_header_packet_.u_global_time_stamp_sync_status == 1) {
    object_list_ptr_->header.stamp.sec = object_header_packet_.u_global_time_stamp_sec.value();
    object_list_ptr_->header.stamp.nanosec = object_header_packet_.u_global_time_stamp_nsec.value();
  } else {
    object_list_ptr_->header.stamp = packet_msg->stamp;
  }

  object_packets_ptr_->header.stamp = packet_msg->stamp;
  object_packets_ptr_->header.frame_id = sensor_configuration_->base_frame;

  object_list_ptr_->internal_time_stamp_usec = object_header_packet_.u_time_stamp.value();
  object_list_ptr_->global_time_stamp_sync_status =
    object_header_packet_.u_global_time_stamp_sync_status;
  object_list_ptr_->signal_status = object_header_packet_.u_signal_status;
  object_list_ptr_->sequence_counter = object_header_packet_.u_sequence_counter;
  object_list_ptr_->cycle_counter = object_header_packet_.u_cycle_counter.value();
  object_list_ptr_->ego_vx = VX_RESOLUTION * object_header_packet_.u_ego_vx.value() + VX_MIN_VALUE;
  object_list_ptr_->ego_yaw_rate =
    YAW_RATE_RESOLUTION * object_header_packet_.u_ego_yaw_rate.value() + YAW_RATE_MIN_VALUE;
  object_list_ptr_->motion_type = object_header_packet_.u_motion_type;

  object_list_ptr_->objects.reserve(object_header_packet_.u_number_of_objects);

  object_packets_ptr_->packets.emplace_back(std::move(*packet_msg));
}

void ContinentalSRR520Decoder::process_object_element_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  constexpr auto DIST_RESOLUTION = 0.009155553;
  constexpr auto V_ABS_RESOLUTION = 0.009156391;
  constexpr auto A_ABS_RESOLUTION = 0.019569472;
  constexpr auto DIST_STD_RESOLUTION = 0.001831166;
  constexpr auto V_ABS_STD_RESOLUTION = 0.001831166;
  constexpr auto A_ABS_STD_RESOLUTION = 0.001831166;
  constexpr auto OBJECT_BOX_RESOLUTION = 0.007326007;
  constexpr auto OBJECT_ORIENTATION_RESOLUTION = 0.001534729;
  constexpr auto OBJECT_RCS_RESOLUTION = 0.024425989;
  constexpr auto OBJECT_SCORE_RESOLUTION = 6.666666667;

  constexpr auto DIST_MIN_VALUE = -300.f;
  constexpr auto V_ABS_MIN_VALUE = -75.f;
  constexpr auto A_ABS_MIN_VALUE = -10.f;
  constexpr auto OBJECT_ORIENTATION_MIN_VALUE = -3.14159;
  constexpr auto OBJECT_RCS_MIN_VALUE = -50.f;

  if (object_packets_ptr_->packets.size() == 0) {
    if (!first_object_packet_) {
      print_error("Object element before header. This can happen during the first iteration");
    }
    return;
  }

  if (object_list_ptr_->objects.size() >= object_header_packet_.u_number_of_objects) {
    object_packets_ptr_->packets.emplace_back(std::move(*packet_msg));
    return;
  }

  ObjectPacket object_packet;
  std::memcpy(&object_packet, packet_msg->data.data() + 4 * sizeof(uint8_t), sizeof(ObjectPacket));

  static_assert(sizeof(ObjectPacket) == object_packet_size);
  assert(packet_msg->data.size() == object_packet_size + 4);
  assert(object_header_packet_.u_sequence_counter == object_packet.u_sequence_counter);
  assert(
    object_packets_ptr_->packets.size() ==
    static_cast<std::size_t>(object_packet.u_message_counter + 1));

  for (const auto & fragment : object_packet.fragments) {
    continental_msgs::msg::ContinentalSrr520Object object_msg;
    const auto & data = fragment.data;

    if (object_list_ptr_->objects.size() >= object_header_packet_.u_number_of_objects) {
      break;
    }

    object_msg.object_id = data[0];

    uint16_t u_dist_x = ((static_cast<uint16_t>(data[1]) << 8) | data[2]);
    uint16_t u_dist_y = ((static_cast<uint16_t>(data[3]) << 8) | data[4]);
    assert(u_dist_x <= 65534);
    assert(u_dist_y <= 65534);
    object_msg.dist_x = DIST_RESOLUTION * u_dist_x + DIST_MIN_VALUE;
    object_msg.dist_y = DIST_RESOLUTION * u_dist_y + DIST_MIN_VALUE;

    uint16_t u_v_abs_x =
      (static_cast<uint16_t>(data[5]) << 6) | (static_cast<uint16_t>(data[6] & 0xfc) >> 2);
    assert(u_v_abs_x <= 16382);
    object_msg.v_abs_x = V_ABS_RESOLUTION * u_v_abs_x + V_ABS_MIN_VALUE;

    uint16_t u_v_abs_y = (static_cast<uint16_t>(data[6] & 0x03) << 12) |
                         (static_cast<uint16_t>(data[7]) << 4) |
                         (static_cast<uint16_t>(data[8] & 0xF0) >> 4);
    assert(u_v_abs_y <= 16382);
    object_msg.v_abs_y = V_ABS_RESOLUTION * u_v_abs_y + V_ABS_MIN_VALUE;

    uint16_t u_a_abs_x =
      (static_cast<uint16_t>(data[8] & 0x0f) << 6) | (static_cast<uint16_t>(data[9] & 0xfc) >> 2);
    assert(u_a_abs_x <= 1022);
    object_msg.a_abs_x = A_ABS_RESOLUTION * u_a_abs_x + A_ABS_MIN_VALUE;

    uint16_t u_a_abs_y =
      (static_cast<uint16_t>(data[9] & 0x03) << 8) | static_cast<uint16_t>(data[10]);
    assert(u_a_abs_y <= 1022);
    object_msg.a_abs_y = A_ABS_RESOLUTION * u_a_abs_y + A_ABS_MIN_VALUE;

    uint16_t u_dist_x_std =
      (static_cast<uint16_t>(data[11]) << 6) | (static_cast<uint16_t>(data[12] & 0xfc) >> 2);
    assert(u_dist_x_std <= 16383);
    object_msg.dist_x_std = DIST_STD_RESOLUTION * u_dist_x_std;

    uint16_t u_dist_y_std = (static_cast<uint16_t>(data[12] & 0x03) << 12) |
                            (static_cast<uint16_t>(data[13]) << 4) |
                            (static_cast<uint16_t>(data[14] & 0xF0) >> 4);
    assert(u_dist_y_std <= 16383);
    object_msg.dist_y_std = DIST_STD_RESOLUTION * u_dist_y_std;

    uint16_t u_v_abs_x_std = (static_cast<uint16_t>(data[14] & 0x0f) << 10) |
                             (static_cast<uint16_t>(data[15]) << 2) |
                             (static_cast<uint16_t>(data[15] & 0x03) >> 6);
    assert(u_v_abs_x_std <= 16383);
    object_msg.v_abs_x_std = V_ABS_STD_RESOLUTION * u_v_abs_x_std;

    uint16_t u_v_abs_y_std =
      (static_cast<uint16_t>(data[16] & 0x3f) << 8) | static_cast<uint16_t>(data[17]);
    assert(u_v_abs_y_std <= 16383);
    object_msg.v_abs_y_std = V_ABS_STD_RESOLUTION * u_v_abs_y_std;

    uint16_t u_a_abs_x_std =
      (static_cast<uint16_t>(data[18]) << 6) | (static_cast<uint16_t>(data[19] & 0xfc) >> 2);
    assert(u_a_abs_x_std <= 16383);
    object_msg.a_abs_x_std = A_ABS_STD_RESOLUTION * u_a_abs_x_std;

    uint16_t u_a_abs_y_std = (static_cast<uint16_t>(data[19] & 0x03) << 12) |
                             (static_cast<uint16_t>(data[20]) << 4) |
                             (static_cast<uint16_t>(data[21] & 0xF0) >> 4);
    assert(u_a_abs_y_std <= 16383);
    object_msg.a_abs_y_std = A_ABS_STD_RESOLUTION * u_a_abs_y_std;

    uint16_t u_box_length =
      (static_cast<uint16_t>(data[21] & 0x0f) << 8) | static_cast<uint16_t>(data[22]);
    assert(u_box_length <= 4095);
    object_msg.box_length = OBJECT_BOX_RESOLUTION * u_box_length;

    uint16_t u_box_width =
      (static_cast<uint16_t>(data[23]) << 4) | (static_cast<uint16_t>(data[24] & 0xF0) >> 4);
    assert(u_box_width <= 4095);
    object_msg.box_width = OBJECT_BOX_RESOLUTION * u_box_width;

    uint16_t u_orientation =
      (static_cast<uint16_t>(data[24] & 0x0f) << 8) | static_cast<uint16_t>(data[25]);
    assert(u_orientation <= 4094);
    object_msg.orientation =
      OBJECT_ORIENTATION_RESOLUTION * u_orientation + OBJECT_ORIENTATION_MIN_VALUE;

    uint16_t u_rcs =
      (static_cast<uint16_t>(data[26]) << 4) | (static_cast<uint16_t>(data[27] & 0xF0) >> 4);
    assert(u_rcs <= 4094);
    object_msg.rcs = OBJECT_RCS_RESOLUTION * u_rcs + OBJECT_RCS_MIN_VALUE;

    uint8_t u_score = data[27] & 0x0f;
    assert(u_score <= 15);
    object_msg.score = OBJECT_SCORE_RESOLUTION * u_score;

    object_msg.life_cycles = (static_cast<uint16_t>(data[28]) << 8) | data[29];
    assert(object_msg.life_cycles <= 65535);

    object_msg.box_valid = data[30] & 0x01;
    object_msg.object_status = (data[30] & 0x06) >> 1;

    object_list_ptr_->objects.push_back(object_msg);
  }

  object_packets_ptr_->packets.emplace_back(std::move(*packet_msg));
}

void ContinentalSRR520Decoder::process_crc_list_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  const auto crc_id = packet_msg->data[4];  // first 4 bits are the can id

  if (crc_id == near_crc_id) {
    process_near_crc_list_packet(std::move(packet_msg));
  } else if (crc_id == hrr_crc_id) {
    process_hrrcrc_list_packet(std::move(packet_msg));  // cspell: ignore HRRCRC
  } else if (crc_id == object_crc_id) {
    process_object_crc_list_packet(std::move(packet_msg));
  } else {
    print_error(std::string("Unrecognized CRC id=") + std::to_string(crc_id));
  }
}

void ContinentalSRR520Decoder::process_near_crc_list_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  if (rdi_near_packets_ptr_->packets.size() != rdi_near_packet_num + 1) {
    if (!first_rdi_near_packet_) {
      print_error("Incorrect number of RDI Near elements before CRC list");
    }

    rdi_near_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    near_detection_list_ptr_ =
      std::make_unique<continental_msgs::msg::ContinentalSrr520DetectionList>();
    return;
  }

  uint16_t transmitted_crc =
    (static_cast<uint16_t>(packet_msg->data[5]) << 8) | packet_msg->data[6];
  uint16_t computed_crc =
    crc16_packets(rdi_near_packets_ptr_->packets.begin(), rdi_near_packets_ptr_->packets.end(), 4);

  if (transmitted_crc != computed_crc) {
    print_error(
      "RDI Near: Transmitted CRC list does not coincide with the computed one. Ignoring packet");

    rdi_near_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    near_detection_list_ptr_ =
      std::make_unique<continental_msgs::msg::ContinentalSrr520DetectionList>();
    return;
  }

  rdi_near_packets_ptr_->packets.emplace_back(std::move(*packet_msg));

  if (near_detection_list_callback_) {
    near_detection_list_callback_(std::move(near_detection_list_ptr_));
  }

  if (nebula_packets_callback_) {
    nebula_packets_callback_(std::move(rdi_near_packets_ptr_));
  }

  rdi_near_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  near_detection_list_ptr_ =
    std::make_unique<continental_msgs::msg::ContinentalSrr520DetectionList>();
}

void ContinentalSRR520Decoder::process_hrrcrc_list_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  if (rdi_hrr_packets_ptr_->packets.size() != rdi_hrr_packet_num + 1) {
    if (!first_rdi_hrr_packet_) {
      print_error("Incorrect number of RDI HRR elements before CRC list");
    }

    rdi_hrr_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    hrr_detection_list_ptr_ =
      std::make_unique<continental_msgs::msg::ContinentalSrr520DetectionList>();
    return;
  }

  uint16_t transmitted_crc =
    (static_cast<uint16_t>(packet_msg->data[5]) << 8) | packet_msg->data[6];
  uint16_t computed_crc =
    crc16_packets(rdi_hrr_packets_ptr_->packets.begin(), rdi_hrr_packets_ptr_->packets.end(), 4);

  if (transmitted_crc != computed_crc) {
    print_error(
      "RDI HRR: Transmitted CRC list does not coincide with the computed one. Ignoring packet");
    rdi_hrr_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    hrr_detection_list_ptr_ =
      std::make_unique<continental_msgs::msg::ContinentalSrr520DetectionList>();
    return;
  }

  rdi_hrr_packets_ptr_->packets.emplace_back(std::move(*packet_msg));

  if (hrr_detection_list_callback_) {
    hrr_detection_list_callback_(std::move(hrr_detection_list_ptr_));
  }

  if (nebula_packets_callback_) {
    nebula_packets_callback_(std::move(rdi_hrr_packets_ptr_));
  }

  rdi_hrr_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  hrr_detection_list_ptr_ =
    std::make_unique<continental_msgs::msg::ContinentalSrr520DetectionList>();
}

void ContinentalSRR520Decoder::process_object_crc_list_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  if (object_packets_ptr_->packets.size() != object_packet_num + 1) {
    if (!first_object_packet_) {
      print_error("Incorrect number of object packages before CRC list");
    }

    object_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    object_list_ptr_ = std::make_unique<continental_msgs::msg::ContinentalSrr520ObjectList>();
    return;
  }

  uint16_t transmitted_crc =
    (static_cast<uint16_t>(packet_msg->data[5]) << 8) | packet_msg->data[6];
  uint16_t computed_crc =
    crc16_packets(object_packets_ptr_->packets.begin(), object_packets_ptr_->packets.end(), 4);

  if (transmitted_crc != computed_crc) {
    print_error(
      "Object: Transmitted CRC list does not coincide with the computed one. Ignoring packet");

    object_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    object_list_ptr_ = std::make_unique<continental_msgs::msg::ContinentalSrr520ObjectList>();
    return;
  }

  object_packets_ptr_->packets.emplace_back(std::move(*packet_msg));

  if (object_list_callback_) {
    object_list_callback_(std::move(object_list_ptr_));
  }

  if (nebula_packets_callback_) {
    nebula_packets_callback_(std::move(object_packets_ptr_));
  }

  object_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  object_list_ptr_ = std::make_unique<continental_msgs::msg::ContinentalSrr520ObjectList>();
}

void ContinentalSRR520Decoder::process_sensor_status_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  static_assert(sizeof(StatusPacket) == status_packet_size);

  constexpr float status_distance_resolution = 1e-3f;
  constexpr float status_distance_min_value = -32.767;
  constexpr float status_angle_resolution = 9.58766f;
  constexpr float status_angle_min_value = -3.14159f;
  constexpr auto status_angle_std_resolution = 1.52593e-05;

  StatusPacket status_packet;
  std::memcpy(&status_packet, packet_msg->data.data() + 4 * sizeof(uint8_t), sizeof(status_packet));

  auto diagnostic_array_msg_ptr = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();

  diagnostic_array_msg_ptr->header.frame_id = sensor_configuration_->frame_id;
  diagnostic_array_msg_ptr->header.stamp = packet_msg->stamp;
  diagnostic_array_msg_ptr->status.resize(1);

  auto & diagnostic_status = diagnostic_array_msg_ptr->status.front();
  auto & diagnostic_values = diagnostic_status.values;
  diagnostic_values.reserve(33);
  diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  diagnostic_status.message = "Sensor diagnostics for the SRR520";
  diagnostic_status.hardware_id = "SRR";
  diagnostic_status.name = "SRR";
  diagnostic_msgs::msg::KeyValue key_value;

  key_value.key = "time_stamp";
  key_value.value = std::to_string(status_packet.u_time_stamp.value());
  diagnostic_values.push_back(key_value);

  key_value.key = "global_time_stamp_sec";
  key_value.value = std::to_string(status_packet.u_global_time_stamp_sec.value());
  diagnostic_values.push_back(key_value);

  key_value.key = "global_time_stamp_nsec";
  key_value.value = std::to_string(status_packet.u_global_time_stamp_nsec.value());
  diagnostic_values.push_back(key_value);

  key_value.key = "global_time_stamp_sync_status";
  key_value.value = std::to_string(status_packet.u_global_time_stamp_sync_status);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_global_time_stamp_sync_status != 1
                              ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                              : diagnostic_status.level;

  std::stringstream sw_version_ss;
  sw_version_ss << static_cast<uint16_t>(status_packet.u_sw_version_major) << "."
                << static_cast<uint16_t>(status_packet.u_sw_version_minor) << "."
                << static_cast<uint16_t>(status_packet.u_sw_version_patch);
  key_value.key = "sw_version_patch";
  key_value.value = sw_version_ss.str();
  diagnostic_values.push_back(key_value);

  key_value.key = "sensor_id";
  key_value.value = std::to_string(status_packet.u_sensor_id);
  diagnostic_values.push_back(key_value);

  key_value.key = "long_pos";
  key_value.value = std::to_string(
    status_distance_resolution * status_packet.u_long_pos.value() + status_distance_min_value);
  diagnostic_values.push_back(key_value);

  key_value.key = "lat_pos";
  key_value.value = std::to_string(
    status_distance_resolution * status_packet.u_lat_pos.value() + status_distance_min_value);
  diagnostic_values.push_back(key_value);

  key_value.key = "vert_pos";
  key_value.value = std::to_string(
    status_distance_resolution * status_packet.u_vert_pos.value() + status_distance_min_value);
  diagnostic_values.push_back(key_value);

  key_value.key = "long_pos_cog";
  key_value.value = std::to_string(
    status_distance_resolution * status_packet.u_long_pos_cog.value() + status_distance_min_value);
  diagnostic_values.push_back(key_value);

  key_value.key = "wheelbase";
  key_value.value = std::to_string(status_distance_resolution * status_packet.u_wheelbase.value());
  diagnostic_values.push_back(key_value);

  key_value.key = "yaw_angle";
  key_value.value = std::to_string(
    status_angle_resolution * status_packet.u_yaw_angle.value() + status_angle_min_value);
  diagnostic_values.push_back(key_value);

  key_value.key = "cover_damping";
  key_value.value = std::to_string(
    status_distance_resolution * status_packet.u_cover_damping.value() + status_distance_min_value);
  diagnostic_values.push_back(key_value);

  uint8_t plug_orientation = status_packet.u_plug_orientation & 0b1;
  key_value.key = "plug_orientation";
  key_value.value = plug_orientation == 0 ? "0:PLUG_BOTTOM" : "1:PLUG_TOP";
  diagnostic_values.push_back(key_value);

  std::vector<std::string> defective_message_vector;

  if (status_packet.u_defective & 0x01) {
    defective_message_vector.push_back("Current Near Scan RF Error");
  }
  if (status_packet.u_defective & 0x02) {
    defective_message_vector.push_back("Near Scan RF this OPC");
  }
  if (status_packet.u_defective & 0x04) {
    defective_message_vector.push_back("Current HRR Scan RF Error");
  }
  if (status_packet.u_defective & 0x08) {
    defective_message_vector.push_back("HRR Scan RF this OPC");
  }
  if (status_packet.u_defective & 0x10) {
    defective_message_vector.push_back("Current RF HW Error");
  }
  if (status_packet.u_defective & 0x20) {
    defective_message_vector.push_back("RF HW Error this OPC");
  }
  if (status_packet.u_defective & 0x40) {
    defective_message_vector.push_back("Current HW Error");
  }
  if (status_packet.u_defective & 0x80) {
    defective_message_vector.push_back("HW Error this OPC");
  }

  key_value.key = "defective";
  key_value.value = defective_message_vector.size() == 0
                      ? "Ok"
                      : boost::algorithm::join(defective_message_vector, ", ");
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_defective != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  std::vector<std::string> supply_voltage_message_vector;

  if (status_packet.u_supply_voltage_limit & 0x01) {
    supply_voltage_message_vector.push_back("Current Overvoltage Error");
  }
  if (status_packet.u_supply_voltage_limit & 0x02) {
    supply_voltage_message_vector.push_back("Overvoltage Error this OPC");
  }
  if (status_packet.u_supply_voltage_limit & 0x04) {
    supply_voltage_message_vector.push_back("Current Undervoltage Error");
  }
  if (status_packet.u_supply_voltage_limit & 0x08) {
    supply_voltage_message_vector.push_back("Undervoltage Error this OPC");
  }

  key_value.key = "supply_voltage_limit";
  key_value.value = supply_voltage_message_vector.size() == 0
                      ? "Ok"
                      : boost::algorithm::join(supply_voltage_message_vector, ", ");
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_supply_voltage_limit != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  std::vector<std::string> temperature_message_vector;

  if (status_packet.u_sensor_off_temp & 0x01) {
    temperature_message_vector.push_back("Current Overtemperature Error");
  }
  if (status_packet.u_sensor_off_temp & 0x02) {
    temperature_message_vector.push_back("Overtemperature Error this OPC");
  }
  if (status_packet.u_sensor_off_temp & 0x04) {
    temperature_message_vector.push_back("Current Undertemperature Error");
  }
  if (status_packet.u_sensor_off_temp & 0x08) {
    temperature_message_vector.push_back("Undertemperature Error this OPC");
  }

  key_value.key = "sensor_off_temp";
  key_value.value = temperature_message_vector.size() == 0
                      ? "Ok"
                      : boost::algorithm::join(temperature_message_vector, ", ");
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_sensor_off_temp != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  auto valid_flag_to_string = [](uint8_t status) -> std::string {
    std::vector<std::string> status_vector;
    if (status == 0) {
      return "Ok";
    }
    if (status > 3) {
      return "Invalid value";
    }
    if (status & 0x01) {
      status_vector.push_back("Current error");
    }
    if (status & 0x02) {
      status_vector.push_back("Error this OPC");
    }

    return boost::algorithm::join(status_vector, ", ");
  };

  uint8_t u_long_vel_invalid = status_packet.u_dynamics_invalid0 & 0b00000011;
  key_value.key = "long_vel_invalid";
  key_value.value = valid_flag_to_string(u_long_vel_invalid);
  diagnostic_values.push_back(key_value);

  uint8_t u_long_accel_invalid = (status_packet.u_dynamics_invalid0 & 0b00001100) >> 2;
  key_value.key = "long_accel_invalid";
  key_value.value = valid_flag_to_string(u_long_accel_invalid);
  diagnostic_values.push_back(key_value);

  uint8_t u_lat_accel_invalid = (status_packet.u_dynamics_invalid0 & 0b00110000) >> 4;
  key_value.key = "lat_accel_invalid";
  key_value.value = valid_flag_to_string(u_lat_accel_invalid);
  diagnostic_values.push_back(key_value);

  uint8_t u_long_dir_invalid = (status_packet.u_dynamics_invalid0 & 0b11000000) >> 6;
  key_value.key = "long_dir_invalid";
  key_value.value = valid_flag_to_string(u_long_dir_invalid);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_dynamics_invalid0 != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                              : diagnostic_status.level;

  uint8_t u_yaw_rate_invalid = status_packet.u_dynamics_invalid1 & 0b00000011;
  key_value.key = "yaw_rate_invalid";
  key_value.value = valid_flag_to_string(u_yaw_rate_invalid);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_dynamics_invalid1 != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                              : diagnostic_status.level;

  std::vector<std::string> ext_disturbed_message_vector;

  if (status_packet.u_ext_disturbed & 0x01) {
    ext_disturbed_message_vector.push_back("Current Near Scan RF Error");
  }
  if (status_packet.u_ext_disturbed & 0x02) {
    ext_disturbed_message_vector.push_back("Near Scan RF this OPC");
  }
  if (status_packet.u_ext_disturbed & 0x04) {
    ext_disturbed_message_vector.push_back("Current HRR Scan RF Error");
  }
  if (status_packet.u_ext_disturbed & 0x08) {
    ext_disturbed_message_vector.push_back("HRR Scan RF this OPC");
  }
  if (status_packet.u_ext_disturbed & 0x10) {
    ext_disturbed_message_vector.push_back("Current RF HW Error");
  }
  if (status_packet.u_ext_disturbed & 0x20) {
    ext_disturbed_message_vector.push_back("RF HW Error this OPC");
  }
  if (status_packet.u_ext_disturbed & 0x40) {
    ext_disturbed_message_vector.push_back("Current HW Error");
  }
  if (status_packet.u_ext_disturbed & 0x80) {
    ext_disturbed_message_vector.push_back("HW Error this OPC");
  }

  key_value.key = "ext_disturbed";
  key_value.value = ext_disturbed_message_vector.size() == 0
                      ? "Ok"
                      : boost::algorithm::join(ext_disturbed_message_vector, ", ");
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_ext_disturbed != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  key_value.key = "com_error";
  key_value.value = valid_flag_to_string(status_packet.u_com_error);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_com_error != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  std::vector<std::string> sw_message_vector;
  uint8_t u_sw_error = status_packet.u_sw_error.value() & 0xff;

  if (u_sw_error & 0x01) {
    sw_message_vector.push_back("Current Internal SW Error");
  }
  if (u_sw_error & 0x02) {
    sw_message_vector.push_back("Internal SW Error this OPC");
  }
  if (u_sw_error & 0x04) {
    sw_message_vector.push_back("Reset Error");
  }
  if (u_sw_error & 0x08) {
    sw_message_vector.push_back("Current Nvm Integrity");
  }
  if (u_sw_error & 0x10) {
    sw_message_vector.push_back("Nvm Integrity Error this OPC");
  }
  if (u_sw_error & 0x20) {
    sw_message_vector.push_back("RF HW Error this OPC");
  }
  if (u_sw_error & 0x40) {
    sw_message_vector.push_back("Runtime Error");
  }
  if (u_sw_error & 0x80) {
    sw_message_vector.push_back("Last Sensor Config Message Rejected Upper");
  }

  key_value.key = "sw_error";
  key_value.value =
    sw_message_vector.size() == 0 ? "Ok" : boost::algorithm::join(sw_message_vector, ", ");
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = (status_packet.u_sw_error.value() & 0x0f) != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  bool aln_driving = status_packet.u_aln_status_azimuth_available & 0b00000001;
  bool aln_sensor = (status_packet.u_aln_status_azimuth_available & 0b00000010) >> 1;

  key_value.key = "aln_status_driving";
  key_value.value = aln_driving ? "Driving conditions met" : "Driving conditions NOT met";
  diagnostic_values.push_back(key_value);

  key_value.key = "aln_status_sensor";
  key_value.value = aln_sensor ? "Sensor is adjusted" : "Sensor is not adjusted";
  diagnostic_values.push_back(key_value);

  key_value.key = "aln_azimuth_available";
  key_value.value =
    std::to_string((status_packet.u_aln_status_azimuth_available & 0b00000100) >> 2);
  diagnostic_values.push_back(key_value);

  key_value.key = "aln_current_azimuth_std";
  key_value.value =
    std::to_string(status_angle_std_resolution * status_packet.u_aln_current_azimuth_std.value());
  diagnostic_values.push_back(key_value);

  key_value.key = "aln_current_azimuth";
  key_value.value = std::to_string(
    status_angle_resolution * status_packet.u_aln_current_azimuth.value() + status_angle_min_value);
  diagnostic_values.push_back(key_value);

  key_value.key = "aln_current_delta";
  key_value.value = std::to_string(
    status_angle_resolution * status_packet.u_aln_current_delta.value() + status_angle_min_value);
  diagnostic_values.push_back(key_value);

  uint16_t computed_crc = crc16_packet(packet_msg->data.begin() + 4, packet_msg->data.end() - 3);
  key_value.key = "crc_check";
  key_value.value =
    std::to_string(status_packet.u_crc.value()) + "|" + std::to_string(computed_crc);
  diagnostic_values.push_back(key_value);

  key_value.key = "sequence_counter";
  key_value.value = std::to_string(status_packet.u_sequence_counter);
  diagnostic_values.push_back(key_value);

  if (status_callback_) {
    status_callback_(std::move(diagnostic_array_msg_ptr));
  }

  auto nebula_packets = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  nebula_packets->header.stamp = packet_msg->stamp;
  nebula_packets->header.frame_id = sensor_configuration_->frame_id;
  nebula_packets->packets.emplace_back(std::move(*packet_msg));

  if (nebula_packets_callback_) {
    nebula_packets_callback_(std::move(nebula_packets));
  }
}

void ContinentalSRR520Decoder::process_sync_follow_up_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  if (sync_follow_up_callback_) {
    sync_follow_up_callback_(packet_msg->stamp);
  }

  auto nebula_packets = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  nebula_packets->header.stamp = packet_msg->stamp;
  nebula_packets->header.frame_id = sensor_configuration_->frame_id;
  nebula_packets->packets.emplace_back(std::move(*packet_msg));

  if (nebula_packets_callback_) {
    nebula_packets_callback_(std::move(nebula_packets));
  }
}

void ContinentalSRR520Decoder::set_logger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger_ptr_ = logger;
}

void ContinentalSRR520Decoder::print_info(std::string info)
{
  if (parent_node_logger_ptr_) {
    RCLCPP_INFO_STREAM((*parent_node_logger_ptr_), info);
  } else {
    std::cout << info << std::endl;
  }
}

void ContinentalSRR520Decoder::print_error(std::string error)
{
  if (parent_node_logger_ptr_) {
    RCLCPP_ERROR_STREAM((*parent_node_logger_ptr_), error);
  } else {
    std::cerr << error << std::endl;
  }
}

void ContinentalSRR520Decoder::print_debug(std::string debug)
{
  if (parent_node_logger_ptr_) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger_ptr_), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

}  // namespace nebula::drivers::continental_srr520
