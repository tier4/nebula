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

#include "nebula_decoders/nebula_decoders_continental/decoders/continental_srr520_decoder.hpp"

#include "nebula_common/continental/continental_srr520.hpp"

#include <nebula_common/continental/crc.hpp>

#include <cmath>
#include <utility>

namespace nebula
{
namespace drivers
{
namespace continental_srr520
{
ContinentalSRR520Decoder::ContinentalSRR520Decoder(
  const std::shared_ptr<continental_srr520::ContinentalSRR520SensorConfiguration> &
    sensor_configuration)
{
  sensor_configuration_ = sensor_configuration;
}

Status ContinentalSRR520Decoder::RegisterNearDetectionListCallback(
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList>)>
    detection_list_callback)
{
  near_detection_list_callback_ = std::move(detection_list_callback);
  return Status::OK;
}

Status ContinentalSRR520Decoder::RegisterHRRDetectionListCallback(
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520DetectionList>)>
    detection_list_callback)
{
  hrr_detection_list_callback_ = std::move(detection_list_callback);
  return Status::OK;
}

Status ContinentalSRR520Decoder::RegisterObjectListCallback(
  std::function<void(std::unique_ptr<continental_msgs::msg::ContinentalSrr520ObjectList>)>
    object_list_callback)
{
  object_list_callback_ = std::move(object_list_callback);
  return Status::OK;
}

Status ContinentalSRR520Decoder::RegisterStatusCallback(
  std::function<void(std::unique_ptr<diagnostic_msgs::msg::DiagnosticArray>)> status_callback)
{
  status_callback_ = std::move(status_callback);
  return Status::OK;
}

bool ContinentalSRR520Decoder::ProcessPackets(
  const nebula_msgs::msg::NebulaPackets & nebula_packets)
{
  if (nebula_packets.packets.size() == 0 || nebula_packets.packets.front().data.size() < 2) {
    return false;
  }

  const auto & can_id_messages = nebula_packets.packets.front().data;
  const int can_message_id = (static_cast<uint16_t>(can_id_messages[1]) << 8) | can_id_messages[0];

  if (
    can_message_id == RDI_NEAR_HEADER_CAN_MESSAGE_ID &&
    nebula_packets.packets.size() == RDI_NEAR_PACKET_NUM + 2) {
    return ParseDetectionsListPacket(nebula_packets, true);
  } else if (
    can_message_id == RDI_HRR_HEADER_CAN_MESSAGE_ID &&
    nebula_packets.packets.size() == RDI_HRR_PACKET_NUM + 2) {
    return ParseDetectionsListPacket(nebula_packets, false);
  } else if (
    can_message_id == OBJECT_HEADER_CAN_MESSAGE_ID &&
    nebula_packets.packets.size() == OBJECT_PACKET_NUM + 2) {
    return ParseObjectsListPacket(nebula_packets);
  } else if (can_message_id == STATUS_CAN_MESSAGE_ID && nebula_packets.packets.size() == 2) {
    return ParseStatusPacket(nebula_packets);
  }

  return false;
}

bool ContinentalSRR520Decoder::ParseDetectionsListPacket(
  const nebula_msgs::msg::NebulaPackets & nebula_packets, bool near)
{
  assert(near || nebula_packets.packets.size() != RDI_NEAR_PACKET_NUM);
  assert(!near || nebula_packets.packets.size() != RDI_HRR_PACKET_NUM);
  static_assert(sizeof(ScanHeaderPacket) == RDI_NEAR_HEADER_PACKET_SIZE);
  static_assert(sizeof(ScanHeaderPacket) == RDI_HRR_HEADER_PACKET_SIZE);
  static_assert(sizeof(DetectionPacket) == RDI_NEAR_ELEMENT_PACKET_SIZE);
  static_assert(sizeof(DetectionPacket) == RDI_HRR_ELEMENT_PACKET_SIZE);

  ScanHeaderPacket header_packet;
  assert(nebula_packets.packets[1].data.size() == RDI_NEAR_HEADER_PACKET_SIZE);

  std::memcpy(&header_packet, nebula_packets.packets[1].data.data(), sizeof(ScanHeaderPacket));

  auto scan_msg = std::make_unique<continental_msgs::msg::ContinentalSrr520DetectionList>();

  assert(
    header_packet.u_global_time_stamp_sync_status >= 1 &&
    header_packet.u_global_time_stamp_sync_status <= 3);

  scan_msg->header = nebula_packets.header;
  scan_msg->header.frame_id = sensor_configuration_->frame_id;

  if (header_packet.u_global_time_stamp_sync_status == 1) {
    scan_msg->header.stamp.sec = header_packet.u_global_time_stamp_sec.value();
    scan_msg->header.stamp.sec = header_packet.u_global_time_stamp_nsec.value();
  }

  scan_msg->internal_time_stamp_usec = header_packet.u_time_stamp.value();
  scan_msg->global_time_stamp_sync_status = header_packet.u_global_time_stamp_sync_status;
  scan_msg->signal_status = header_packet.u_signal_status;
  scan_msg->sequence_counter = header_packet.u_sequence_counter;
  scan_msg->cycle_counter = header_packet.u_cycle_counter.value();
  scan_msg->vambig = 0.003051851f * header_packet.u_vambig.value() - 100.f;
  scan_msg->max_range = 0.1f * header_packet.u_max_range.value();

  int parsed_detections = 0;
  int detection_packet_index = 0;
  scan_msg->detections.reserve(header_packet.u_number_of_detections.value());

  for (auto it = nebula_packets.packets.cbegin() + 2; it != nebula_packets.packets.cend(); it++) {
    assert(it->data.size() == RDI_NEAR_ELEMENT_PACKET_SIZE);

    if (parsed_detections >= header_packet.u_number_of_detections.value()) {
      break;
    }

    DetectionPacket detection_packet;
    std::memcpy(&detection_packet, it->data.data(), sizeof(DetectionPacket));

    assert(header_packet.u_sequence_counter == detection_packet.u_sequence_counter);
    assert(detection_packet_index == detection_packet.u_message_counter);

    for (const auto & fragment : detection_packet.fragments) {
      continental_msgs::msg::ContinentalSrr520Detection detection_msg;
      const auto & data = fragment.data;

      if (parsed_detections >= header_packet.u_number_of_detections.value()) {
        break;
      }

      uint16_t u_range =
        (static_cast<uint16_t>(data[0]) << 4) | (static_cast<uint16_t>(data[1] & 0xF0) >> 4);
      detection_msg.range = 0.024420024 * u_range;

      uint16_t u_azimuth =
        (static_cast<uint16_t>(data[1] & 0x0f) << 5) | (static_cast<uint16_t>(data[2] & 0xF8) >> 3);
      detection_msg.azimuth_angle = 0.006159986 * u_azimuth - 1.570796327;

      uint16_t u_range_rate =
        (static_cast<uint16_t>(data[2] & 0x07) << 8) | static_cast<uint16_t>(data[3]);
      detection_msg.range_rate = 0.014662757 * u_range_rate - 15.f;

      uint8_t u_rcs = (data[4] & 0xFE) >> 1;
      detection_msg.rcs = 0.476190476 * u_rcs - 40.f;

      detection_msg.pdh00 = 100 * (data[4] & 0x01);
      detection_msg.pdh01 = 100 * ((data[5] & 0x80) >> 7);
      detection_msg.pdh02 = 100 * ((data[5] & 0x40) >> 6);
      detection_msg.pdh03 = 100 * ((data[5] & 0x20) >> 5);
      detection_msg.pdh04 = 100 * ((data[5] & 0x10) >> 4);

      uint8_t u_snr = data[5] & 0x0f;
      detection_msg.snr = 1.7 * u_snr + 11.f;

      scan_msg->detections.push_back(detection_msg);
      parsed_detections++;
    }

    detection_packet_index++;
  }

  if (near) {
    near_detection_list_callback_(std::move(scan_msg));
  } else {
    hrr_detection_list_callback_(std::move(scan_msg));
  }

  return true;
}

bool ContinentalSRR520Decoder::ParseObjectsListPacket(
  const nebula_msgs::msg::NebulaPackets & nebula_packets)
{
  assert(nebula_packets.packets.size() != OBJECT_PACKET_NUM);
  static_assert(sizeof(ObjectHeaderPacket) == OBJECT_HEADER_PACKET_SIZE);

  ObjectHeaderPacket header_packet;
  assert(nebula_packets.packets[1].data.size() == RDI_NEAR_HEADER_PACKET_SIZE);

  std::memcpy(&header_packet, nebula_packets.packets[1].data.data(), sizeof(ScanHeaderPacket));

  auto objects_msg = std::make_unique<continental_msgs::msg::ContinentalSrr520ObjectList>();

  assert(
    header_packet.u_global_time_stamp_sync_status >= 1 &&
    header_packet.u_global_time_stamp_sync_status <= 3);

  objects_msg->header = nebula_packets.header;
  objects_msg->header.frame_id = sensor_configuration_->base_frame;

  if (header_packet.u_global_time_stamp_sync_status == 1) {
    objects_msg->header.stamp.sec = header_packet.u_global_time_stamp_sec.value();
    objects_msg->header.stamp.sec = header_packet.u_global_time_stamp_nsec.value();
  }

  objects_msg->internal_time_stamp_usec = header_packet.u_time_stamp.value();
  objects_msg->global_time_stamp_sync_status = header_packet.u_global_time_stamp_sync_status;
  objects_msg->signal_status = header_packet.u_signal_status;
  objects_msg->sequence_counter = header_packet.u_sequence_counter;
  objects_msg->cycle_counter = header_packet.u_cycle_counter.value();
  objects_msg->ego_vx = 0.003051851 * header_packet.u_ego_vx.value() - 100.f;
  objects_msg->ego_yaw_rate = 9.58766e-05 * header_packet.u_ego_yaw_rate.value() - 3.14159;
  objects_msg->motion_type = header_packet.u_motion_type;

  int parsed_objects = 0;
  int object_packet_index = 0;
  objects_msg->objects.reserve(header_packet.u_number_of_objects);

  for (auto it = nebula_packets.packets.cbegin() + 2; it != nebula_packets.packets.cend(); it++) {
    assert(it->data.size() == OBJECT_PACKET_SIZE);

    if (parsed_objects >= header_packet.u_number_of_objects) {
      break;
    }

    DetectionPacket detection_packet;
    std::memcpy(&detection_packet, it->data.data(), sizeof(DetectionPacket));

    assert(header_packet.u_sequence_counter == detection_packet.u_sequence_counter);
    assert(object_packet_index == detection_packet.u_message_counter);

    for (const auto & fragment : detection_packet.fragments) {
      continental_msgs::msg::ContinentalSrr520Object object_msg;
      const auto & data = fragment.data;

      if (parsed_objects >= header_packet.u_number_of_objects) {
        break;
      }

      object_msg.object_id = data[0];
      object_msg.dist_x = 0.009155553 * ((static_cast<uint16_t>(data[1]) << 8) | data[2]) - 300.f;
      object_msg.dist_y = 0.009155553 * ((static_cast<uint16_t>(data[3]) << 8) | data[4]) - 300.f;

      uint16_t u_v_abs_x =
        (static_cast<uint16_t>(data[5]) << 6) | (static_cast<uint16_t>(data[6] & 0xfc) >> 2);
      object_msg.v_abs_x = 0.009156391 * u_v_abs_x - 75.f;

      uint16_t u_v_abs_y = (static_cast<uint16_t>(data[6] & 0x03) << 12) |
                           (static_cast<uint16_t>(data[7]) << 4) |
                           (static_cast<uint16_t>(data[8] & 0xF0) >> 4);
      object_msg.v_abs_y = 0.009156391 * u_v_abs_y - 75.f;

      uint16_t u_a_abs_x =
        (static_cast<uint16_t>(data[8] & 0x0f) << 6) | (static_cast<uint16_t>(data[9] & 0xfc) >> 2);
      object_msg.a_abs_x = 0.019569472 * u_a_abs_x - 10.f;

      uint16_t u_a_abs_y =
        (static_cast<uint16_t>(data[9] & 0x03) << 8) | static_cast<uint16_t>(data[10]);
      object_msg.a_abs_y = 0.019569472 * u_a_abs_y - 10.f;

      uint16_t u_dist_x_std =
        (static_cast<uint16_t>(data[11]) << 6) | (static_cast<uint16_t>(data[12] & 0xfc) >> 2);
      object_msg.dist_x_std = 0.001831166 * u_dist_x_std;

      uint16_t u_dist_y_std = (static_cast<uint16_t>(data[12] & 0x03) << 12) |
                              (static_cast<uint16_t>(data[13]) << 4) |
                              (static_cast<uint16_t>(data[14] & 0xF0) >> 4);
      object_msg.dist_y_std = 0.001831166 * u_dist_y_std;

      uint16_t u_v_abs_x_std = (static_cast<uint16_t>(data[14] & 0x0f) << 10) |
                               (static_cast<uint16_t>(data[15]) << 2) |
                               (static_cast<uint16_t>(data[15] & 0x03) >> 6);
      object_msg.v_abs_x_std = 0.001831166 * u_v_abs_x_std;

      uint16_t u_v_abs_y_std =
        (static_cast<uint16_t>(data[16] & 0x3f) << 8) | static_cast<uint16_t>(data[17]);
      object_msg.v_abs_y_std = 0.001831166 * u_v_abs_y_std;

      uint16_t u_a_abs_x_std =
        (static_cast<uint16_t>(data[18]) << 6) | (static_cast<uint16_t>(data[19] & 0xfc) >> 2);
      object_msg.a_abs_x_std = 0.001831166 * u_a_abs_x_std;

      uint16_t u_a_abs_y_std = (static_cast<uint16_t>(data[19] & 0x03) << 12) |
                               (static_cast<uint16_t>(data[20]) << 4) |
                               (static_cast<uint16_t>(data[21] & 0xF0) >> 4);
      object_msg.a_abs_y_std = 0.001831166 * u_a_abs_y_std;

      uint16_t u_box_length =
        (static_cast<uint16_t>(data[21] & 0x0f) << 8) | static_cast<uint16_t>(data[22]);
      object_msg.box_length = 0.007326007 * u_box_length;

      uint16_t u_box_width =
        (static_cast<uint16_t>(data[23]) << 4) | (static_cast<uint16_t>(data[24] & 0xF0) >> 4);
      object_msg.box_width = 0.007326007 * u_box_width;

      uint16_t u_orientation =
        (static_cast<uint16_t>(data[24] & 0x0f) << 8) | static_cast<uint16_t>(data[25]);
      object_msg.orientation = 0.001534729 * u_orientation - 3.14159;

      uint16_t u_rcs =
        (static_cast<uint16_t>(data[26]) << 4) | (static_cast<uint16_t>(data[27] & 0xF0) >> 4);
      object_msg.rcs = 0.024425989 * u_rcs - 50.f;

      uint8_t u_score = data[27] & 0x0f;
      object_msg.score = 6.666666667 * u_score;

      object_msg.life_cycles = (static_cast<uint16_t>(data[28]) << 8) | data[29];

      object_msg.box_valid = data[30] & 0x01;
      object_msg.object_status = (data[30] & 0x06) >> 1;

      objects_msg->objects.push_back(object_msg);
      parsed_objects++;
    }

    object_packet_index++;
  }

  object_list_callback_(std::move(objects_msg));

  return true;
}

bool ContinentalSRR520Decoder::ParseStatusPacket(
  const nebula_msgs::msg::NebulaPackets & nebula_packets)
{
  static_assert(sizeof(StatusPacket) == 64);

  StatusPacket status_packet;
  std::memcpy(&status_packet, nebula_packets.packets[1].data.data(), sizeof(status_packet));

  auto diagnostic_array_msg_ptr = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();

  diagnostic_array_msg_ptr->header = nebula_packets.header;
  diagnostic_array_msg_ptr->status.resize(1);

  auto & diagnostic_status = diagnostic_array_msg_ptr->status.front();
  auto & diagnostic_values = diagnostic_status.values;
  diagnostic_values.reserve(33);  // add number later
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
  key_value.value = std::to_string(1e-3 * status_packet.u_long_pos.value() - 32.767);
  diagnostic_values.push_back(key_value);

  key_value.key = "lat_pos";
  key_value.value = std::to_string(1e-3 * status_packet.u_lat_pos.value() - 32.767);
  diagnostic_values.push_back(key_value);

  key_value.key = "vert_pos";
  key_value.value = std::to_string(1e-3 * status_packet.u_vert_pos.value() - 32.767);
  diagnostic_values.push_back(key_value);

  key_value.key = "long_pos_cog";
  key_value.value = std::to_string(1e-3 * status_packet.u_long_pos_cog.value() - 32.767);
  diagnostic_values.push_back(key_value);

  key_value.key = "wheelbase";
  key_value.value = std::to_string(1e-3 * status_packet.u_wheelbase.value());
  diagnostic_values.push_back(key_value);

  key_value.key = "yaw_angle";
  key_value.value = std::to_string(9.58766e-05 * status_packet.u_yaw_angle.value() - 3.14159);
  diagnostic_values.push_back(key_value);

  key_value.key = "cover_damping";
  key_value.value = std::to_string(1e-3 * status_packet.u_cover_damping.value() - 32.767);
  diagnostic_values.push_back(key_value);

  key_value.key = "plug_orientation";
  key_value.value = std::to_string(status_packet.u_plug_orientation & 0b1);
  diagnostic_values.push_back(key_value);

  key_value.key = "defective";
  key_value.value = std::to_string(status_packet.u_defective);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_defective != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  key_value.key = "supply_voltage_limit";
  key_value.value = std::to_string(status_packet.u_supply_voltage_limit);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_supply_voltage_limit != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  key_value.key = "sensor_off_temp";
  key_value.value = std::to_string(status_packet.u_sensor_off_temp);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_sensor_off_temp != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  key_value.key = "sensor_off_temp";
  key_value.value = std::to_string(status_packet.u_sensor_off_temp);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_sensor_off_temp != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  key_value.key = "long_vel_invalid";
  key_value.value = std::to_string(status_packet.u_dynamics_invalid0 & 0b00000011);
  diagnostic_values.push_back(key_value);

  key_value.key = "long_accel_invalid";
  key_value.value = std::to_string((status_packet.u_dynamics_invalid0 & 0b00001100) >> 2);
  diagnostic_values.push_back(key_value);

  key_value.key = "lat_accel_invalid";
  key_value.value = std::to_string((status_packet.u_dynamics_invalid0 & 0b00110000) >> 4);
  diagnostic_values.push_back(key_value);

  key_value.key = "long_dir_invalid";
  key_value.value = std::to_string((status_packet.u_dynamics_invalid0 & 0b11000000) >> 6);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_dynamics_invalid0 != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                              : diagnostic_status.level;

  key_value.key = "yaw_rate_invalid";
  key_value.value = std::to_string(status_packet.u_dynamics_invalid1 & 0b00000011);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_dynamics_invalid1 != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                              : diagnostic_status.level;

  key_value.key = "ext_disturbed";
  key_value.value = std::to_string(status_packet.u_ext_disturbed);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_ext_disturbed != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  key_value.key = "com_error";
  key_value.value = std::to_string(status_packet.u_com_error);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = status_packet.u_com_error != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  key_value.key = "sw_error";
  key_value.value = std::to_string(status_packet.u_sw_error.value() & 0x0f);
  diagnostic_values.push_back(key_value);
  diagnostic_status.level = (status_packet.u_sw_error.value() & 0x0f) != 0
                              ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                              : diagnostic_status.level;

  key_value.key = "aln_status";
  key_value.value = std::to_string(status_packet.u_aln_status_azimuth_available & 0b00000011);
  diagnostic_values.push_back(key_value);

  key_value.key = "aln_azimuth_available";
  key_value.value =
    std::to_string((status_packet.u_aln_status_azimuth_available & 0b00000100) >> 2);
  diagnostic_values.push_back(key_value);

  key_value.key = "aln_current_azimuth_std";
  key_value.value = std::to_string(1.52593e-05 * status_packet.u_aln_current_azimuth_std.value());
  diagnostic_values.push_back(key_value);

  key_value.key = "aln_current_azimuth";
  key_value.value =
    std::to_string(9.58766e-05 * status_packet.u_aln_current_azimuth.value() - 3.14159);
  diagnostic_values.push_back(key_value);

  key_value.key = "aln_current_delta";
  key_value.value =
    std::to_string(9.58766e-05 * status_packet.u_aln_current_delta.value() - 3.14159);
  diagnostic_values.push_back(key_value);

  uint16_t computed_crc =
    crc16_packet(nebula_packets.packets[1].data.begin(), nebula_packets.packets[1].data.end() - 3);
  key_value.key = "crc_check";
  key_value.value =
    std::to_string(status_packet.u_crc.value()) + "|" + std::to_string(computed_crc);
  diagnostic_values.push_back(key_value);

  key_value.key = "sequence_counter";
  key_value.value = std::to_string(status_packet.u_sequence_counter);
  diagnostic_values.push_back(key_value);

  status_callback_(std::move(diagnostic_array_msg_ptr));

  return true;
}

}  // namespace continental_srr520
}  // namespace drivers
}  // namespace nebula
