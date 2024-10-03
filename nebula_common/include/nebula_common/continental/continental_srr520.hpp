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

#pragma once

#include "nebula_common/nebula_common.hpp"

#include <boost/endian/buffers.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <string>

namespace nebula
{
namespace drivers
{
namespace continental_srr520
{

/// @brief struct for SRR520 sensor configuration
struct ContinentalSRR520SensorConfiguration : CANSensorConfigurationBase
{
  std::string base_frame{};
  bool sync_use_bus_time{};
  float configuration_vehicle_wheelbase{};
};

/// @brief Convert ContinentalSRR520SensorConfiguration to string (Overloading the <<
/// operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(
  std::ostream & os, ContinentalSRR520SensorConfiguration const & arg)
{
  os << "Continental SRR520 Sensor Configuration:" << '\n';
  os << (CANSensorConfigurationBase)(arg) << '\n';
  os << "Base Frame: " << arg.base_frame << '\n';
  os << "Sync Use Bus Time: " << arg.sync_use_bus_time << '\n';
  os << "Vehicle Wheelbase: " << arg.configuration_vehicle_wheelbase;
  return os;
}

using boost::endian::big_float32_buf_t;
using boost::endian::big_uint16_buf_t;
using boost::endian::big_uint32_buf_t;
using boost::endian::big_uint64_buf_t;

// CAN MSG IDS
constexpr int rdi_near_header_can_message_id = 900;
constexpr int rdi_near_element_can_message_id = 901;
constexpr int rdi_hrr_header_can_message_id = 1100;
constexpr int rdi_hrr_element_can_message_id = 1101;
constexpr int object_header_can_message_id = 1200;
constexpr int object_can_message_id = 1201;
constexpr int crc_list_can_message_id = 800;
constexpr int status_can_message_id = 700;
constexpr int sync_follow_up_can_message_id = 53;
constexpr int veh_dyn_can_message_id = 600;
constexpr int sensor_config_can_message_id = 601;

// CRC IDS
constexpr int near_crc_id = 0;
constexpr int hrr_crc_id = 1;
constexpr int object_crc_id = 2;
constexpr int time_domain_id =
  0;  // For details, please refer to AUTOSAR's "Time Synchronization over CAN" document

constexpr int rdi_near_header_packet_size = 32;
constexpr int rdi_near_element_packet_size = 64;
constexpr int rdi_hrr_header_packet_size = 32;
constexpr int rdi_hrr_element_packet_size = 64;
constexpr int object_header_packet_size = 32;
constexpr int object_packet_size = 64;
constexpr int crc_list_packet_size = 4;
constexpr int status_packet_size = 64;
constexpr int sync_follow_up_can_packet_size = 8;
constexpr int veh_dyn_can_packet_size = 8;
constexpr int configuration_packet_size = 16;

constexpr int rdi_near_packet_num = 50;
constexpr int rdi_hrr_packet_num = 20;
constexpr int object_packet_num = 20;

constexpr int fragments_per_detection_packet = 10;
constexpr int fragments_per_object_packet = 2;
constexpr int detection_fragment_size = 6;
constexpr int object_fragment_size = 31;

constexpr int max_rdi_near_detections = rdi_near_packet_num * fragments_per_detection_packet;
constexpr int max_rdi_hrr_detections = rdi_hrr_packet_num * fragments_per_detection_packet;
constexpr int max_objects = object_packet_num * fragments_per_object_packet;

#pragma pack(push, 1)

struct StatusPacket
{
  big_uint32_buf_t u_time_stamp;               // 0
  big_uint32_buf_t u_global_time_stamp_sec;    // 4
  big_uint32_buf_t u_global_time_stamp_nsec;   // 8
  uint8_t u_global_time_stamp_sync_status;     // 12
  uint8_t u_sw_version_major;                  // 13
  uint8_t u_sw_version_minor;                  // 14
  uint8_t u_sw_version_patch;                  // 15
  uint8_t u_sensor_id;                         // 16
  big_uint16_buf_t u_long_pos;                 // 17
  big_uint16_buf_t u_lat_pos;                  // 19
  big_uint16_buf_t u_vert_pos;                 // 21
  big_uint16_buf_t u_long_pos_cog;             // 23
  big_uint16_buf_t u_wheelbase;                // 25
  big_uint16_buf_t u_yaw_angle;                // 27
  big_uint16_buf_t u_cover_damping;            // 29
  uint8_t u_plug_orientation;                  // 31
  uint8_t u_defective;                         // 32
  uint8_t u_supply_voltage_limit;              // 33
  uint8_t u_sensor_off_temp;                   // 34
  uint8_t u_dynamics_invalid0;                 // 35
  uint8_t u_dynamics_invalid1;                 // 36
  uint8_t u_ext_disturbed;                     // 37
  uint8_t u_com_error;                         // 38
  big_uint16_buf_t u_sw_error;                 // 39
  uint8_t u_aln_status_azimuth_available;      // 41
  big_uint16_buf_t u_aln_current_azimuth_std;  // 42
  big_uint16_buf_t u_aln_current_azimuth;      // 44
  big_uint16_buf_t u_aln_current_delta;        // 46
  uint8_t reserved1[13];                       // Reserved fields, no change needed
  big_uint16_buf_t u_crc;                      // 61
  uint8_t u_sequence_counter;                  // 63
};

struct ScanHeaderPacket
{
  big_uint32_buf_t u_time_stamp;              // 0
  big_uint32_buf_t u_global_time_stamp_sec;   // 4
  big_uint32_buf_t u_global_time_stamp_nsec;  // 8
  uint8_t u_global_time_stamp_sync_status;    // 12
  uint8_t u_signal_status;                    // 13
  uint8_t u_sequence_counter;                 // 14
  big_uint32_buf_t u_cycle_counter;           // 15
  big_uint16_buf_t u_v_ambiguous;             // 19
  big_uint16_buf_t u_max_range;               // 21
  big_uint16_buf_t u_number_of_detections;    // 23
  uint8_t reserved[7];                        // 25
};

struct DetectionFragmentPacket
{
  uint8_t data[detection_fragment_size];
};

struct DetectionPacket
{
  DetectionFragmentPacket fragments[fragments_per_detection_packet];  // 0 - 59
  uint8_t reserved0;                                                  // 60
  uint8_t reserved1;                                                  // 61
  uint8_t u_message_counter;                                          // 62
  uint8_t u_sequence_counter;                                         // 63
};

struct ObjectHeaderPacket
{
  big_uint32_buf_t u_time_stamp;              // 0
  big_uint32_buf_t u_global_time_stamp_sec;   // 4
  big_uint32_buf_t u_global_time_stamp_nsec;  // 8
  uint8_t u_global_time_stamp_sync_status;    // 12
  uint8_t u_signal_status;                    // 13
  uint8_t u_sequence_counter;                 // 14
  big_uint32_buf_t u_cycle_counter;           // 15
  big_uint16_buf_t u_ego_vx;                  // 19
  big_uint16_buf_t u_ego_yaw_rate;            // 21
  uint8_t u_motion_type;                      // 23
  uint8_t u_number_of_objects;                // 24
  uint8_t reserved[7];                        // 25
};

struct ObjectFragmentPacket
{
  uint8_t data[object_fragment_size];
};

struct ObjectPacket
{
  ObjectFragmentPacket fragments[fragments_per_object_packet];  // 0 - 61
  uint8_t u_message_counter;                                    // 62
  uint8_t u_sequence_counter;                                   // 63
};

struct ConfigurationPacket
{
  uint8_t u_sensor_id;
  big_uint16_buf_t u_long_pose;
  big_uint16_buf_t u_lat_pose;
  big_uint16_buf_t u_vert_pos;
  big_uint16_buf_t u_long_pos_cog;
  big_uint16_buf_t u_wheelbase;
  big_uint16_buf_t u_yaw_angle;
  big_uint16_buf_t u_cover_damping;
  uint8_t u_plug_orientation_and_default_settings;
};

struct SyncPacket
{
  uint8_t u_type;
  uint8_t u_crc;
  uint8_t u_time_domain_and_sequence_counter;
  uint8_t u_user_data0;
  big_uint32_buf_t u_sync_time_sec;
};

struct FollowUpPacket
{
  uint8_t u_type;
  uint8_t u_crc;
  uint8_t u_time_domain_and_sequence_counter;
  uint8_t u_reserved;
  big_uint32_buf_t u_sync_time_nsec;
};

#pragma pack(pop)

struct EIGEN_ALIGN16 PointSRR520Detection
{
  PCL_ADD_POINT4D;
  float range;
  float azimuth;
  float range_rate;
  float rcs;
  uint8_t pdh00;
  uint8_t pdh01;
  uint8_t pdh02;
  uint8_t pdh03;
  uint8_t pdh04;
  uint8_t pdh05;
  float snr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Note we only use a subset of the data since POINT_CLOUD_REGISTER_POINT_STRUCT has a limit in the
// number of fields
struct EIGEN_ALIGN16 PointSRR520Object
{
  PCL_ADD_POINT4D;
  uint32_t id;
  uint16_t age;
  float orientation;
  float rcs;
  float score;
  uint8_t object_status;
  float dynamics_abs_vel_x;
  float dynamics_abs_vel_y;
  float dynamics_abs_acc_x;
  float dynamics_abs_acc_y;
  float box_length;
  float box_width;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace continental_srr520
}  // namespace drivers
}  // namespace nebula

POINT_CLOUD_REGISTER_POINT_STRUCT(
  nebula::drivers::continental_srr520::PointSRR520Detection,
  (float, x, x)(float, y, y)(float, z, z)(float, azimuth, azimuth)(float, range, range)(
    float, range_rate, range_rate)(int8_t, rcs, rcs)(uint8_t, pdh00, pdh00)(uint8_t, pdh01, pdh01)(
    uint8_t, pdh02,
    pdh02)(uint16_t, pdh03, pdh03)(uint8_t, pdh04, pdh04)(uint8_t, pdh05, pdh05)(float, snr, snr))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  nebula::drivers::continental_srr520::PointSRR520Object,
  (float, x, x)(float, y, y)(float, z, z)(uint32_t, id, id)(uint16_t, age, age)(
    float, orientation,
    orientation)(float, rcs, rcs)(float, score, score)(uint8_t, object_status, object_status)(
    float, dynamics_abs_vel_x, dynamics_abs_vel_x)(float, dynamics_abs_vel_y, dynamics_abs_vel_y)(
    float, dynamics_abs_acc_x, dynamics_abs_acc_x)(float, dynamics_abs_acc_y, dynamics_abs_acc_y)(
    float, box_length, box_length)(float, box_width, box_width))
