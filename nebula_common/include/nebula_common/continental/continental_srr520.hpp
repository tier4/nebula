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

#pragma once

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>

#include "boost/endian/buffers.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
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
  uint8_t new_sensor_id;
  bool new_plug_bottom{};
  float new_longitudinal_cog{};
  float new_vehicle_wheelbase{};
  float new_cover_damping{};
  bool reset_sensor_configuration{};
};

/// @brief Convert ContinentalSRR520SensorConfiguration to string (Overloading the <<
/// operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(
  std::ostream & os, ContinentalSRR520SensorConfiguration const & arg)
{
  os << (CANSensorConfigurationBase)(arg) << ", BaseFrame: " << arg.base_frame
     << ", NewSensorId: " << arg.new_sensor_id << ", NewPlugBottom: " << arg.new_plug_bottom
     << ", NewLongitudinalCog: " << static_cast<uint16_t>(arg.new_longitudinal_cog)
     << ", NewVehicleWheelbase: " << arg.new_vehicle_wheelbase
     << ", NewCoverDamping: " << arg.new_cover_damping
     << ", ResetSensorConfiguration: " << arg.reset_sensor_configuration;
  return os;
}

/// @brief semantic struct of ARS548 Status
struct ContinentalSRR520Status
{
  uint32_t timestamp_nanoseconds;
  uint32_t timestamp_seconds;
  std::string timestamp_sync_status;
  uint8_t sw_version_major;
  uint8_t sw_version_minor;
  uint8_t sw_version_patch;
  float longitudinal;
  float lateral;
  float vertical;
  float yaw;
  float pitch;
  std::string plug_orientation;
  float length;
  float width;
  float height;
  float wheel_base;
  uint16_t max_distance;
  std::string frequency_slot;
  uint8_t cycle_time;
  uint8_t time_slot;
  std::string hcc;
  std::string power_save_standstill;
  std::string sensor_ip_address0;
  std::string sensor_ip_address1;
  uint8_t configuration_counter;
  std::string longitudinal_velocity_status;
  std::string longitudinal_acceleration_status;
  std::string lateral_acceleration_status;
  std::string yaw_rate_status;
  std::string steering_angle_status;
  std::string driving_direction_status;
  std::string characteristic_speed_status;
  std::string radar_status;
  std::string voltage_status;
  std::string temperature_status;
  std::string blockage_status;

  ContinentalSRR520Status() {}

  /// @brief Stream ContinentalRadarStatus method
  /// @param os
  /// @param arg
  /// @return stream
  friend std::ostream & operator<<(std::ostream & os, ContinentalSRR520Status const & arg)
  {
    os << "timestamp_nanoseconds: " << arg.timestamp_nanoseconds;
    os << ", ";
    os << "timestamp_seconds: " << arg.timestamp_seconds;
    os << ", ";
    os << "timestamp_sync_status: " << arg.timestamp_sync_status;
    os << ", ";
    os << "sw_version_major: " << arg.sw_version_major;
    os << ", ";
    os << "sw_version_minor: " << arg.sw_version_minor;
    os << ", ";
    os << "sw_version_patch: " << arg.sw_version_patch;
    os << ", ";
    os << "longitudinal: " << arg.longitudinal;
    os << ", ";
    os << "lateral: " << arg.lateral;
    os << ", ";
    os << "vertical: " << arg.vertical;
    os << ", ";
    os << "yaw: " << arg.yaw;
    os << ", ";
    os << "pitch: " << arg.pitch;
    os << ", ";
    os << "plug_orientation: " << arg.plug_orientation;
    os << ", ";
    os << "length: " << arg.length;
    os << ", ";
    os << "width: " << arg.width;
    os << ", ";
    os << "height: " << arg.height;
    os << ", ";
    os << "wheel_base: " << arg.wheel_base;
    os << ", ";
    os << "max_distance: " << arg.max_distance;
    os << ", ";
    os << "frequency_slot: " << arg.frequency_slot;
    os << ", ";
    os << "cycle_time: " << arg.cycle_time;
    os << ", ";
    os << "time_slot: " << arg.time_slot;
    os << ", ";
    os << "hcc: " << arg.hcc;
    os << ", ";
    os << "power_save_standstill: " << arg.power_save_standstill;
    os << ", ";
    os << "sensor_ip_address0: " << arg.sensor_ip_address0;
    os << ", ";
    os << "sensor_ip_address1: " << arg.sensor_ip_address1;
    os << ", ";
    os << "configuration_counter: " << arg.configuration_counter;
    os << ", ";
    os << "status_longitudinal_velocity: " << arg.longitudinal_velocity_status;
    os << ", ";
    os << "status_longitudinal_acceleration: " << arg.longitudinal_acceleration_status;
    os << ", ";
    os << "status_lateral_acceleration: " << arg.lateral_acceleration_status;
    os << ", ";
    os << "status_yaw_rate: " << arg.yaw_rate_status;
    os << ", ";
    os << "status_steering_angle: " << arg.steering_angle_status;
    os << ", ";
    os << "status_driving_direction: " << arg.driving_direction_status;
    os << ", ";
    os << "characteristic_speed: " << arg.characteristic_speed_status;
    os << ", ";
    os << "radar_status: " << arg.radar_status;
    os << ", ";
    os << "temperature_status: " << arg.temperature_status;
    os << ", ";
    os << "voltage_status: " << arg.voltage_status;
    os << ", ";
    os << "blockage_status: " << arg.blockage_status;

    return os;
  }
};

using boost::endian::big_float32_buf_t;
using boost::endian::big_uint16_buf_t;
using boost::endian::big_uint32_buf_t;
using boost::endian::big_uint64_buf_t;

// CAN MSG IDS
constexpr int RDI_NEAR_HEADER_CAN_MESSAGE_ID = 900;
constexpr int RDI_NEAR_ELEMENT_CAN_MESSAGE_ID = 901;
constexpr int RDI_HRR_HEADER_CAN_MESSAGE_ID = 1100;
constexpr int RDI_HRR_ELEMENT_CAN_MESSAGE_ID = 1101;
constexpr int OBJECT_HEADER_CAN_MESSAGE_ID = 1200;
constexpr int OBJECT_CAN_MESSAGE_ID = 1201;
constexpr int CRC_LIST_CAN_MESSAGE_ID = 800;
constexpr int STATUS_CAN_MESSAGE_ID = 700;
constexpr int SYNC_FUP_CAN_MESSAGE_ID = 53;
constexpr int VEH_DYN_CAN_MESSAGE_ID = 600;
constexpr int SENSOR_CONFIG_CAN_MESSAGE_ID = 601;

// CRC IDS
constexpr int NEAR_CRC_ID = 0;
constexpr int HRR_CRC_ID = 1;
constexpr int OBJECT_CRC_ID = 2;
constexpr int TIME_DOMAIN_ID = 0;

//
constexpr int RDI_NEAR_HEADER_PACKET_SIZE = 32;
constexpr int RDI_NEAR_ELEMENT_PACKET_SIZE = 64;
constexpr int RDI_HRR_HEADER_PACKET_SIZE = 32;
constexpr int RDI_HRR_ELEMENT_PACKET_SIZE = 64;
constexpr int OBJECT_HEADER_PACKET_SIZE = 32;
constexpr int OBJECT_PACKET_SIZE = 64;
constexpr int CRC_LIST_PACKET_SIZE = 4;
constexpr int STATUS_PACKET_SIZE = 64;
constexpr int CONFIGURATION_PACKET_SIZE = 16;

constexpr int RDI_NEAR_PACKET_NUM = 50;
constexpr int RDI_HRR_PACKET_NUM = 20;
constexpr int OBJECT_PACKET_NUM = 20;

constexpr int FRAGMENTS_PER_DETECTION_PACKET = 10;
constexpr int FRAGMENTS_PER_OBJECT_PACKET = 2;
constexpr int DETECTION_FRAGMENT_SIZE = 6;
constexpr int OBJECT_FRAGMENT_SIZE = 31;

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
  big_uint16_buf_t u_vambig;                  // 19
  big_uint16_buf_t u_max_range;               // 21
  big_uint16_buf_t u_number_of_detections;    // 23
  uint8_t reserved[7];                        // 25
};

struct DetectionFragmentPacket
{
  uint8_t data[DETECTION_FRAGMENT_SIZE];
};

struct DetectionPacket
{
  DetectionFragmentPacket fragments[FRAGMENTS_PER_DETECTION_PACKET];  // 0 - 59
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
  uint8_t data[OBJECT_FRAGMENT_SIZE];
};

struct ObjectPacket
{
  ObjectFragmentPacket fragments[FRAGMENTS_PER_OBJECT_PACKET];  // 0 - 59
  uint8_t reserved0;                                            // 60
  uint8_t reserved1;                                            // 61
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

struct PointSRR520Detection
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
} EIGEN_ALIGN16;

// Note we only use a subset of the data since POINT_CLOUD_REGISTER_POINT_STRUCT has a limit in the
// number of fields
struct PointSRR520Object
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
} EIGEN_ALIGN16;

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
