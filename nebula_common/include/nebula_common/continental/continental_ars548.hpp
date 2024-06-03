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
#include <boost/algorithm/string/join.hpp>
#include <boost/format.hpp>

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
#include <vector>

namespace nebula
{
namespace drivers
{
namespace continental_ars548
{

/// @brief struct for ARS548 sensor configuration
struct ContinentalARS548SensorConfiguration : EthernetSensorConfigurationBase
{
  std::string multicast_ip{};
  std::string base_frame{};
  std::string object_frame{};
  uint16_t configuration_host_port{};
  uint16_t configuration_sensor_port{};
  bool use_sensor_time{};
  float configuration_vehicle_length{};
  float configuration_vehicle_width{};
  float configuration_vehicle_height{};
  float configuration_vehicle_wheelbase{};
};

/// @brief struct for Multiple ARS548 sensor configuration
struct MultiContinentalARS548SensorConfiguration : ContinentalARS548SensorConfiguration
{
  std::vector<std::string> sensor_ips{};
  std::vector<std::string> frame_ids{};
};

/// @brief Convert ContinentalARS548SensorConfiguration to string (Overloading the <<
/// operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(
  std::ostream & os, ContinentalARS548SensorConfiguration const & arg)
{
  os << (EthernetSensorConfigurationBase)(arg) << ", MulticastIP: " << arg.multicast_ip
     << ", BaseFrame: " << arg.base_frame << ", ObjectFrame: " << arg.object_frame
     << ", ConfigurationHostPort: " << arg.configuration_host_port
     << ", ConfigurationSensorPort: " << arg.configuration_sensor_port
     << ", UseSensorTime: " << arg.use_sensor_time
     << ", ConfigurationVehicleLength: " << arg.configuration_vehicle_length
     << ", ConfigurationVehicleWidth: " << arg.configuration_vehicle_width
     << ", ConfigurationVehicleHeight: " << arg.configuration_vehicle_height
     << ", ConfigurationVehicleWheelbase: " << arg.configuration_vehicle_wheelbase;
  return os;
}

/// @brief Convert MultiContinentalARS548SensorConfiguration to string (Overloading the <<
/// operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(
  std::ostream & os, MultiContinentalARS548SensorConfiguration const & arg)
{
  std::stringstream sensor_ips_ss;
  sensor_ips_ss << "[";
  for (const auto sensor_ip : arg.sensor_ips) {
    sensor_ips_ss << sensor_ip << ", ";
  }
  sensor_ips_ss << "]";

  std::stringstream frame_ids_ss;
  frame_ids_ss << "[";
  for (const auto frame_id : arg.frame_ids) {
    frame_ids_ss << frame_id << ", ";
  }
  frame_ids_ss << "]";

  os << (ContinentalARS548SensorConfiguration)(arg) << ", MulticastIP: " << arg.multicast_ip
     << ", SensorIPs: " << sensor_ips_ss.str() << ", FrameIds: " << frame_ids_ss.str();
  return os;
}

/// @brief semantic struct of ARS548 Status
struct ContinentalARS548Status
{
  // Filled with raw sensor data
  uint32_t timestamp_nanoseconds{};
  uint32_t timestamp_seconds{};
  std::string timestamp_sync_status{};
  uint8_t sw_version_major{};
  uint8_t sw_version_minor{};
  uint8_t sw_version_patch{};
  float longitudinal{};
  float lateral{};
  float vertical{};
  float yaw{};
  float pitch{};
  std::string plug_orientation{};
  float length{};
  float width{};
  float height{};
  float wheel_base{};
  uint16_t max_distance{};
  std::string frequency_slot{};
  uint8_t cycle_time{};
  uint8_t time_slot{};
  std::string hcc{};
  std::string power_save_standstill{};
  std::string sensor_ip_address0{};
  std::string sensor_ip_address1{};
  uint8_t configuration_counter{};
  std::string longitudinal_velocity_status{};
  std::string longitudinal_acceleration_status{};
  std::string lateral_acceleration_status{};
  std::string yaw_rate_status{};
  std::string steering_angle_status{};
  std::string driving_direction_status{};
  std::string characteristic_speed_status{};
  std::string radar_status{};
  std::string voltage_status{};
  std::string temperature_status{};
  std::string blockage_status{};

  // Processed data
  uint64_t detection_first_stamp{};
  uint64_t detection_last_stamp{};
  uint64_t detection_total_count{};
  uint64_t detection_dropped_dt_count{};
  uint64_t detection_empty_count{};

  uint64_t object_first_stamp{};
  uint64_t object_last_stamp{};
  uint64_t object_total_count{};
  uint64_t object_dropped_dt_count{};
  uint64_t object_empty_count{};

  uint64_t status_total_count{};
  uint64_t radar_invalid_count{};

  ContinentalARS548Status() {}

  /// @brief Stream ContinentalRadarStatus method
  /// @param os
  /// @param arg
  /// @return stream
  friend std::ostream & operator<<(std::ostream & os, ContinentalARS548Status const & arg)
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

constexpr int CONFIGURATION_SERVICE_ID = 0;
constexpr int CONFIGURATION_METHOD_ID = 390;
constexpr int CONFIGURATION_PAYLOAD_LENGTH = 56;
constexpr int CONFIGURATION_UDP_LENGTH = 64;

constexpr int DETECTION_LIST_METHOD_ID = 336;
constexpr int OBJECT_LIST_METHOD_ID = 329;
constexpr int SENSOR_STATUS_METHOD_ID = 380;
constexpr int FILTER_STATUS_METHOD_ID = 396;

constexpr int DETECTION_LIST_UDP_PAYLOAD = 35336;
constexpr int OBJECT_LIST_UDP_PAYLOAD = 9401;
constexpr int SENSOR_STATUS_UDP_PAYLOAD = 84;
constexpr int FILTER_STATUS_UDP_PAYLOAD = 330;

constexpr int DETECTION_LIST_PDU_LENGTH = 35328;
constexpr int OBJECT_LIST_PDU_LENGTH = 9393;
constexpr int SENSOR_STATUS_PDU_LENGTH = 76;
constexpr int FILTER_STATUS_PDU_LENGTH = 322;

constexpr int DETECTION_FILTER_PROPERTIES_NUM = 7;
constexpr int OBJECT_FILTER_PROPERTIES_NUM = 24;
constexpr int MAX_DETECTIONS = 800;
constexpr int MAX_OBJECTS = 50;

constexpr int SYNC_OK = 1;
constexpr int NEVER_SYNC = 2;
constexpr int SYNC_LOST = 3;

constexpr int PLUG_RIGHT = 0;
constexpr int PLUG_LEFT = 1;

constexpr int FREQUENCY_SLOT_LOW = 0;
constexpr int FREQUENCY_SLOT_MID = 1;
constexpr int FREQUENCY_SLOT_HIGH = 2;

constexpr int HCC_WORLDWIDE = 1;
constexpr int HCC_JAPAN = 2;

constexpr int POWERSAVE_STANDSTILL_OFF = 0;
constexpr int POWERSAVE_STANDSTILL_ON = 1;

constexpr int VDY_OK = 0;
constexpr int VDY_NOTOK = 1;

constexpr int STATE_INIT = 0;
constexpr int STATE_OK = 1;
constexpr int STATE_INVALID = 2;

constexpr int BLOCKAGE_STATUS_BLIND = 0;
constexpr int BLOCKAGE_STATUS_HIGH = 1;
constexpr int BLOCKAGE_STATUS_MID = 2;
constexpr int BLOCKAGE_STATUS_LOW = 3;
constexpr int BLOCKAGE_STATUS_NONE = 4;

constexpr int BLOCKAGE_TEST_FAILED = 0;
constexpr int BLOCKAGE_TEST_PASSED = 1;
constexpr int BLOCKAGE_TEST_ONGOING = 2;

#pragma pack(push, 1)

struct HeaderPacket
{
  big_uint16_buf_t service_id{};
  big_uint16_buf_t method_id{};
  big_uint32_buf_t length{};
};

struct HeaderSomeIPPacket
{
  big_uint16_buf_t client_id;
  big_uint16_buf_t session_id;
  uint8_t protocol_version;
  uint8_t interface_version;
  uint8_t message_type;
  uint8_t return_code;
};

struct HeaderE2EP07Packet
{
  big_uint64_buf_t crc;
  big_uint32_buf_t length;
  big_uint32_buf_t sqc;
  big_uint32_buf_t data_id;
};

struct StampSyncStatusPacket
{
  big_uint32_buf_t timestamp_nanoseconds;
  big_uint32_buf_t timestamp_seconds;
  uint8_t timestamp_sync_status;
};

struct DetectionPacket
{
  big_float32_buf_t azimuth_angle;
  big_float32_buf_t azimuth_angle_std;
  uint8_t invalid_flags;
  big_float32_buf_t elevation_angle;
  big_float32_buf_t elevation_angle_std;
  big_float32_buf_t range;
  big_float32_buf_t range_std;
  big_float32_buf_t range_rate;
  big_float32_buf_t range_rate_std;
  int8_t rcs;
  big_uint16_buf_t measurement_id;
  uint8_t positive_predictive_value;
  uint8_t classification;
  uint8_t multi_target_probability;
  big_uint16_buf_t object_id;
  uint8_t ambiguity_flag;
  big_uint16_buf_t sort_index;
};

struct DetectionListPacket
{
  HeaderPacket header;
  HeaderSomeIPPacket header_some_ip;
  HeaderE2EP07Packet header_e2ep07;
  StampSyncStatusPacket stamp;
  big_uint32_buf_t event_data_qualifier;
  uint8_t extended_qualifier;
  big_uint16_buf_t origin_invalid_flags;
  big_float32_buf_t origin_x_pos;
  big_float32_buf_t origin_x_std;
  big_float32_buf_t origin_y_pos;
  big_float32_buf_t origin_y_std;
  big_float32_buf_t origin_z_pos;
  big_float32_buf_t origin_z_std;
  big_float32_buf_t origin_roll;
  big_float32_buf_t origin_roll_std;
  big_float32_buf_t origin_pitch;
  big_float32_buf_t origin_pitch_std;
  big_float32_buf_t origin_yaw;
  big_float32_buf_t origin_yaw_std;
  uint8_t list_invalid_flags;
  DetectionPacket detections[MAX_DETECTIONS];
  big_float32_buf_t list_rad_vel_domain_min;
  big_float32_buf_t list_rad_vel_domain_max;
  big_uint32_buf_t number_of_detections;
  big_float32_buf_t alignment_azimuth_correction;
  big_float32_buf_t alignment_elevation_correction;
  uint8_t alignment_status;
  uint8_t reserved[14];
};

struct ObjectPacket
{
  big_uint16_buf_t status_sensor;
  big_uint32_buf_t id;
  big_uint16_buf_t age;
  uint8_t status_measurement;
  uint8_t status_movement;
  big_uint16_buf_t position_invalid_flags;
  uint8_t position_reference;
  big_float32_buf_t position_x;
  big_float32_buf_t position_x_std;
  big_float32_buf_t position_y;
  big_float32_buf_t position_y_std;
  big_float32_buf_t position_z;
  big_float32_buf_t position_z_std;
  big_float32_buf_t position_covariance_xy;
  big_float32_buf_t position_orientation;
  big_float32_buf_t position_orientation_std;
  uint8_t existence_invalid_flags;
  big_float32_buf_t existence_probability;
  big_float32_buf_t existence_ppv;
  uint8_t classification_car;
  uint8_t classification_truck;
  uint8_t classification_motorcycle;
  uint8_t classification_bicycle;
  uint8_t classification_pedestrian;
  uint8_t classification_animal;
  uint8_t classification_hazard;
  uint8_t classification_unknown;
  uint8_t classification_overdrivable;
  uint8_t classification_underdrivable;
  uint8_t dynamics_abs_vel_invalid_flags;
  big_float32_buf_t dynamics_abs_vel_x;
  big_float32_buf_t dynamics_abs_vel_x_std;
  big_float32_buf_t dynamics_abs_vel_y;
  big_float32_buf_t dynamics_abs_vel_y_std;
  big_float32_buf_t dynamics_abs_vel_covariance_xy;
  uint8_t dynamics_rel_vel_invalid_flags;
  big_float32_buf_t dynamics_rel_vel_x;
  big_float32_buf_t dynamics_rel_vel_x_std;
  big_float32_buf_t dynamics_rel_vel_y;
  big_float32_buf_t dynamics_rel_vel_y_std;
  big_float32_buf_t dynamics_rel_vel_covariance_xy;
  uint8_t dynamics_abs_accel_invalid_flags;
  big_float32_buf_t dynamics_abs_accel_x;
  big_float32_buf_t dynamics_abs_accel_x_std;
  big_float32_buf_t dynamics_abs_accel_y;
  big_float32_buf_t dynamics_abs_accel_y_std;
  big_float32_buf_t dynamics_abs_accel_covariance_xy;
  uint8_t dynamics_rel_accel_invalid_flags;
  big_float32_buf_t dynamics_rel_accel_x;
  big_float32_buf_t dynamics_rel_accel_x_std;
  big_float32_buf_t dynamics_rel_accel_y;
  big_float32_buf_t dynamics_rel_accel_y_std;
  big_float32_buf_t dynamics_rel_accel_covariance_xy;
  uint8_t dynamics_orientation_invalid_flags;
  big_float32_buf_t dynamics_orientation_rate_mean;
  big_float32_buf_t dynamics_orientation_rate_std;
  big_uint32_buf_t shape_length_status;
  uint8_t shape_length_edge_invalid_flags;
  big_float32_buf_t shape_length_edge_mean;
  big_float32_buf_t shape_length_edge_std;
  big_uint32_buf_t shape_width_status;
  uint8_t shape_width_edge_invalid_flags;
  big_float32_buf_t shape_width_edge_mean;
  big_float32_buf_t shape_width_edge_std;
};

struct ObjectListPacket
{
  HeaderPacket header;
  HeaderSomeIPPacket header_some_ip;
  HeaderE2EP07Packet header_e2ep07;
  StampSyncStatusPacket stamp;
  big_uint32_buf_t event_data_qualifier;
  uint8_t extended_qualifier;
  uint8_t number_of_objects;
  ObjectPacket objects[MAX_OBJECTS];
};

struct StatusConfigurationPacket
{
  big_float32_buf_t longitudinal{};
  big_float32_buf_t lateral{};
  big_float32_buf_t vertical{};
  big_float32_buf_t yaw{};
  big_float32_buf_t pitch{};
  uint8_t plug_orientation{};
  big_float32_buf_t length{};
  big_float32_buf_t width{};
  big_float32_buf_t height{};
  big_float32_buf_t wheelbase{};
  big_uint16_buf_t maximum_distance{};
  uint8_t frequency_slot{};
  uint8_t cycle_time{};
  uint8_t time_slot{};
  uint8_t hcc{};
  uint8_t powersave_standstill{};
  uint8_t sensor_ip_address00{};
  uint8_t sensor_ip_address01{};
  uint8_t sensor_ip_address02{};
  uint8_t sensor_ip_address03{};
  uint8_t sensor_ip_address10{};
  uint8_t sensor_ip_address11{};
  uint8_t sensor_ip_address12{};
  uint8_t sensor_ip_address13{};
};

struct SensorStatusPacket
{
  HeaderPacket header;
  StampSyncStatusPacket stamp;
  uint8_t sw_version_major;
  uint8_t sw_version_minor;
  uint8_t sw_version_patch;
  StatusConfigurationPacket status;
  uint8_t configuration_counter;
  uint8_t longitudinal_velocity_status;
  uint8_t longitudinal_acceleration_status;
  uint8_t lateral_acceleration_status;
  uint8_t yaw_rate_status;
  uint8_t steering_angle_status;
  uint8_t driving_direction_status;
  uint8_t characteristic_speed_status;
  uint8_t radar_status;
  uint8_t voltage_status;
  uint8_t temperature_status;
  uint8_t blockage_status;
};

struct ConfigurationPacket
{
  HeaderPacket header{};
  StatusConfigurationPacket configuration{};
  uint8_t new_sensor_mounting{};
  uint8_t new_vehicle_parameters{};
  uint8_t new_radar_parameters{};
  uint8_t new_network_configuration{};
};

struct AccelerationLateralCoGPacket
{
  HeaderPacket header;
  uint8_t reserved0[6];
  big_float32_buf_t acceleration_lateral;
  uint8_t reserved1[22];
};

struct AccelerationLongitudinalCoGPacket
{
  HeaderPacket header;
  uint8_t reserved0[6];
  big_float32_buf_t acceleration_lateral;
  uint8_t reserved1[22];
};

struct CharacteristicSpeedPacket
{
  HeaderPacket header;
  uint8_t reserved0[2];
  uint8_t characteristic_speed;
  uint8_t reserved1[8];
};

struct DrivingDirectionPacket
{
  HeaderPacket header;
  uint8_t reserved0;
  uint8_t driving_direction;
  uint8_t reserved1[20];
};

struct SteeringAngleFrontAxlePacket
{
  HeaderPacket header;
  uint8_t reserved0[6];
  big_float32_buf_t steering_angle_front_axle;
  uint8_t reserved1[22];
};

struct VelocityVehiclePacket
{
  HeaderPacket header;
  uint8_t reserved0[3];
  big_float32_buf_t velocity_vehicle;
  uint8_t reserved1[21];
};

struct YawRatePacket
{
  HeaderPacket header;
  uint8_t reserved0[6];
  big_float32_buf_t yaw_rate;
  uint8_t reserved1[22];
};

struct FilterStatusEntryPacket
{
  uint8_t active;
  uint8_t data_index;
  big_float32_buf_t min_value;
  big_float32_buf_t max_value;
};

struct FilterStatusPacket
{
  HeaderPacket header;
  StampSyncStatusPacket stamp;
  uint8_t filter_configuration_counter;
  uint8_t detection_sort_index;
  uint8_t object_sort_index;
  FilterStatusEntryPacket detection_filters[DETECTION_FILTER_PROPERTIES_NUM];
  FilterStatusEntryPacket object_filters[OBJECT_FILTER_PROPERTIES_NUM];
};

#pragma pack(pop)

struct EIGEN_ALIGN16 PointARS548Detection
{
  PCL_ADD_POINT4D;
  float azimuth;
  float azimuth_std;
  float elevation;
  float elevation_std;
  float range;
  float range_std;
  float range_rate;
  float range_rate_std;
  int8_t rcs;
  uint16_t measurement_id;
  uint8_t positive_predictive_value;
  uint8_t classification;
  uint8_t multi_target_probability;
  uint16_t object_id;
  uint8_t ambiguity_flag;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Note we only use a subset of the data since POINT_CLOUD_REGISTER_POINT_STRUCT has a limit in the
// number of fields
struct EIGEN_ALIGN16 PointARS548Object
{
  PCL_ADD_POINT4D;
  uint32_t id;
  uint16_t age;
  uint8_t status_measurement;
  uint8_t status_movement;
  uint8_t position_reference;
  uint8_t classification_car;
  uint8_t classification_truck;
  uint8_t classification_motorcycle;
  uint8_t classification_bicycle;
  uint8_t classification_pedestrian;
  float dynamics_abs_vel_x;
  float dynamics_abs_vel_y;
  float dynamics_rel_vel_x;
  float dynamics_rel_vel_y;
  float shape_length_edge_mean;
  float shape_width_edge_mean;
  float dynamics_orientation_rate_mean;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace continental_ars548
}  // namespace drivers
}  // namespace nebula

POINT_CLOUD_REGISTER_POINT_STRUCT(
  nebula::drivers::continental_ars548::PointARS548Detection,
  (float, x, x)(float, y, y)(float, z, z)(float, azimuth, azimuth)(float, azimuth_std, azimuth_std)(
    float, elevation, elevation)(float, elevation_std, elevation_std)(float, range, range)(
    float, range_std, range_std)(int8_t, rcs, rcs)(uint16_t, measurement_id, measurement_id)(
    uint8_t, positive_predictive_value,
    positive_predictive_value)(uint8_t, classification, classification)(
    uint8_t, multi_target_probability, multi_target_probability)(uint16_t, object_id, object_id)(
    uint8_t, ambiguity_flag, ambiguity_flag))

// Note: we can only use up to 20 fields
POINT_CLOUD_REGISTER_POINT_STRUCT(
  nebula::drivers::continental_ars548::PointARS548Object,
  (float, x, x)(float, y, y)(float, z, z)(uint32_t, id, id)(uint16_t, age, age)(
    uint8_t, status_measurement, status_measurement)(uint8_t, status_movement, status_movement)(
    uint8_t, position_reference,
    position_reference)(uint8_t, classification_car, classification_car)(
    uint8_t, classification_truck,
    classification_truck)(uint8_t, classification_motorcycle, classification_motorcycle)(
    uint8_t, classification_bicycle,
    classification_bicycle)(uint8_t, classification_pedestrian, classification_pedestrian)(
    float, dynamics_abs_vel_x, dynamics_abs_vel_x)(float, dynamics_abs_vel_y, dynamics_abs_vel_y)(
    float, dynamics_rel_vel_x, dynamics_rel_vel_x)(float, dynamics_rel_vel_y, dynamics_rel_vel_y)(
    float, shape_length_edge_mean,
    shape_length_edge_mean)(float, shape_width_edge_mean, shape_width_edge_mean)(
    float, dynamics_orientation_rate_mean, dynamics_orientation_rate_mean))
