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

#include <boost/algorithm/string/join.hpp>
#include <boost/endian/buffers.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <optional>
#include <string>

namespace nebula
{
namespace drivers
{
namespace continental_ars548
{

inline bool is_corner_radar(float yaw)
{
  return std::abs(yaw) > deg2rad(5.0) && std::abs(yaw) < deg2rad(90.0);
}

/// @brief struct for ARS548 sensor configuration
struct ContinentalARS548SensorConfiguration : EthernetSensorConfigurationBase
{
  std::string multicast_ip{};
  std::string base_frame{};
  std::string object_frame{};
  uint16_t configuration_host_port{};
  uint16_t configuration_sensor_port{};
  bool use_sensor_time{};
  int radar_info_rate_subsample{};
  float configuration_vehicle_length{};
  float configuration_vehicle_width{};
  float configuration_vehicle_height{};
  float configuration_vehicle_wheelbase{};
};

/// @brief Convert ContinentalARS548SensorConfiguration to string (Overloading the <<
/// operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(
  std::ostream & os, ContinentalARS548SensorConfiguration const & arg)
{
  os << "Continental ARS548 Sensor Configuration:" << '\n';
  os << (EthernetSensorConfigurationBase)(arg) << '\n';
  os << "Multicast IP: " << arg.multicast_ip << '\n';
  os << "Base Frame: " << arg.base_frame << '\n';
  os << "Object Frame: " << arg.object_frame << '\n';
  os << "Host Port: " << arg.configuration_host_port << '\n';
  os << "Sensor Port: " << arg.configuration_sensor_port << '\n';
  os << "UseSensor Time: " << arg.use_sensor_time << '\n';
  os << "RadarInfo Rate Subsample: " << arg.radar_info_rate_subsample << '\n';
  os << "Vehicle Length: " << arg.configuration_vehicle_length << '\n';
  os << "Vehicle Width: " << arg.configuration_vehicle_width << '\n';
  os << "Vehicle Height: " << arg.configuration_vehicle_height << '\n';
  os << "Vehicle Wheelbase: " << arg.configuration_vehicle_wheelbase;
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

constexpr int configuration_service_id = 0;
constexpr int configuration_method_id = 390;
constexpr int configuration_payload_length = 56;
constexpr int configuration_udp_length = 64;

constexpr int detection_list_method_id = 336;
constexpr int object_list_method_id = 329;
constexpr int sensor_status_method_id = 380;
constexpr int filter_status_method_id = 396;

constexpr int detection_list_udp_payload = 35336;
constexpr int object_list_udp_payload = 9401;
constexpr int sensor_status_udp_payload = 84;
constexpr int filter_status_udp_payload = 330;

constexpr int detection_list_pdu_length = 35328;
constexpr int object_list_pdu_length = 9393;
constexpr int sensor_status_pdu_length = 76;
constexpr int filter_status_pdu_length = 322;

constexpr int detection_filter_properties_num = 7;
constexpr int object_filter_properties_num = 24;
constexpr int max_detections = 800;
constexpr int max_objects = 50;

constexpr int measurement_status_measured = 0;
constexpr int measurement_status_predicted = 1;
constexpr int measurement_status_new = 2;
constexpr int measurement_status_invalid = 255;

constexpr int movement_status_dynamic = 0;
constexpr int movement_status_static = 1;
constexpr int movement_status_invalid = 255;

constexpr int sync_ok = 1;
constexpr int never_sync = 2;
constexpr int sync_lost = 3;

constexpr int plug_right = 0;
constexpr int plug_left = 1;

constexpr float raw_prob_norm = 100.f;

constexpr int maximum_distance_min_value = 93;
constexpr int maximum_distance_max_value = 1514;

constexpr int frequency_slot_low = 0;
constexpr int frequency_slot_mid = 1;
constexpr int frequency_slot_high = 2;

constexpr int min_cycle_time_ms = 50;
constexpr int max_cycle_time_ms = 100;

constexpr int min_time_slot_ms = 10;
constexpr int max_time_slot_ms = 90;

constexpr int hcc_worldwide = 1;
constexpr int hcc_japan = 2;

constexpr int powersave_standstill_off = 0;
constexpr int powersave_standstill_on = 1;

constexpr int vdy_ok = 0;
constexpr int vdy_notok = 1;

constexpr int state_init = 0;
constexpr int state_ok = 1;
constexpr int state_invalid = 2;

constexpr int blockage_status_blind = 0;
constexpr int blockage_status_high = 1;
constexpr int blockage_status_mid = 2;
constexpr int blockage_status_low = 3;
constexpr int blockage_status_none = 4;

constexpr int blockage_test_failed = 0;
constexpr int blockage_test_passed = 1;
constexpr int blockage_test_ongoing = 2;

constexpr int min_odometry_hz = 10;
constexpr int max_odometry_hz = 50;

struct FieldInfo
{
  FieldInfo(
    std::optional<float> min_value, std::optional<float> max_value,
    std::optional<float> resolution) noexcept
  : min_value(min_value), max_value(max_value), resolution(resolution)
  {
  }
  std::optional<float> min_value;
  std::optional<float> max_value;
  std::optional<float> resolution;
};

// Detection field infos
const FieldInfo azimuth_info{-M_PI, M_PI, std::nullopt};
const FieldInfo azimuth_std_info{0.f, 1.f, std::nullopt};
const FieldInfo elevation_info{-M_PI, M_PI, std::nullopt};
const FieldInfo elevation_std_info{0.f, 1.f, std::nullopt};

const FieldInfo range_info{0.f, 301.f, std::nullopt};
const FieldInfo range_std_info{0.f, 1.f, std::nullopt};
const FieldInfo range_rate_info{-100.f, 100.f, 0.f};
const FieldInfo range_rate_std_info{0.f, 1.f, std::nullopt};

const FieldInfo rcs_info{-128.f, 127.f, 1.f};
const FieldInfo measurement_id_info{0.f, 65535.f, 1.f};
const FieldInfo positive_predictive_value_info{0.f, 100.f, 1.f};
const FieldInfo classification_info{0.f, 255.f, 1.f};
const FieldInfo multi_target_probability_info{0.f, 1.f, 0.01f};
const FieldInfo object_id_info{0.f, 65535.f, 1.f};
const FieldInfo ambiguity_flag_info{0.f, 1.f, 0.01f};

// Object field infos
const FieldInfo age_info{0.f, 65535.f, 1.f};
const FieldInfo measurement_status_info{0.f, 255.f, 1.f};
const FieldInfo movement_status_info{0.f, 255.f, 1.f};

const FieldInfo position_x_info{-1600.f, 1600.f, std::nullopt};
const FieldInfo position_y_info{-1600.f, 1600.f, std::nullopt};
const FieldInfo position_z_info{-1600.f, 1600.f, std::nullopt};

const FieldInfo velocity_x_info{std::nullopt, std::nullopt, std::nullopt};
const FieldInfo velocity_y_info{std::nullopt, std::nullopt, std::nullopt};

const FieldInfo acceleration_x_info{std::nullopt, std::nullopt, std::nullopt};
const FieldInfo acceleration_y_info{std::nullopt, std::nullopt, std::nullopt};

const FieldInfo size_x_info{std::nullopt, std::nullopt, std::nullopt};
const FieldInfo size_y_info{std::nullopt, std::nullopt, std::nullopt};

const FieldInfo orientation_info{-M_PI, M_PI, std::nullopt};
const FieldInfo orientation_std_info{0.f, std::nullopt, std::nullopt};
const FieldInfo orientation_rate_info{std::nullopt, std::nullopt, std::nullopt};
const FieldInfo orientation_rate_std_info{0.f, std::nullopt, std::nullopt};

const FieldInfo existence_probability_info{0.f, 1.f, 0.01f};

#pragma pack(push, 1)

struct HeaderPacket
{
  big_uint16_buf_t service_id{};
  big_uint16_buf_t method_id{};
  big_uint32_buf_t length{};
};

struct HeaderSomeIPPacket
{
  big_uint16_buf_t client_id{};
  big_uint16_buf_t session_id{};
  uint8_t protocol_version{};
  uint8_t interface_version{};
  uint8_t message_type{};
  uint8_t return_code{};
};

struct HeaderE2EP07Packet
{
  big_uint64_buf_t crc{};
  big_uint32_buf_t length{};
  big_uint32_buf_t sqc{};
  big_uint32_buf_t data_id{};
};

struct StampSyncStatusPacket
{
  big_uint32_buf_t timestamp_nanoseconds{};
  big_uint32_buf_t timestamp_seconds{};
  uint8_t timestamp_sync_status{};
};

struct DetectionPacket
{
  big_float32_buf_t azimuth_angle{};
  big_float32_buf_t azimuth_angle_std{};
  uint8_t invalid_flags{};
  big_float32_buf_t elevation_angle{};
  big_float32_buf_t elevation_angle_std{};
  big_float32_buf_t range{};
  big_float32_buf_t range_std{};
  big_float32_buf_t range_rate{};
  big_float32_buf_t range_rate_std{};
  int8_t rcs{};
  big_uint16_buf_t measurement_id{};
  uint8_t raw_positive_predictive_value{};
  uint8_t classification{};
  uint8_t raw_multi_target_probability{};
  big_uint16_buf_t object_id{};
  uint8_t raw_ambiguity_flag{};
  big_uint16_buf_t sort_index{};
};

struct DetectionListPacket
{
  HeaderPacket header{};
  HeaderSomeIPPacket header_some_ip{};
  HeaderE2EP07Packet header_e2ep07{};
  StampSyncStatusPacket stamp{};
  big_uint32_buf_t event_data_qualifier{};
  uint8_t extended_qualifier{};
  big_uint16_buf_t origin_invalid_flags{};
  big_float32_buf_t origin_x_pos{};
  big_float32_buf_t origin_x_std{};
  big_float32_buf_t origin_y_pos{};
  big_float32_buf_t origin_y_std{};
  big_float32_buf_t origin_z_pos{};
  big_float32_buf_t origin_z_std{};
  big_float32_buf_t origin_roll{};
  big_float32_buf_t origin_roll_std{};
  big_float32_buf_t origin_pitch{};
  big_float32_buf_t origin_pitch_std{};
  big_float32_buf_t origin_yaw{};
  big_float32_buf_t origin_yaw_std{};
  uint8_t list_invalid_flags{};
  DetectionPacket detections[max_detections];
  big_float32_buf_t list_rad_vel_domain_min{};
  big_float32_buf_t list_rad_vel_domain_max{};
  big_uint32_buf_t number_of_detections{};
  big_float32_buf_t alignment_azimuth_correction{};
  big_float32_buf_t alignment_elevation_correction{};
  uint8_t alignment_status{};
  uint8_t reserved[14];
};

struct ObjectPacket
{
  big_uint16_buf_t status_sensor{};
  big_uint32_buf_t id{};
  big_uint16_buf_t age{};
  uint8_t status_measurement{};
  uint8_t status_movement{};
  big_uint16_buf_t position_invalid_flags{};
  uint8_t position_reference{};
  big_float32_buf_t position_x{};
  big_float32_buf_t position_x_std{};
  big_float32_buf_t position_y{};
  big_float32_buf_t position_y_std{};
  big_float32_buf_t position_z{};
  big_float32_buf_t position_z_std{};
  big_float32_buf_t position_covariance_xy{};
  big_float32_buf_t position_orientation{};
  big_float32_buf_t position_orientation_std{};
  uint8_t existence_invalid_flags{};
  big_float32_buf_t raw_existence_probability{};
  big_float32_buf_t existence_ppv{};
  uint8_t raw_classification_car{};
  uint8_t raw_classification_truck{};
  uint8_t raw_classification_motorcycle{};
  uint8_t raw_classification_bicycle{};
  uint8_t raw_classification_pedestrian{};
  uint8_t raw_classification_animal{};
  uint8_t raw_classification_hazard{};
  uint8_t raw_classification_unknown{};
  uint8_t classification_overdrivable{};
  uint8_t classification_underdrivable{};
  uint8_t dynamics_abs_vel_invalid_flags{};
  big_float32_buf_t dynamics_abs_vel_x{};
  big_float32_buf_t dynamics_abs_vel_x_std{};
  big_float32_buf_t dynamics_abs_vel_y{};
  big_float32_buf_t dynamics_abs_vel_y_std{};
  big_float32_buf_t dynamics_abs_vel_covariance_xy{};
  uint8_t dynamics_rel_vel_invalid_flags{};
  big_float32_buf_t dynamics_rel_vel_x{};
  big_float32_buf_t dynamics_rel_vel_x_std{};
  big_float32_buf_t dynamics_rel_vel_y{};
  big_float32_buf_t dynamics_rel_vel_y_std{};
  big_float32_buf_t dynamics_rel_vel_covariance_xy{};
  uint8_t dynamics_abs_accel_invalid_flags{};
  big_float32_buf_t dynamics_abs_accel_x{};
  big_float32_buf_t dynamics_abs_accel_x_std{};
  big_float32_buf_t dynamics_abs_accel_y{};
  big_float32_buf_t dynamics_abs_accel_y_std{};
  big_float32_buf_t dynamics_abs_accel_covariance_xy{};
  uint8_t dynamics_rel_accel_invalid_flags{};
  big_float32_buf_t dynamics_rel_accel_x{};
  big_float32_buf_t dynamics_rel_accel_x_std{};
  big_float32_buf_t dynamics_rel_accel_y{};
  big_float32_buf_t dynamics_rel_accel_y_std{};
  big_float32_buf_t dynamics_rel_accel_covariance_xy{};
  uint8_t dynamics_orientation_invalid_flags{};
  big_float32_buf_t dynamics_orientation_rate_mean{};
  big_float32_buf_t dynamics_orientation_rate_std{};
  big_uint32_buf_t shape_length_status{};
  uint8_t shape_length_edge_invalid_flags{};
  big_float32_buf_t shape_length_edge_mean{};
  big_float32_buf_t shape_length_edge_std{};
  big_uint32_buf_t shape_width_status{};
  uint8_t shape_width_edge_invalid_flags{};
  big_float32_buf_t shape_width_edge_mean{};
  big_float32_buf_t shape_width_edge_std{};
};

struct ObjectListPacket
{
  HeaderPacket header{};
  HeaderSomeIPPacket header_some_ip{};
  HeaderE2EP07Packet header_e2ep07{};
  StampSyncStatusPacket stamp{};
  big_uint32_buf_t event_data_qualifier{};
  uint8_t extended_qualifier{};
  uint8_t number_of_objects{};
  ObjectPacket objects[max_objects];
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
  HeaderPacket header{};
  StampSyncStatusPacket stamp{};
  uint8_t sw_version_major{};
  uint8_t sw_version_minor{};
  uint8_t sw_version_patch{};
  StatusConfigurationPacket status{};
  uint8_t configuration_counter{};
  uint8_t longitudinal_velocity_status{};
  uint8_t longitudinal_acceleration_status{};
  uint8_t lateral_acceleration_status{};
  uint8_t yaw_rate_status{};
  uint8_t steering_angle_status{};
  uint8_t driving_direction_status{};
  uint8_t characteristic_speed_status{};
  uint8_t radar_status{};
  uint8_t voltage_status{};
  uint8_t temperature_status{};
  uint8_t blockage_status{};
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
  HeaderPacket header{};
  uint8_t reserved0[6];
  big_float32_buf_t acceleration_lateral{};
  uint8_t reserved1[22];
};

struct AccelerationLongitudinalCoGPacket
{
  HeaderPacket header{};
  uint8_t reserved0[6];
  big_float32_buf_t acceleration_lateral{};
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
  HeaderPacket header{};
  uint8_t reserved0{};
  uint8_t driving_direction{};
  uint8_t reserved1[20];
};

struct SteeringAngleFrontAxlePacket
{
  HeaderPacket header{};
  uint8_t reserved0[6];
  big_float32_buf_t steering_angle_front_axle{};
  uint8_t reserved1[22];
};

struct VelocityVehiclePacket
{
  HeaderPacket header{};
  uint8_t reserved0[3];
  big_float32_buf_t velocity_vehicle{};
  uint8_t reserved1[21];
};

struct YawRatePacket
{
  HeaderPacket header{};
  uint8_t reserved0[6];
  big_float32_buf_t yaw_rate{};
  uint8_t reserved1[22];
};

struct FilterStatusEntryPacket
{
  uint8_t active{};
  uint8_t data_index{};
  big_float32_buf_t min_value{};
  big_float32_buf_t max_value{};
};

struct FilterStatusPacket
{
  HeaderPacket header{};
  StampSyncStatusPacket stamp{};
  uint8_t filter_configuration_counter{};
  uint8_t detection_sort_index{};
  uint8_t object_sort_index{};
  FilterStatusEntryPacket detection_filters[detection_filter_properties_num];
  FilterStatusEntryPacket object_filters[object_filter_properties_num];
};

#pragma pack(pop)

template <typename T>
inline float normalize_probability(T & raw_prob)
{
  return static_cast<float>(raw_prob) / raw_prob_norm;
};

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
