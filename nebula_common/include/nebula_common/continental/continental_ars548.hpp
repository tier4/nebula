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

#pragma once
/**
 * Continental ARS548
 */
#include "boost/endian/buffers.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>
#include <ctime>

namespace nebula
{
namespace drivers
{
namespace continental_ars548
{

using namespace boost::endian;

#pragma pack(push, 1)

struct Header
{
  big_uint16_buf_t service_id;
  big_uint16_buf_t method_id;
  big_uint32_buf_t length;
};

struct HeaderSOMEIP
{
  big_uint16_buf_t client_id;
  big_uint16_buf_t session_id;
  uint8_t protocol_version;
  uint8_t interface_version;
  uint8_t message_type;
  uint8_t return_code;
};

struct HeaderE2EP07
{
  big_uint64_buf_t crc;
  big_uint32_buf_t length;
  big_uint32_buf_t sqc;
  big_uint32_buf_t data_id;
};

struct StampSyncStatus
{
  big_uint32_buf_t timestamp_nanoseconds;
  big_uint32_buf_t timestamp_seconds;
  uint8_t timestamp_sync_status;
};

struct Detection
{  // Actual 44 . Datasheet is 44
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

struct DetectionList
{
  Header header;
  HeaderSOMEIP header_someip;
  HeaderE2EP07 header_e2ep07;
  StampSyncStatus stamp;
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
  Detection detections[800];
  big_float32_buf_t list_rad_vel_domain_min;
  big_float32_buf_t list_rad_vel_domain_max;
  big_uint32_buf_t number_of_detections;
  big_float32_buf_t alignment_azimuth_correction;
  big_float32_buf_t alignment_elevation_correction;
  uint8_t alignment_status;
  uint8_t reserverd[14];
};

struct Object
{                                  // Datasheet 187 Current 184. Datsheet: 32x40 + 16x3 + 8x21
  big_uint16_buf_t status_sensor;  // Current 32x40  16x3 21x1
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
  uint8_t existance_invalid_flags;
  big_float32_buf_t existance_probability;
  big_float32_buf_t existance_ppv;
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

struct ObjectList
{
  Header header;
  HeaderSOMEIP header_someip;
  HeaderE2EP07 header_e2ep07;
  StampSyncStatus stamp;
  big_uint32_buf_t event_data_qualifier;
  uint8_t extended_qualifier;
  uint8_t number_of_objects;
  Object objects[50];
};

struct StatusConfiguration
{
  big_float32_buf_t longitudinal;
  big_float32_buf_t lateral;
  big_float32_buf_t vertical;
  big_float32_buf_t yaw;
  big_float32_buf_t pitch;
  uint8_t plug_orientation;
  big_float32_buf_t length;
  big_float32_buf_t width;
  big_float32_buf_t height;
  big_float32_buf_t wheelbase;
  big_uint16_buf_t maximum_distance;
  uint8_t frequency_slot;
  uint8_t cycle_time;
  uint8_t time_slot;
  uint8_t hcc;
  uint8_t powersave_standstill;
  uint8_t sensor_ip_address00;
  uint8_t sensor_ip_address01;
  uint8_t sensor_ip_address02;
  uint8_t sensor_ip_address03;
  uint8_t sensor_ip_address10;
  uint8_t sensor_ip_address11;
  uint8_t sensor_ip_address12;
  uint8_t sensor_ip_address13;
};

struct SensorStatus
{  // Actual 44 + 2 + 30 = 76 bytes. datasheet: 76
  Header header;
  StampSyncStatus stamp;
  uint8_t sw_version_major;
  uint8_t sw_version_minor;
  uint8_t sw_version_patch;
  StatusConfiguration status;
  uint8_t configuration_counter;
  uint8_t status_longitudinal_velocity;
  uint8_t status_longitudinal_acceleration;
  uint8_t status_lateral_acceleration;
  uint8_t status_yaw_rate;
  uint8_t status_steering_angle;
  uint8_t status_driving_direction;
  uint8_t status_characteristic_speed;
  uint8_t status_radar_status;
  uint8_t status_voltage_status;
  uint8_t status_temperature_status;
  uint8_t status_blockage_status;
};

struct Configuration
{
  Header header;
  StatusConfiguration configuration;
  uint8_t new_sensor_mounting;
  uint8_t new_vehicle_parameters;
  uint8_t new_radar_parameters;
  uint8_t new_network_configuration;
};

struct AccelerationLateralCoG
{
  Header header;
  uint8_t reserved0[6];
  big_float32_buf_t acceleration_lateral;
  uint8_t reserved1[22];
};

struct AccelerationLongitudinalCoG
{
  Header header;
  uint8_t reserved0[6];
  big_float32_buf_t acceleration_lateral;
  uint8_t reserved1[22];
};

struct CharasteristicSpeed
{
  Header header;
  uint8_t reserved0[2];
  big_float32_buf_t characteristic_speed;
  uint8_t reserved1[8];
};

struct DrivingDirection
{
  Header header;
  uint8_t reserved0;
  big_float32_buf_t driving_direction;
  uint8_t reserved1[20];
};

struct SteeringAngleFrontAxle
{
  Header header;
  uint8_t reserved0[6];
  big_float32_buf_t steering_angle_front_axle;
  uint8_t reserved1[22];
};

struct VelocityVehicle
{
  Header header;
  uint8_t reserved0[3];
  big_float32_buf_t velocity_vehicle;
  uint8_t reserved1[21];
};

struct YawRate
{
  Header header;
  uint8_t reserved0[6];
  big_float32_buf_t yaw_rate;
  uint8_t reserved1[221];
};

#pragma pack(pop)

constexpr int SERVICE_ID_BYTE = 0;
constexpr int METHOD_ID_BYTE = 2;
constexpr int LENGTH_BYTE = 4;

constexpr int CONFIGURATION_SERVICE_ID = 0;
constexpr int CONFIGURATION_METHOD_ID = 390;
constexpr int CONFIGURATION_PAYLOAD_LENGTH = 56;

constexpr int STATUS_TIMESTAMP_NANOSECONDS_BYTE = 8;
constexpr int STATUS_TIMESTAMP_SECONDS_BYTE = 12;
constexpr int STATUS_SYNC_STATUS_BYTE = 16;
constexpr int STATUS_SW_VERSION_MAJOR_BYTE = 17;
constexpr int STATUS_SW_VERSION_MINOR_BYTE = 18;
constexpr int STATUS_SW_VERSION_PATCH_BYTE = 19;

constexpr int STATUS_LONGITUDINAL_BYTE = 20;
constexpr int STATUS_LATERAL_BYTE = 24;
constexpr int STATUS_VERTICAL_BYTE = 28;
constexpr int STATUS_YAW_BYTE = 32;
constexpr int STATUS_PITCH_BYTE = 36;

constexpr int STATUS_PLUG_ORIENTATION_BYTE = 40;
constexpr int STATUS_LENGTH_BYTE = 41;
constexpr int STATUS_WIDTH_BYTE = 45;
constexpr int STATUS_HEIGHT_BYTE = 49;
constexpr int STATUS_WHEEL_BASE_BYTE = 53;
constexpr int STATUS_MAXIMUM_DISTANCE_BYTE = 57;
constexpr int STATUS_FREQUENCY_SLOT_BYTE = 59;
constexpr int STATUS_CYCLE_TIME_BYTE = 60;
constexpr int STATUS_TIME_SLOT_BYTE = 61;
constexpr int STATUS_HCC_BYTE = 62;
constexpr int STATUS_POWER_SAVING_STANDSTILL_BYTE = 63;
constexpr int STATUS_SENSOR_IP_ADDRESS0_BYTE = 64;
constexpr int STATUS_SENSOR_IP_ADDRESS1_BYTE = 68;
constexpr int STATUS_CONFIGURATION_COUNTER_BYTE = 72;
constexpr int STATUS_LONGITUDINAL_VELOCITY_BYTE = 73;
constexpr int STATUS_LONGITUDINAL_ACCELERATION_BYTE = 74;
constexpr int STATUS_LATERAL_ACCELERATION_BYTE = 75;
constexpr int STATUS_YAW_RATE_BYTE = 76;
constexpr int STATUS_STEERING_ANGLE_BYTE = 77;
constexpr int STATUS_DRIVING_DIRECTION_BYTE = 78;
constexpr int STATUS_CHARACTERISTIC_SPEED_BYTE = 79;
constexpr int STATUS_RADAR_STATUS_BYTE = 80;
constexpr int STATUS_VOLTAGE_STATUS_BYTE = 81;
constexpr int STATUS_TEMPERATURE_STATUS_BYTE = 82;
constexpr int STATUS_BLOCKAGE_STATUS_BYTE = 83;

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

constexpr int DETECTION_LIST_CRC_BYTE = 16;
constexpr int DETECTION_LIST_LENGTH_BYTE = 24;
constexpr int DETECTION_LIST_SQC_BYTE = 28;
constexpr int DETECTION_LIST_DATA_ID_BYTE = 32;
constexpr int DETECTION_LIST_TIMESTAMP_NANOSECONDS_BYTE = 36;
constexpr int DETECTION_LIST_TIMESTAMP_SECONDS_BYTE = 40;
constexpr int DETECTION_LIST_TIMESTAMP_SYNC_STATUS_BYTE = 44;
constexpr int DETECTION_LIST_ORIGIN_X_POS_BYTE = 52;
constexpr int DETECTION_LIST_ORIGIN_Y_POS_BYTE = 60;
constexpr int DETECTION_LIST_ORIGIN_Z_POS_BYTE = 68;
constexpr int DETECTION_LIST_PITCH_BYTE = 84;
constexpr int DETECTION_LIST_PITCH_STD_BYTE = 88;
constexpr int DETECTION_LIST_YAW_BYTE = 92;
constexpr int DETECTION_LIST_YAW_STD_BYTE = 96;
constexpr int DETECTION_LIST_ARRAY_BYTE = 101;
constexpr int DETECTION_LIST_RAD_VEL_DOMAIN_MIN_BYTE = 35301;
constexpr int DETECTION_LIST_RAD_VEL_DOMAIN_MAX_BYTE = 35305;
constexpr int DETECTION_LIST_NUMBER_OF_DETECTIONS_BYTE = 35309;
constexpr int DETECTION_LIST_AZIMUTH_CORRECTION_BYTE = 35313;
constexpr int DETECTION_LIST_ELEVATION_CORRECTION_BYTE = 35317;
constexpr int DETECTION_LIST_ALIGNMENT_STATUS_BYTE = 35321;

constexpr int DETECTION_STRUCT_SIZE = 44;
constexpr int DETECTION_AZIMUTH_ANGLE_BYTE = 0;
constexpr int DETECTION_AZIMUTH_ANGLE_STD_BYTE = 4;
constexpr int DETECTION_INVALID_FLAGS_BYTE = 8;
constexpr int DETECTION_ELEVATION_ANGLE_BYTE = 9;
constexpr int DETECTION_ELEVATION_ANGLE_STD_BYTE = 13;
constexpr int DETECTION_RANGE_BYTE = 17;
constexpr int DETECTION_RANGE_STD_BYTE = 21;
constexpr int DETECTION_RANGE_RATE_BYTE = 25;
constexpr int DETECTION_RANGE_RATE_STD_BYTE = 29;
constexpr int DETECTION_RCS_BYTE = 33;
constexpr int DETECTION_MEASUREMENT_ID_BYTE = 34;
constexpr int DETECTION_POSITIVE_PREDICTIVE_VALUE_BYTE = 36;
constexpr int DETECTION_CLASSIFICATION_BYTE = 37;
constexpr int DETECTION_MULT_TARGET_PROBABILITY_BYTE = 38;
constexpr int DETECTION_OBJECT_ID_BYTE = 39;
constexpr int DETECTION_AMBIGUITY_FLAG_BYTE = 41;

constexpr int OBJECT_LIST_CRC_BYTE = 16;
constexpr int OBJECT_LIST_LENGTH_BYTE = 24;
constexpr int OBJECT_LIST_SQC_BYTE = 28;
constexpr int OBJECT_LIST_DATA_ID_BYTE = 32;
constexpr int OBJECT_LIST_TIMESTAMP_NANOSECONDS_BYTE = 36;
constexpr int OBJECT_LIST_TIMESTAMP_SECONDS_BYTE = 40;
constexpr int OBJECT_LIST_TIMESTAMP_SYNC_STATUS_BYTE = 44;
constexpr int OBJECT_LIST_NUMBER_OF_OBJECTS_BYTE = 50;
constexpr int OBJECT_LIST_ARRAY_BYTE = 51;

constexpr int OBJECT_STRUCT_SIZE = 187;
constexpr int OBJECT_STATUS_SENSOR_BYTE = 0;
constexpr int OBJECT_ID_BYTE = 2;
constexpr int OBJECT_AGE_BYTE = 6;
constexpr int OBJECT_STATUS_MEASUREMENT_BYTE = 8;
constexpr int OBJECT_STATUS_MOVEMENT_BYTE = 9;
constexpr int OBJECT_POSITION_REFERENCE_BYTE = 12;
constexpr int OBJECT_POSITION_X_BYTE = 13;
constexpr int OBJECT_POSITION_X_STD_BYTE = 17;
constexpr int OBJECT_POSITION_Y_BYTE = 21;
constexpr int OBJECT_POSITION_Y_STD_BYTE = 25;
constexpr int OBJECT_POSITION_Z_BYTE = 29;
constexpr int OBJECT_POSITION_Z_STD_BYTE = 33;

constexpr int OBJECT_POSITION_COVARIANCE_XY_BYTE = 37;
constexpr int OBJECT_ORIENTATION_BYTE = 41;
constexpr int OBJECT_ORIENTATION_STD_BYTE = 45;

constexpr int OBJECT_EXISTENCE_PROBABILITY_BYTE = 50;

constexpr int OBJECT_CLASSIFICATION_CAR_BYTE = 58;
constexpr int OBJECT_CLASSIFICATION_TRUCK_BYTE = 59;
constexpr int OBJECT_CLASSIFICATION_MOTORCYCLE_BYTE = 60;
constexpr int OBJECT_CLASSIFICATION_BICYCLE_BYTE = 61;
constexpr int OBJECT_CLASSIFICATION_PEDESTRIAN_BYTE = 62;
constexpr int OBJECT_CLASSIFICATION_ANIMAL_BYTE = 63;
constexpr int OBJECT_CLASSIFICATION_HAZARD_BYTE = 64;
constexpr int OBJECT_CLASSIFICATION_UNKNOWN_BYTE = 65;

constexpr int OBJECT_DYNAMICS_ABS_VEL_X_BYTE = 69;
constexpr int OBJECT_DYNAMICS_ABS_VEL_X_STD_BYTE = 73;
constexpr int OBJECT_DYNAMICS_ABS_VEL_Y_BYTE = 77;
constexpr int OBJECT_DYNAMICS_ABS_VEL_Y_STD_BYTE = 81;
constexpr int OBJECT_DYNAMICS_ABS_VEL_COVARIANCE_XY_BYTE = 85;

constexpr int OBJECT_DYNAMICS_REL_VEL_X_BYTE = 90;
constexpr int OBJECT_DYNAMICS_REL_VEL_X_STD_BYTE = 94;
constexpr int OBJECT_DYNAMICS_REL_VEL_Y_BYTE = 98;
constexpr int OBJECT_DYNAMICS_REL_VEL_Y_STD_BYTE = 102;
constexpr int OBJECT_DYNAMICS_REL_VEL_COVARIANCE_XY_BYTE = 106;

constexpr int OBJECT_DYNAMICS_ABS_ACCEL_X_BYTE = 111;
constexpr int OBJECT_DYNAMICS_ABS_ACCEL_X_STD_BYTE = 115;
constexpr int OBJECT_DYNAMICS_ABS_ACCEL_Y_BYTE = 119;
constexpr int OBJECT_DYNAMICS_ABS_ACCEL_Y_STD_BYTE = 123;
constexpr int OBJECT_DYNAMICS_ABS_ACCEL_COVARIANCE_XY_BYTE = 127;

constexpr int OBJECT_DYNAMICS_REL_ACCEL_X_BYTE = 132;
constexpr int OBJECT_DYNAMICS_REL_ACCEL_X_STD_BYTE = 136;
constexpr int OBJECT_DYNAMICS_REL_ACCEL_Y_BYTE = 140;
constexpr int OBJECT_DYNAMICS_REL_ACCEL_Y_STD_BYTE = 144;
constexpr int OBJECT_DYNAMICS_REL_ACCEL_COVARIANCE_XY_BYTE = 148;

constexpr int OBJECT_DYNAMICS_ORIENTATION_RATE_BYTE = 153;
constexpr int OBJECT_DYNAMICS_ORIENTATION_RATE_STD_BYTE = 157;

constexpr int OBJECT_SHAPE_LENGTH_EDGE_MEAN_BYTE = 166;
constexpr int OBJECT_SHAPE_WIDTH_EDGE_MEAN_BYTE = 179;

static constexpr int DETECTION_FILTER_PROPERTIES_NUM = 7;
static constexpr int OBJECT_FILTER_PROPERTIES_NUM = 24;

}  // namespace continental_ars548
}  // namespace drivers
}  // namespace nebula
