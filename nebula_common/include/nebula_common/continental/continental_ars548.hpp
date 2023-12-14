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
constexpr int SERVICE_ID_BYTE = 0;
constexpr int METHOD_ID_BYTE = 2;
constexpr int LENGTH_BYTE = 4;

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
