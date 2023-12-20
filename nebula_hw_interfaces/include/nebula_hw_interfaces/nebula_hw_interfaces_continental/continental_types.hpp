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

#ifndef CONTINENTAL_TYPES_HPP
#define CONTINENTAL_TYPES_HPP

#include <boost/algorithm/string/join.hpp>
#include <boost/format.hpp>

#include <ostream>
#include <string>

namespace nebula
{

/// @brief semantic struct of ARS548 Status
struct ContinentalARS548Status
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

  ContinentalARS548Status() {}

  /// @brief Stream ContinentalRadarStatus method
  /// @param os
  /// @param arg
  /// @return stream
  friend std::ostream & operator<<(std::ostream & os, nebula::ContinentalARS548Status const & arg)
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
    os << "blockage_status_rate: " << arg.blockage_status;

    return os;
  }
};

}  // namespace nebula
#endif  // CONTINENTAL_TYPES_HPP
