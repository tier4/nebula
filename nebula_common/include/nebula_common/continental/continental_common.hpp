
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

#ifndef NEBULA_CONTINENTAL_COMMON_H
#define NEBULA_CONTINENTAL_COMMON_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <bitset>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
namespace nebula
{
namespace drivers
{
/// @brief struct for ARS548 sensor configuration
struct ContinentalARS548SensorConfiguration : EthernetSensorConfigurationBase
{
  std::string multicast_ip{};
  std::string new_sensor_ip{};
  std::string base_frame{};
  uint16_t configuration_host_port{};
  uint16_t configuration_sensor_port{};
  bool use_sensor_time{};
  uint16_t new_plug_orientation{};
  float new_vehicle_length{};
  float new_vehicle_width{};
  float new_vehicle_height{};
  float new_vehicle_wheelbase{};
  uint16_t new_radar_maximum_distance{};
  uint16_t new_radar_frequency_slot{};
  uint16_t new_radar_cycle_time{};
  uint16_t new_radar_time_slot{};
  uint16_t new_radar_country_code{};
  uint16_t new_radar_powersave_standstill{};
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
     << ", NewSensorIP: " << arg.new_sensor_ip << ", BaseFrame: " << arg.base_frame
     << ", ConfigurationHostPort: " << arg.configuration_host_port
     << ", ConfigurationSensorPort: " << arg.configuration_sensor_port
     << ", UseSensorTime: " << arg.use_sensor_time
     << ", NewPlugOrientation: " << arg.new_plug_orientation
     << ", NewVehicleLength: " << arg.new_vehicle_length
     << ", NewVehicleWidth: " << arg.new_vehicle_width
     << ", NewVehicleHeight: " << arg.new_vehicle_height
     << ", NewVehicleWheelbase: " << arg.new_vehicle_wheelbase
     << ", NewRadarMaximumDistance: " << arg.new_radar_maximum_distance
     << ", NewRadarFrequencySlot: " << arg.new_radar_frequency_slot
     << ", NewRadarCycleTime: " << arg.new_radar_cycle_time
     << ", NewRadarTimeSlot: " << arg.new_radar_time_slot
     << ", NewRadarCountryCode: " << arg.new_radar_country_code
     << ", RadarPowersaveStandstill: " << arg.new_radar_powersave_standstill;
  return os;
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_CONTINENTAL_COMMON_H
