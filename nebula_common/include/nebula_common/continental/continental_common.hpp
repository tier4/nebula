
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
/// @brief struct for Hesai sensor configuration
struct ContinentalRadarEthernetSensorConfiguration : EthernetSensorConfigurationBase
{
  std::string multicast_ip{};
  uint16_t configuration_host_port{};
  uint16_t configuration_sensor_port{};
};

/// @brief Convert ContinentalRadarEthernetSensorConfiguration to string (Overloading the <<
/// operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(
  std::ostream & os, ContinentalRadarEthernetSensorConfiguration const & arg)
{
  os << (EthernetSensorConfigurationBase)(arg) << ", MulticastIP: " << arg.multicast_ip
     << ", ConfigurationHostPort: " << arg.configuration_host_port
     << ", ConfigurationSensorPort: " << arg.configuration_sensor_port;
  return os;
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_CONTINENTAL_COMMON_H
