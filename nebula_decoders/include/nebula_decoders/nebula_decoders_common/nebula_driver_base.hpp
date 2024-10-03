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

#ifndef NEBULA_DRIVER_BASE_H
#define NEBULA_DRIVER_BASE_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>
#include <vector>

namespace nebula::drivers
{
/// @brief Base class for each sensor driver
class NebulaDriverBase
{
public:
  NebulaDriverBase(NebulaDriverBase && c) = delete;
  NebulaDriverBase & operator=(NebulaDriverBase && c) = delete;
  NebulaDriverBase(const NebulaDriverBase & c) = delete;
  NebulaDriverBase & operator=(const NebulaDriverBase & c) = delete;

  NebulaDriverBase() = default;

  /// @brief Virtual function for setting calibration configuration
  /// @param calibration_configuration CalibrationConfiguration including file path
  /// @return Resulting status
  virtual Status set_calibration_configuration(
    const CalibrationConfigurationBase & calibration_configuration) = 0;
};

}  // namespace nebula::drivers
#endif  // NEBULA_DRIVER_BASE_H
