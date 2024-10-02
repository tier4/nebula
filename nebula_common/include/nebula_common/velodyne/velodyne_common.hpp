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

#ifndef NEBULA_VELODYNE_COMMON_H
#define NEBULA_VELODYNE_COMMON_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/velodyne/velodyne_calibration_decoder.hpp"

#include <string>
namespace nebula
{
namespace drivers
{
/// @brief struct for Velodyne sensor configuration
struct VelodyneSensorConfiguration : LidarConfigurationBase
{
  uint16_t gnss_port{};
  double scan_phase{};
  uint16_t rotation_speed;
  uint16_t cloud_min_angle;
  uint16_t cloud_max_angle;
};
/// @brief Convert VelodyneSensorConfiguration to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, VelodyneSensorConfiguration const & arg)
{
  os << "Velodyne Sensor Configuration:" << '\n';
  os << (LidarConfigurationBase)(arg) << '\n';
  os << "GNSS Port: " << arg.gnss_port << '\n';
  os << "Scan Phase: " << arg.scan_phase << '\n';
  os << "Rotation Speed: " << arg.rotation_speed << '\n';
  os << "FoV Start: " << arg.cloud_min_angle << '\n';
  os << "FoV End: " << arg.cloud_max_angle;
  return os;
}

/// @brief struct for Velodyne calibration configuration
struct VelodyneCalibrationConfiguration : CalibrationConfigurationBase
{
  VelodyneCalibration velodyne_calibration;
  inline nebula::Status load_from_file(const std::string & calibration_file)
  {
    velodyne_calibration.read(calibration_file);
    if (!velodyne_calibration.initialized) {
      return Status::INVALID_CALIBRATION_FILE;
    } else {
      return Status::OK;
    }
  }
  inline nebula::Status save_file(const std::string & calibration_file)
  {
    velodyne_calibration.write(calibration_file);
    return Status::OK;
  }
};

/// @brief Convert return mode name to ReturnMode enum (Velodyne-specific return_mode_from_string)
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @return Corresponding ReturnMode
inline ReturnMode return_mode_from_string_velodyne(const std::string & return_mode)
{
  if (return_mode == "Strongest") return ReturnMode::SINGLE_STRONGEST;
  if (return_mode == "Last") return ReturnMode::SINGLE_LAST;
  if (return_mode == "Dual") return ReturnMode::DUAL_ONLY;

  return ReturnMode::UNKNOWN;
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_VELODYNE_COMMON_H
