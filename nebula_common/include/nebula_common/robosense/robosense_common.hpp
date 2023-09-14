#ifndef NEBULA_ROBOSENSE_COMMON_H
#define NEBULA_ROBOSENSE_COMMON_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <bitset>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

namespace nebula
{
namespace drivers
{
/// @brief struct for Robosense sensor configuration
struct RobosenseSensorConfiguration : SensorConfigurationBase
{
  uint16_t gnss_port{};  // difop
  double scan_phase{};   // start/end angle
  uint16_t rotation_speed;
  uint16_t cloud_min_angle;
  uint16_t cloud_max_angle;
  // time_sync_source
  // operation_mode
  // noise_filter
};

/// @brief Convert RobosenseSensorConfiguration to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, RobosenseSensorConfiguration const & arg)
{
  os << (SensorConfigurationBase)(arg) << ", GnssPort: " << arg.gnss_port
     << ", ScanPhase:" << arg.scan_phase << ", RotationSpeed:" << arg.rotation_speed
     << ", FOV(Start):" << arg.cloud_min_angle << ", FOV(End):" << arg.cloud_max_angle;
  return os;
}

/// @brief Convert return mode name to ReturnMode enum (Robosense-specific ReturnModeFromString)
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @return Corresponding ReturnMode
inline ReturnMode ReturnModeFromStringRobosense(const std::string & return_mode)
{
  if (return_mode == "Last") return ReturnMode::LAST;
  if (return_mode == "Strongest") return ReturnMode::STRONGEST;
  if (return_mode == "Dual") return ReturnMode::DUAL;

  return ReturnMode::UNKNOWN;
}

/// @brief struct for Robosense calibration configuration
struct RobosenseCalibrationConfiguration : CalibrationConfigurationBase
{
  std::map<size_t, float> elev_angle_map;
  std::map<size_t, float> azimuth_offset_map;

  inline nebula::Status LoadFromFile(const std::string & calibration_file)
  {
    std::ifstream ifs(calibration_file);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }

    std::string header;
    std::getline(ifs, header);

    char sep;
    int laser_id;
    float elevation;
    float azimuth;
    while (!ifs.eof()) {
      ifs >> laser_id >> sep >> elevation >> sep >> azimuth;
      elev_angle_map[laser_id - 1] = elevation;
      azimuth_offset_map[laser_id - 1] = azimuth;
    }
    ifs.close();
    return Status::OK;
  }
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_ROBOSENSE_COMMON_H
