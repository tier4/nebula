#ifndef NEBULA_VELODYNE_COMMON_H
#define NEBULA_VELODYNE_COMMON_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/velodyne/velodyne_calibration_decoder.hpp"

#include <fstream>
#include <sstream>
namespace nebula
{
namespace drivers
{
/// @brief struct for Velodyne sensor configuration
struct VelodyneSensorConfiguration : SensorConfigurationBase
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
  os << (SensorConfigurationBase)(arg) << ", GnssPort: " << arg.gnss_port
     << ", ScanPhase:" << arg.scan_phase << ", RotationSpeed:" << arg.rotation_speed
     << ", FOV(Start):" << arg.cloud_min_angle << ", FOV(End):" << arg.cloud_max_angle;
  return os;
}

/// @brief struct for Velodyne calibration configuration
struct VelodyneCalibrationConfiguration : CalibrationConfigurationBase
{
  VelodyneCalibration velodyne_calibration;
  inline nebula::Status LoadFromFile(const std::string & calibration_file)
  {
    velodyne_calibration.read(calibration_file);
    if (!velodyne_calibration.initialized) {
      return Status::INVALID_CALIBRATION_FILE;
    } else {
      return Status::OK;
    }
  }
  inline nebula::Status SaveFile(const std::string & calibration_file)
  {
    velodyne_calibration.write(calibration_file);
    return Status::OK;
  }
};

/// @brief Convert return mode name to ReturnMode enum (Velodyne-specific ReturnModeFromString)
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @return Corresponding ReturnMode
inline ReturnMode ReturnModeFromStringVelodyne(const std::string & return_mode)
{
  if (return_mode == "Strongest") return ReturnMode::SINGLE_STRONGEST;
  if (return_mode == "Last") return ReturnMode::SINGLE_LAST;
  if (return_mode == "Dual") return ReturnMode::DUAL_ONLY;

  return ReturnMode::UNKNOWN;
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_VELODYNE_COMMON_H
