#ifndef NEBULA_VELODYNE_COMMON_H
#define NEBULA_VELODYNE_COMMON_H

#include <fstream>
#include <sstream>

#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"
#include "decoders/velodyne_calibration_decoder.hpp"
namespace nebula
{
namespace drivers
{
struct VelodyneSensorConfiguration : SensorConfigurationBase
{
  uint16_t gnss_port{};
  double scan_phase{};
  uint16_t rotation_speed;
  uint16_t cloud_min_angle;
  uint16_t cloud_max_angle;
};
inline std::ostream & operator<<(std::ostream & os, VelodyneSensorConfiguration const & arg)
{
  os << (SensorConfigurationBase)(arg) << ", GnssPort: " << arg.gnss_port
     << ", ScanPhase:" << arg.scan_phase << ", RotationSpeed:" << arg.rotation_speed
     << ", FOV(Start):" << arg.cloud_min_angle << ", FOV(End):" << arg.cloud_max_angle;
  return os;
}

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
