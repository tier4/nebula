#ifndef NEBULA_VELODYNE_COMMON_H
#define NEBULA_VELODYNE_COMMON_H

#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"
#include "decoders/velodyne_calibration_decoder.hpp"

#include <fstream>
#include <sstream>
namespace nebula
{
namespace drivers
{
struct VelodyneSensorConfiguration : SensorConfigurationBase
{
  uint16_t gnss_port{};
  double scan_phase{};
  double cloud_min_angle;
  double cloud_max_angle;
};
inline std::ostream & operator<<(std::ostream & os, VelodyneSensorConfiguration const & arg)
{
  os << (SensorConfigurationBase)(arg) << ", GnssPort: " << arg.gnss_port
     << ", ScanPhase:" << arg.scan_phase;
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
    }
    else {
      return Status::OK;
    }
  }
  inline nebula::Status SaveFile(const std::string & calibration_file)
  {
    velodyne_calibration.write(calibration_file);
  }
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_VELODYNE_COMMON_H
