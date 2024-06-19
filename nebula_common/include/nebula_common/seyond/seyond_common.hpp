#ifndef NEBULA_SEYOND_COMMON_H
#define NEBULA_SEYOND_COMMON_H

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
/// @brief struct for Seyond sensor configuration
struct SeyondSensorConfiguration : public LidarConfigurationBase
{
  uint16_t gnss_port{};
  double scan_phase{};
  double dual_return_distance_threshold{};
  uint16_t rotation_speed;
  uint16_t cloud_min_angle;
  uint16_t cloud_max_angle;
  PtpProfile ptp_profile;
  uint8_t ptp_domain;
  PtpTransportType ptp_transport_type;
  PtpSwitchType ptp_switch_type;
};
/// @brief Convert SeyondSensorConfiguration to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, SeyondSensorConfiguration const & arg)
{
  os << (LidarConfigurationBase)(arg) << ", GnssPort: " << arg.gnss_port
     << ", ScanPhase:" << arg.scan_phase << ", RotationSpeed:" << arg.rotation_speed
     << ", FOV(Start):" << arg.cloud_min_angle << ", FOV(End):" << arg.cloud_max_angle
     << ", DualReturnDistanceThreshold:" << arg.dual_return_distance_threshold
     << ", PtpProfile:" << arg.ptp_profile << ", PtpDomain:" << std::to_string(arg.ptp_domain)
     << ", PtpTransportType:" << arg.ptp_transport_type
     << ", PtpSwitchType:" << arg.ptp_switch_type;
  return os;
}

struct SeyondCalibrationConfigurationBase : public CalibrationConfigurationBase
{

};

/// @brief struct for Seyond calibration configuration
struct SeyondCalibrationConfiguration : public SeyondCalibrationConfigurationBase
{
  std::map<size_t, float> elev_angle_map;
  std::map<size_t, float> azimuth_offset_map;
};

/*
<option value="0">Last Return</option>
<option value="1">Strongest Return</option>
<option value="3">First Return</option>
<option value="2">Last Return + Strongest Return</option>
<option value="4">First Return + Last Return</option>
<option value="5">First Return + Strongest Return</option>
*/
/*
<option value="0">Last Return</option>
<option value="1">Strongest Return</option>
<option value="3">First Return</option>
<option value="2">Last Return + Strongest Return</option>
<option value="4">First Return + Strongest Return</option>
<option value="5">First Return + Last Return</option>
<option value="6">First Return + Last Return + Strongest Return</option>
*/

/// @brief Convert return mode name to ReturnMode enum (Seyond-specific ReturnModeFromString)
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @param sensor_model Model for correct conversion
/// @return Corresponding ReturnMode
inline ReturnMode ReturnModeFromStringSeyond(
  const std::string & return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::SEYOND_FALCON_KINETIC:
    case SensorModel::SEYOND_ROBIN_W:
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "Dual") return ReturnMode::DUAL;
      if (return_mode == "DualStrongestLast") return ReturnMode::DUAL_STRONGEST_LAST;
      break;
    default:
      break;
  }
  return ReturnMode::UNKNOWN;
}

/// @brief Convert return mode number to ReturnMode enum
/// @param return_mode Return mode number from the hardware response
/// @param sensor_model Model for correct conversion
/// @return Corresponding ReturnMode
inline ReturnMode ReturnModeFromIntSeyond(const int return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::SEYOND_FALCON_KINETIC:
    case SensorModel::SEYOND_ROBIN_W:
      if (return_mode == 1) return ReturnMode::STRONGEST;
      if (return_mode == 2) return ReturnMode::DUAL;
      if (return_mode == 3) return ReturnMode::DUAL_STRONGEST_LAST;
      break;
    default:
      break;
  }
  return ReturnMode::UNKNOWN;
}

/// @brief Convert ReturnMode enum to return mode number
/// @param return_mode target ReturnMode
/// @param sensor_model Model for correct conversion
/// @return Corresponding return mode number for the hardware
inline int IntFromReturnModeSeyond(const ReturnMode return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::SEYOND_FALCON_KINETIC:
    case SensorModel::SEYOND_ROBIN_W:
      if (return_mode == ReturnMode::STRONGEST) return 1;
      if (return_mode == ReturnMode::DUAL) return 2;
      if (return_mode == ReturnMode::DUAL_STRONGEST_LAST) return 3;
      break;
    default:
      break;
  }
  return -1;
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_SEYOND_COMMON_H
