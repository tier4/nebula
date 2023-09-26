#ifndef NEBULA_ROBOSENSE_COMMON_H
#define NEBULA_ROBOSENSE_COMMON_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#ifdef _WIN32
#include <ws2tcpip.h>
#else  //__linux__
#include <arpa/inet.h>
#endif
#include <bitset>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
namespace nebula
{
namespace drivers

{

inline int16_t RS_SWAP_INT16(int16_t value)
{
  uint8_t * v = (uint8_t *)&value;

  uint8_t temp;
  temp = v[0];
  v[0] = v[1];
  v[1] = temp;

  return value;
}

struct DeviceInfo
{
  void operator=(DeviceInfo & deviceInfo)
  {
    memcpy(this->sn, deviceInfo.sn, 6);
    memcpy(this->mac, deviceInfo.mac, 6);
    memcpy(this->top_ver, deviceInfo.top_ver, 5);
    memcpy(this->bottom_ver, deviceInfo.bottom_ver, 5);
    this->rpm = deviceInfo.rpm;
    this->bot_fpga_temperature = deviceInfo.bot_fpga_temperature;
    this->recv_A_temperature = deviceInfo.recv_A_temperature;
    this->recv_B_temperature = deviceInfo.recv_B_temperature;
    this->main_fpga_temperature = deviceInfo.main_fpga_temperature;
    this->main_fpga_core_temperature = deviceInfo.main_fpga_core_temperature;
    this->lane_up = deviceInfo.lane_up;
    this->lane_up_cnt = deviceInfo.lane_up_cnt;
    this->main_status = deviceInfo.main_status;
    this->gps_status = deviceInfo.gps_status;
  }
  uint8_t sn[6];
  uint8_t mac[6];
  uint8_t top_ver[5];
  uint8_t bottom_ver[5];
  uint16_t rpm;
  uint16_t bot_fpga_temperature;
  uint16_t recv_A_temperature;
  uint16_t recv_B_temperature;
  uint16_t main_fpga_temperature;
  uint16_t main_fpga_core_temperature;
  uint8_t lane_up;
  uint16_t lane_up_cnt;
  uint16_t main_status;
  uint8_t gps_status;
};

struct DeviceCalculateParamsInfo : DeviceInfo
{
  float bot_fpga_temperature_val;
  float recv_A_temperature_val;
  float recv_B_temperature_val;
  float main_fpga_temperature_val;
  float main_fpga_core_temperature_val;
};
//
// define M_PI
//
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES  // for VC++, required to use const M_IP in <math.h>
#endif

#include <math.h>

#define DEGREE_TO_RADIAN(deg) ((deg)*M_PI / 180)
#define RADIAN_TO_DEGREE(deg) ((deg)*180 / M_PI)
/// @brief struct for Robosense sensor configuration
struct RobosenseSensorConfiguration : SensorConfigurationBase
{
  uint16_t difop_port{};
  double scan_phase{};
  double dual_return_distance_threshold{};
  uint16_t rotation_speed;
  uint16_t cloud_min_angle;
  uint16_t cloud_max_angle;
};
/// @brief Convert RobosenseSensorConfiguration to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, RobosenseSensorConfiguration const & arg)
{
  os << (SensorConfigurationBase)(arg) << ", DifopPort: " << arg.difop_port
     << ", ScanPhase:" << arg.scan_phase << ", RotationSpeed:" << arg.rotation_speed
     << ", FOV(Start):" << arg.cloud_min_angle << ", FOV(End):" << arg.cloud_max_angle;
  return os;
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

  /// @brief Loading calibration data (not used)
  /// @param calibration_content
  /// @return Resulting status
  inline nebula::Status LoadFromString(const std::string & calibration_content)
  {
    std::stringstream ss;
    ss << calibration_content;

    std::string header;
    std::getline(ss, header);

    char sep;
    int laser_id;
    float elevation;
    float azimuth;
    while (!ss.eof()) {
      ss >> laser_id >> sep >> elevation >> sep >> azimuth;
      elev_angle_map[laser_id - 1] = elevation;
      azimuth_offset_map[laser_id - 1] = azimuth;
    }
    return Status::OK;
  }

  /// @brief Saving calibration data (not used)
  /// @param calibration_file
  /// @return Resulting status
  inline nebula::Status SaveFile(const std::string & calibration_file)
  {
    std::ofstream ofs(calibration_file);
    if (!ofs) {
      return Status::CANNOT_SAVE_FILE;
    }
    ofs << "Laser id,Elevation,Azimuth" << std::endl;
    for (const auto & pair : elev_angle_map) {
      auto laser_id = pair.first + 1;
      float elevation = pair.second;
      float azimuth = azimuth_offset_map[pair.first];
      ofs << laser_id << "," << elevation << "," << azimuth << std::endl;
    }
    ofs.close();

    return Status::OK;
  }
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

/// @brief Convert return mode name to ReturnMode enum (Robosense-specific ReturnModeFromString)
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @param sensor_model Model for correct conversion
/// @return Corresponding ReturnMode
inline ReturnMode ReturnModeFromStringRobosense(
  const std::string & return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::ROBOSENSE_HELIOS:
      if (return_mode == "Dual") return ReturnMode::DUAL;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "First") return ReturnMode::FIRST;
      break;
    default:
      if (return_mode == "Dual") return ReturnMode::DUAL;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "First") return ReturnMode::FIRST;
      break;
  }

  return ReturnMode::UNKNOWN;
}

/// @brief Convert return mode number to ReturnMode enum
/// @param return_mode Return mode number from the hardware response
/// @param sensor_model Model for correct conversion
/// @return Corresponding ReturnMode
inline ReturnMode ReturnModeFromIntRobosense(
  const int return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::ROBOSENSE_HELIOS:
      if (return_mode == 0) return ReturnMode::DUAL;
      if (return_mode == 4) return ReturnMode::STRONGEST;
      if (return_mode == 5) return ReturnMode::LAST;
      if (return_mode == 6) return ReturnMode::FIRST;
      break;
    default:
      if (return_mode == 0) return ReturnMode::DUAL;
      if (return_mode == 4) return ReturnMode::STRONGEST;
      if (return_mode == 5) return ReturnMode::LAST;
      if (return_mode == 6) return ReturnMode::FIRST;
      break;
  }

  return ReturnMode::UNKNOWN;
}

/// @brief Convert ReturnMode enum to return mode number
/// @param return_mode target ReturnMode
/// @param sensor_model Model for correct conversion
/// @return Corresponding return mode number for the hardware
inline int IntFromReturnModeRobosense(
  const ReturnMode return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::ROBOSENSE_HELIOS:
      if (return_mode == ReturnMode::DUAL) return 0;
      if (return_mode == ReturnMode::STRONGEST) return 4;
      if (return_mode == ReturnMode::LAST) return 5;
      if (return_mode == ReturnMode::FIRST) return 6;
      break;
    default:
      if (return_mode == ReturnMode::DUAL) return 0;
      if (return_mode == ReturnMode::STRONGEST) return 4;
      if (return_mode == ReturnMode::LAST) return 5;
      if (return_mode == ReturnMode::FIRST) return 6;
      break;
  }

  return -1;
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_ROBOSENSE_COMMON_H
