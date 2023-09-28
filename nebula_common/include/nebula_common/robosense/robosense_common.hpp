#pragma once

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
  if (return_mode == "Dual") return ReturnMode::DUAL;
  if (return_mode == "Strongest") return ReturnMode::STRONGEST;
  if (return_mode == "Last") return ReturnMode::LAST;
  if (return_mode == "First") return ReturnMode::FIRST;

  return ReturnMode::UNKNOWN;
}

/// @brief struct for Robosense calibration configuration
struct RobosenseCalibrationConfiguration : CalibrationConfigurationBase
{
  std::map<size_t, float> elev_angle_map;
  std::map<size_t, float> azimuth_offset_map;

  template <typename stream_t>
  void LoadFromStream(stream_t & stream)
  {
    std::string header;
    std::getline(stream, header);

    char sep;
    int laser_id;
    float elevation;
    float azimuth;
    while (!stream.eof()) {
      stream >> laser_id >> sep >> elevation >> sep >> azimuth;
      elev_angle_map[laser_id - 1] = elevation;
      azimuth_offset_map[laser_id - 1] = azimuth;
    }
  }

  inline nebula::Status LoadFromFile(const std::string & calibration_file)
  {
    std::ifstream ifs(calibration_file);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }

    LoadFromStream(ifs);

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

    LoadFromStream(ss);

    return Status::OK;
  }

  //  inline nebula::Status LoadFromDifop(const std::string & calibration_file)

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

}  // namespace drivers
}  // namespace nebula
