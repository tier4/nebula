#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <bitset>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <vector>

namespace nebula
{
namespace drivers
{

// Flag for detecting Bpearl version
constexpr uint8_t BPEARL_V4_FLAG = 0x04;

/// @brief struct for Robosense sensor configuration
struct RobosenseSensorConfiguration : LidarConfigurationBase
{
  uint16_t gnss_port{};  // difop
  double scan_phase{};   // start/end angle
  double dual_return_distance_threshold{};
};

/// @brief Convert RobosenseSensorConfiguration to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, RobosenseSensorConfiguration const & arg)
{
  os << (LidarConfigurationBase)(arg) << ", GnssPort: " << arg.gnss_port
     << ", ScanPhase:" << arg.scan_phase;
  return os;
}

/// @brief Convert return mode name to ReturnMode enum (Robosense-specific ReturnModeFromString)
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @return Corresponding ReturnMode
inline ReturnMode ReturnModeFromStringRobosense(const std::string & return_mode)
{
  if (return_mode == "Dual") return ReturnMode::DUAL;
  if (return_mode == "Strongest") return ReturnMode::SINGLE_STRONGEST;
  if (return_mode == "Last") return ReturnMode::SINGLE_LAST;
  if (return_mode == "First") return ReturnMode::SINGLE_FIRST;

  return ReturnMode::UNKNOWN;
}

struct ChannelCorrection
{
  float azimuth{NAN};
  float elevation{NAN};
  uint16_t channel{};

  [[nodiscard]] bool has_value() const { return !std::isnan(azimuth) && !std::isnan(elevation); }
};

/// @brief struct for Robosense calibration configuration
struct RobosenseCalibrationConfiguration : CalibrationConfigurationBase
{
  std::vector<ChannelCorrection> calibration;

  void SetChannelSize(const size_t channel_num) { calibration.resize(channel_num); }

  template <typename stream_t>
  inline nebula::Status LoadFromStream(stream_t & stream)
  {
    std::string header;
    std::getline(stream, header);

    char sep;
    int laser_id;
    float elevation;
    float azimuth;
    Status load_status = Status::OK;
    for (size_t i = 0; i < calibration.size(); ++i) {
      stream >> laser_id >> sep >> elevation >> sep >> azimuth;

      if (laser_id <= 0 || laser_id > calibration.size()) {
        std::cout << "Invalid laser id: " << laser_id << std::endl;
        load_status = Status::INVALID_CALIBRATION_FILE;
      }
      if (std::isnan(elevation) || std::isnan(azimuth)) {
        std::cout << "Invalid calibration data" << laser_id << "," << elevation << "," << azimuth
                  << std::endl;
        load_status = Status::INVALID_CALIBRATION_FILE;
      }
      if (
        calibration[laser_id - 1].has_value() && calibration[laser_id - 1].elevation != elevation &&
        calibration[laser_id - 1].azimuth != azimuth) {
        std::cout << "Duplicate calibration data for laser id: " << laser_id << std::endl;
        load_status = Status::INVALID_CALIBRATION_FILE;
      }

      ChannelCorrection correction{azimuth, elevation};
      calibration[laser_id - 1] = correction;
    }

    for (auto & calib : calibration) {
      if (!calib.has_value()) {
        std::cout << calib.elevation << "," << calib.azimuth << std::endl;
        std::cout << "Missing calibration data" << std::endl;
        load_status = Status::INVALID_CALIBRATION_FILE;
      }
    }

    if (load_status != Status::OK) {
      for (auto & correction : calibration) {
        correction.elevation = NAN;
        correction.azimuth = NAN;
      }
    }

    return load_status;
  }

  inline nebula::Status LoadFromFile(const std::string & calibration_file)
  {
    std::ifstream ifs(calibration_file);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }

    const auto status = LoadFromStream(ifs);
    ifs.close();
    return status;
  }

  /// @brief Loading calibration data (not used)
  /// @param calibration_content
  /// @return Resulting status
  inline nebula::Status LoadFromString(const std::string & calibration_content)
  {
    std::stringstream ss;
    ss << calibration_content;

    const auto status = LoadFromStream(ss);
    return status;
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

    for (size_t i = 0; i < calibration.size(); ++i) {
      auto laser_id = i + 1;
      float elevation = calibration[i].elevation;
      float azimuth = calibration[i].azimuth;
      ofs << laser_id << "," << elevation << "," << azimuth << std::endl;
    }

    ofs.close();
    return Status::OK;
  }

  [[nodiscard]] inline ChannelCorrection GetCorrection(const size_t channel_id) const
  {
    return calibration[channel_id];
  }

  void CreateCorrectedChannels()
  {
    for(auto& correction : calibration) {
      uint16_t channel = 0;
      for(const auto& compare:calibration) {
        if(compare.elevation < correction.elevation) ++channel;
      }
      correction.channel = channel;
    }
  }
};

}  // namespace drivers
}  // namespace nebula
