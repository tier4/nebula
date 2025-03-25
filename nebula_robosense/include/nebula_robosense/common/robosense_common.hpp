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

#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{

// Flag for detecting Bpearl version
constexpr uint8_t bpearl_v4_flag = 0x04;

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
  os << "Robosense Sensor Configuration:" << '\n';
  os << (LidarConfigurationBase)(arg) << '\n';
  os << "GNSS Port: " << arg.gnss_port << '\n';
  os << "Scan Phase: " << arg.scan_phase;
  return os;
}

/// @brief Convert return mode name to ReturnMode enum (Robosense-specific return_mode_from_string)
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @return Corresponding ReturnMode
inline ReturnMode return_mode_from_string_robosense(const std::string & return_mode)
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

  void set_channel_size(const size_t channel_num) { calibration.resize(channel_num); }

  template <typename stream_t>
  inline nebula::Status load_from_stream(stream_t & stream)
  {
    std::string header;
    std::getline(stream, header);

    char sep;
    size_t laser_id;
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

  inline nebula::Status load_from_file(const std::string & calibration_file)
  {
    std::ifstream ifs(calibration_file);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }

    const auto status = load_from_stream(ifs);
    ifs.close();
    return status;
  }

  /// @brief Loading calibration data (not used)
  /// @param calibration_content
  /// @return Resulting status
  inline nebula::Status load_from_string(const std::string & calibration_content)
  {
    std::stringstream ss;
    ss << calibration_content;

    const auto status = load_from_stream(ss);
    return status;
  }

  //  inline nebula::Status load_from_difop(const std::string & calibration_file)

  /// @brief Saving calibration data (not used)
  /// @param calibration_file
  /// @return Resulting status
  inline nebula::Status save_file(const std::string & calibration_file)
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

  [[nodiscard]] inline ChannelCorrection get_correction(const size_t channel_id) const
  {
    return calibration[channel_id];
  }

  void create_corrected_channels()
  {
    for (auto & correction : calibration) {
      uint16_t channel = 0;
      for (const auto & compare : calibration) {
        if (compare.elevation < correction.elevation) ++channel;
      }
      correction.channel = channel;
    }
  }
};

}  // namespace drivers
}  // namespace nebula
