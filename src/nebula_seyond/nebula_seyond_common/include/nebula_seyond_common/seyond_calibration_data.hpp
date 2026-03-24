// Copyright 2026 TIER IV, Inc.
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

#ifndef NEBULA_SEYOND_CALIBRATION_DATA_HPP
#define NEBULA_SEYOND_CALIBRATION_DATA_HPP

#include <nebula_core_common/util/expected.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <variant>
#include <vector>

namespace nebula::drivers
{

/// @brief Calibration data for Seyond LiDARs
struct SeyondCalibrationData
{
  enum class ErrorCode { OPEN_FAILED, PARSE_FAILED, INVALID_CONTENT, OPEN_FOR_WRITE_FAILED };

  struct Error
  {
    ErrorCode code;
    std::string message;
  };

  double v_angle_offset{0.0};
  std::vector<uint8_t> angle_hv_table{};

  /// @brief Load calibration data from a file
  static util::expected<SeyondCalibrationData, Error> load_from_file(
    const std::string & calibration_file)
  {
    std::ifstream file;
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try {
      file.open(calibration_file);
    } catch (const std::exception & e) {
      return Error{
        ErrorCode::OPEN_FAILED,
        "Failed to open calibration file: " + calibration_file + " Error: " + e.what()};
    }

    // placeholder for now
    return SeyondCalibrationData{};
  }

  /// @brief Save calibration data to a file
  util::expected<std::monostate, Error> save_to_file(const std::string & calibration_file)
  {
    std::ofstream file;
    file.exceptions(std::ofstream::failbit | std::ofstream::badbit);

    try {
      file.open(calibration_file);
    } catch (const std::exception & e) {
      return Error{
        ErrorCode::OPEN_FOR_WRITE_FAILED,
        "Failed to open calibration file for writing: " + calibration_file + " Error: " + e.what()};
    }

    // placeholder for now
    return std::monostate{};
  }
};

}  // namespace nebula::drivers

#endif  // NEBULA_SEYOND_CALIBRATION_DATA_HPP
