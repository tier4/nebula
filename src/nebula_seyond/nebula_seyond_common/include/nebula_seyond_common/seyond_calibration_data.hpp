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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
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

  /// @brief v_angle_offset for FalconK/RobinW coarse calibration fallback
  double v_angle_offset{0.0};
  /// @brief Raw binary anglehv_table fetched from sensor (or loaded from file)
  std::vector<uint8_t> angle_hv_table{};

  /// @brief Load calibration data from a binary file
  static util::expected<SeyondCalibrationData, Error> load_from_file(
    const std::string & calibration_file)
  {
    std::ifstream file(calibration_file, std::ios::binary);
    if (!file.is_open()) {
      return Error{ErrorCode::OPEN_FAILED, "Failed to open calibration file: " + calibration_file};
    }

    SeyondCalibrationData calibration;

    // First line: optional v_angle_offset as a text value
    // Remaining bytes: raw binary anglehv_table
    std::string first_line;
    std::getline(file, first_line);
    if (!first_line.empty()) {
      try {
        calibration.v_angle_offset = std::stod(first_line);
      } catch (const std::exception &) {
        // Not a numeric first line — treat whole file as binary table
        file.seekg(0, std::ios::beg);
      }
    }

    // Read remaining bytes as anglehv_table binary blob
    calibration.angle_hv_table.assign(
      std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());

    return calibration;
  }

  /// @brief Save calibration data to a binary file
  util::expected<std::monostate, Error> save_to_file(const std::string & calibration_file) const
  {
    std::ofstream file(calibration_file, std::ios::binary | std::ios::trunc);
    if (!file.is_open()) {
      return Error{
        ErrorCode::OPEN_FOR_WRITE_FAILED,
        "Failed to open calibration file for writing: " + calibration_file};
    }

    // Write v_angle_offset as first line if non-zero
    if (v_angle_offset != 0.0) {
      file << v_angle_offset << "\n";
    }

    // Write raw binary table
    if (!angle_hv_table.empty()) {
      file.write(
        reinterpret_cast<const char *>(angle_hv_table.data()),
        static_cast<std::streamsize>(angle_hv_table.size()));
    }

    return std::monostate{};
  }
};

}  // namespace nebula::drivers

#endif  // NEBULA_SEYOND_CALIBRATION_DATA_HPP
