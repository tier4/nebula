// Copyright 2025 TIER IV, Inc.
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

#include <nebula_core_common/util/expected.hpp>

#include <cstdint>
#include <fstream>
#include <string>
#include <variant>

namespace nebula::drivers
{

/// @brief Calibration data for the Ouster LiDAR (required for some sensors)
/// @details Real sensor integrations can replace this stub with calibration tables
/// such as per-laser angle offsets, distance corrections, or timing offsets.
struct OusterCalibrationData
{
  enum class ErrorCode : uint8_t {
    OPEN_FOR_READ_FAILED,   ///< Opening a calibration file for reading failed.
    OPEN_FOR_WRITE_FAILED,  ///< Opening a calibration file for writing failed.
  };

  /// @brief Error payload for calibration file operations.
  struct Error
  {
    ErrorCode code;
    std::string message;
  };

  /// @brief Load calibration data from a file, e.g. for offline decoding
  /// @param calibration_file Path to the calibration file
  /// @return Parsed calibration data on success, Error on failure.
  static util::expected<OusterCalibrationData, Error> load_from_file(
    const std::string & calibration_file)
  {
    std::ifstream file;
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try {
      file.open(calibration_file);
    } catch (const std::ifstream::failure & e) {
      return Error{
        ErrorCode::OPEN_FOR_READ_FAILED,
        "Failed to open calibration file: " + calibration_file + " Error: " + e.what()};
    }

    // Implement: Parse and validate sensor calibration fields from the opened file.
    return OusterCalibrationData{};
  }

  /// @brief Save calibration data to a file
  /// @param calibration_file Path to save the calibration file
  /// @return std::monostate on success, Error on failure.
  util::expected<std::monostate, Error> save_to_file(const std::string & calibration_file)
  {
    std::ofstream file;
    file.exceptions(std::ofstream::failbit | std::ofstream::badbit);

    try {
      file.open(calibration_file);
    } catch (const std::ofstream::failure & e) {
      return Error{
        ErrorCode::OPEN_FOR_WRITE_FAILED,
        "Failed to open calibration file for writing: " + calibration_file + " Error: " + e.what()};
    }

    // Implement: Serialize sensor calibration fields to the opened file.
    return std::monostate{};
  }
};

}  // namespace nebula::drivers
