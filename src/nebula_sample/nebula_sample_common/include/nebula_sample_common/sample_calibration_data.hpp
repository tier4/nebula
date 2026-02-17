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

/// @brief Calibration data for the Sample LiDAR (optional)
/// @details This struct is only needed if your sensor requires calibration data.
/// Common calibration data includes:
/// - Vertical/horizontal angle corrections per laser
/// - Distance corrections
/// - Intensity corrections
/// - Timing offsets
/// If your sensor doesn't need calibration, you can remove this struct entirely.
struct SampleCalibrationData
{
  enum class ErrorCode : uint8_t {
    OPEN_FOR_READ_FAILED,
    OPEN_FOR_WRITE_FAILED,
  };

  struct Error
  {
    ErrorCode code;
    std::string message;
  };

  /// @brief Load calibration data from a file
  /// @param calibration_file Path to the calibration file
  /// @return Calibration data on success, error otherwise
  static util::expected<SampleCalibrationData, Error> load_from_file(
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

    // Parse the file and populate calibration data fields here

    return SampleCalibrationData{};
  }

  /// @brief Save calibration data to a file
  /// @param calibration_file Path to save the calibration file
  /// @return Nothing on success, error otherwise
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

    // Implementation Items: Implement calibration file writing

    return std::monostate{};
  }
};

}  // namespace nebula::drivers
