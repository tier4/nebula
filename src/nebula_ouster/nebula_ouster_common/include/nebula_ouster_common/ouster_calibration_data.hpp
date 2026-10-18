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

#include <ouster/types.h>

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <variant>

namespace nebula::drivers
{

/// @brief Calibration data for the Ouster LiDAR.
/// @details Wraps the Ouster sensor metadata JSON (equivalent to ouster::sdk::core::SensorInfo).
/// The JSON metadata contains beam angles, transforms, data format, and other sensor-specific
/// calibration needed by the decoder. This struct keeps the common package SDK-free by storing
/// the raw JSON string; the decoder package constructs SensorInfo from it.
struct OusterCalibrationData
{
  enum class ErrorCode : uint8_t {
    OPEN_FOR_READ_FAILED,   ///< Opening a calibration file for reading failed.
    OPEN_FOR_WRITE_FAILED,  ///< Opening a calibration file for writing failed.
    EMPTY_METADATA,         ///< Metadata JSON string is empty.
  };

  /// @brief Error payload for calibration file operations.
  struct Error
  {
    ErrorCode code;
    std::string message;
  };

  /// @brief The path from which calibration was loaded (for diagnostics/logging).
  std::string calibration_file;

  /// @brief Raw sensor metadata JSON string (Ouster SensorInfo serialization).
  std::string metadata_json;
  /// @brief Ouster SensorInfo object constructed from the metadata JSON.
  std::shared_ptr<ouster::sdk::core::SensorInfo> sensor_info;
  /// @brief Ouster PacketFormat object constructed from the SensorInfo.
  std::shared_ptr<ouster::sdk::core::PacketFormat> packet_format;

  /// @brief Load calibration data from a JSON metadata file (e.g. for offline decoding).
  /// @param calibration_file Path to the JSON metadata file.
  /// @return Parsed calibration data on success, Error on failure.
  [[nodiscard]] static util::expected<OusterCalibrationData, Error> load_from_file(
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

    std::stringstream buf;
    buf << file.rdbuf();
    std::string json_content = buf.str();

    if (json_content.empty()) {
      return Error{ErrorCode::EMPTY_METADATA, "Calibration file is empty: " + calibration_file};
    }

    OusterCalibrationData data;
    data.calibration_file = calibration_file;
    data.metadata_json = std::move(json_content);
    data.sensor_info = std::make_shared<ouster::sdk::core::SensorInfo>(data.metadata_json);
    data.packet_format = std::make_shared<ouster::sdk::core::PacketFormat>(*data.sensor_info);
    return data;
  }

  /// @brief Create calibration data from a metadata JSON string (e.g. fetched from sensor HTTP).
  /// @param metadata_json_str The raw JSON metadata string from the sensor.
  /// @return Calibration data on success, Error if the string is empty.
  [[nodiscard]] static util::expected<OusterCalibrationData, Error> load_from_string(
    const std::string & metadata_json_str)
  {
    if (metadata_json_str.empty()) {
      return Error{ErrorCode::EMPTY_METADATA, "Metadata JSON string is empty"};
    }

    OusterCalibrationData data;
    data.metadata_json = metadata_json_str;
    data.sensor_info = std::make_shared<ouster::sdk::core::SensorInfo>(metadata_json_str);
    data.packet_format = std::make_shared<ouster::sdk::core::PacketFormat>(*data.sensor_info);
    return data;
  }

  /// @brief Save calibration data (metadata JSON) to a file.
  /// @param calibration_file Path to save the calibration file.
  /// @return std::monostate on success, Error on failure.
  [[nodiscard]] util::expected<std::monostate, Error> save_to_file(
    const std::string & calibration_file) const
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

    file << metadata_json;
    return std::monostate{};
  }
};

}  // namespace nebula::drivers
