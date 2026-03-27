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
#include <array>
#include <cstdint>
#include <cstring>
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
  /// @brief Raw binary geo yaml blob fetched from sensor (or loaded from file)
  std::vector<uint8_t> geo_yaml{};
  /// @brief Raw binary serial-number yaml blob fetched from sensor (or loaded from file)
  std::vector<uint8_t> sn_yaml{};

  /// @brief Load calibration data from a binary file
  static util::expected<SeyondCalibrationData, Error> load_from_file(
    const std::string & calibration_file)
  {
    constexpr std::array<char, 12> k_magic{'S', 'E', 'Y', 'O', 'N', 'D',
                                           '_', 'C', 'A', 'L', 'I', 'B'};

    std::ifstream file(calibration_file, std::ios::binary);
    if (!file.is_open()) {
      return Error{ErrorCode::OPEN_FAILED, "Failed to open calibration file: " + calibration_file};
    }

    SeyondCalibrationData calibration;

    std::array<char, k_magic.size()> magic{};
    file.read(magic.data(), static_cast<std::streamsize>(magic.size()));
    if (file.good() && std::memcmp(magic.data(), k_magic.data(), k_magic.size()) == 0) {
      uint32_t version = 0;
      uint32_t angle_size = 0;
      uint32_t geo_size = 0;
      uint32_t sn_size = 0;

      file.read(reinterpret_cast<char *>(&version), sizeof(version));
      file.read(
        reinterpret_cast<char *>(&calibration.v_angle_offset), sizeof(calibration.v_angle_offset));
      file.read(reinterpret_cast<char *>(&angle_size), sizeof(angle_size));
      file.read(reinterpret_cast<char *>(&geo_size), sizeof(geo_size));
      file.read(reinterpret_cast<char *>(&sn_size), sizeof(sn_size));

      if (!file.good() || version != 1U) {
        return Error{
          ErrorCode::PARSE_FAILED, "Failed to parse calibration file header: " + calibration_file};
      }

      calibration.angle_hv_table.resize(angle_size);
      calibration.geo_yaml.resize(geo_size);
      calibration.sn_yaml.resize(sn_size);

      file.read(
        reinterpret_cast<char *>(calibration.angle_hv_table.data()),
        static_cast<std::streamsize>(calibration.angle_hv_table.size()));
      file.read(
        reinterpret_cast<char *>(calibration.geo_yaml.data()),
        static_cast<std::streamsize>(calibration.geo_yaml.size()));
      file.read(
        reinterpret_cast<char *>(calibration.sn_yaml.data()),
        static_cast<std::streamsize>(calibration.sn_yaml.size()));

      if (!file.good() && !file.eof()) {
        return Error{
          ErrorCode::PARSE_FAILED, "Failed to parse calibration payload: " + calibration_file};
      }

      return calibration;
    }

    file.clear();
    file.seekg(0, std::ios::beg);

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
    constexpr std::array<char, 12> k_magic{'S', 'E', 'Y', 'O', 'N', 'D',
                                           '_', 'C', 'A', 'L', 'I', 'B'};

    std::ofstream file(calibration_file, std::ios::binary | std::ios::trunc);
    if (!file.is_open()) {
      return Error{
        ErrorCode::OPEN_FOR_WRITE_FAILED,
        "Failed to open calibration file for writing: " + calibration_file};
    }

    const uint32_t version = 1U;
    const auto angle_size = static_cast<uint32_t>(angle_hv_table.size());
    const auto geo_size = static_cast<uint32_t>(geo_yaml.size());
    const auto sn_size = static_cast<uint32_t>(sn_yaml.size());

    file.write(k_magic.data(), static_cast<std::streamsize>(k_magic.size()));
    file.write(reinterpret_cast<const char *>(&version), sizeof(version));
    file.write(reinterpret_cast<const char *>(&v_angle_offset), sizeof(v_angle_offset));
    file.write(reinterpret_cast<const char *>(&angle_size), sizeof(angle_size));
    file.write(reinterpret_cast<const char *>(&geo_size), sizeof(geo_size));
    file.write(reinterpret_cast<const char *>(&sn_size), sizeof(sn_size));

    if (!angle_hv_table.empty()) {
      file.write(
        reinterpret_cast<const char *>(angle_hv_table.data()),
        static_cast<std::streamsize>(angle_hv_table.size()));
    }
    if (!geo_yaml.empty()) {
      file.write(
        reinterpret_cast<const char *>(geo_yaml.data()),
        static_cast<std::streamsize>(geo_yaml.size()));
    }
    if (!sn_yaml.empty()) {
      file.write(
        reinterpret_cast<const char *>(sn_yaml.data()),
        static_cast<std::streamsize>(sn_yaml.size()));
    }

    if (!file.good()) {
      return Error{
        ErrorCode::OPEN_FOR_WRITE_FAILED, "Failed to write calibration file: " + calibration_file};
    }

    return std::monostate{};
  }
};

}  // namespace nebula::drivers

#endif  // NEBULA_SEYOND_CALIBRATION_DATA_HPP
