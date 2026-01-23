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

#include <nebula_core_common/point_cloud.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace nebula::drivers::io
{

namespace detail
{

/// @brief Get size in bytes for a PCD type character and size
inline uint32_t pcd_type_size(char type, uint32_t size)
{
  (void)type;  // Type character is informational; size is authoritative
  return size;
}

/// @brief Convert PointField::DataType to PCD type character
inline char datatype_to_pcd_type(PointField::DataType datatype)
{
  switch (datatype) {
    case PointField::DataType::Int8:
    case PointField::DataType::Int16:
    case PointField::DataType::Int32:
      return 'I';
    case PointField::DataType::UInt8:
    case PointField::DataType::UInt16:
    case PointField::DataType::UInt32:
      return 'U';
    case PointField::DataType::Float32:
    case PointField::DataType::Float64:
      return 'F';
    default:
      throw std::runtime_error("Invalid PointField::DataType");
  }
}

/// @brief Get size in bytes for a PointField::DataType
inline uint32_t datatype_size(PointField::DataType datatype)
{
  switch (datatype) {
    case PointField::DataType::Int8:
    case PointField::DataType::UInt8:
      return 1;
    case PointField::DataType::Int16:
    case PointField::DataType::UInt16:
      return 2;
    case PointField::DataType::Int32:
    case PointField::DataType::UInt32:
    case PointField::DataType::Float32:
      return 4;
    case PointField::DataType::Float64:
      return 8;
    default:
      throw std::runtime_error("Invalid PointField::DataType");
  }
}

/// @brief Trim whitespace from string
inline std::string trim(const std::string & str)
{
  const auto start = str.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) return "";
  const auto end = str.find_last_not_of(" \t\r\n");
  return str.substr(start, end - start + 1);
}

/// @brief Split string by whitespace
inline std::vector<std::string> split(const std::string & str)
{
  std::vector<std::string> tokens;
  std::istringstream iss(str);
  std::string token;
  while (iss >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

/// @brief Structure to hold PCD field information
struct PcdFieldInfo
{
  std::string name;
  uint32_t size;
  char type;
  uint32_t count;
  uint32_t offset;
};

/// @brief Parse a value from ASCII string to the target type
template <typename T>
T parse_value(const std::string & str)
{
  if constexpr (std::is_floating_point_v<T>) {
    return static_cast<T>(std::stod(str));
  } else if constexpr (std::is_signed_v<T>) {
    return static_cast<T>(std::stoll(str));
  } else {
    return static_cast<T>(std::stoull(str));
  }
}

/// @brief Write a value to ASCII string
template <typename T>
std::string value_to_string(T value)
{
  if constexpr (std::is_floating_point_v<T>) {
    std::ostringstream oss;
    oss << std::scientific << value;
    return oss.str();
  } else if constexpr (std::is_same_v<T, int8_t>) {
    return std::to_string(static_cast<int>(value));
  } else if constexpr (std::is_same_v<T, uint8_t>) {
    return std::to_string(static_cast<unsigned int>(value));
  } else {
    return std::to_string(value);
  }
}

}  // namespace detail

/// @brief PCD file reader supporting ASCII and binary formats
class PcdReader
{
public:
  /// @brief Read a PCD file into a PointCloud
  /// @tparam PointT The target point type (must have a static fields() method)
  /// @param filename Path to the PCD file
  /// @return The loaded point cloud
  /// @throws std::runtime_error if the file cannot be read or is malformed
  template <typename PointT>
  static PointCloud<PointT> read(const std::string & filename)
  {
    static_assert(IsPointType<PointT>::value, "PointT must have a fields() method");
    static_assert(std::is_pod_v<PointT>, "PointT must be a POD type");

    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open PCD file: " + filename);
    }

    // Parse header
    std::vector<detail::PcdFieldInfo> pcd_fields;
    uint32_t width = 0;
    uint32_t height = 1;
    uint32_t points = 0;
    std::string data_format;
    uint32_t pcd_point_size = 0;

    std::string line;
    while (std::getline(file, line)) {
      line = detail::trim(line);
      if (line.empty() || line[0] == '#') continue;

      auto tokens = detail::split(line);
      if (tokens.empty()) continue;

      const std::string & keyword = tokens[0];

      if (keyword == "VERSION") {
        // Version 0.7 is expected
        continue;
      } else if (keyword == "FIELDS") {
        for (size_t i = 1; i < tokens.size(); ++i) {
          detail::PcdFieldInfo field;
          field.name = tokens[i];
          pcd_fields.push_back(field);
        }
      } else if (keyword == "SIZE") {
        if (tokens.size() - 1 != pcd_fields.size()) {
          throw std::runtime_error("SIZE count does not match FIELDS count");
        }
        for (size_t i = 1; i < tokens.size(); ++i) {
          pcd_fields[i - 1].size = static_cast<uint32_t>(std::stoul(tokens[i]));
        }
      } else if (keyword == "TYPE") {
        if (tokens.size() - 1 != pcd_fields.size()) {
          throw std::runtime_error("TYPE count does not match FIELDS count");
        }
        for (size_t i = 1; i < tokens.size(); ++i) {
          pcd_fields[i - 1].type = tokens[i][0];
        }
      } else if (keyword == "COUNT") {
        if (tokens.size() - 1 != pcd_fields.size()) {
          throw std::runtime_error("COUNT count does not match FIELDS count");
        }
        for (size_t i = 1; i < tokens.size(); ++i) {
          pcd_fields[i - 1].count = static_cast<uint32_t>(std::stoul(tokens[i]));
        }
      } else if (keyword == "WIDTH") {
        width = static_cast<uint32_t>(std::stoul(tokens[1]));
      } else if (keyword == "HEIGHT") {
        height = static_cast<uint32_t>(std::stoul(tokens[1]));
      } else if (keyword == "VIEWPOINT") {
        // Ignore viewpoint
        continue;
      } else if (keyword == "POINTS") {
        points = static_cast<uint32_t>(std::stoul(tokens[1]));
      } else if (keyword == "DATA") {
        data_format = tokens[1];
        break;  // Header ends here
      }
    }

    // Calculate offsets and total point size for PCD format
    uint32_t offset = 0;
    for (auto & field : pcd_fields) {
      field.offset = offset;
      offset += field.size * field.count;
    }
    pcd_point_size = offset;

    if (points == 0) {
      points = width * height;
    }

    // Build mapping from PCD fields to target point type fields
    const auto target_fields = PointT::fields();
    std::unordered_map<std::string, size_t> target_field_map;
    for (size_t i = 0; i < target_fields.size(); ++i) {
      target_field_map[target_fields[i].name] = i;
    }

    PointCloud<PointT> cloud;
    cloud.reserve(points);

    if (data_format == "ascii") {
      // Read ASCII data
      for (uint32_t i = 0; i < points; ++i) {
        if (!std::getline(file, line)) {
          throw std::runtime_error("Unexpected end of file while reading ASCII data");
        }
        auto values = detail::split(line);
        if (values.size() < pcd_fields.size()) {
          throw std::runtime_error("Not enough values in ASCII line");
        }

        PointT point{};
        size_t value_idx = 0;
        for (const auto & pcd_field : pcd_fields) {
          auto it = target_field_map.find(pcd_field.name);
          if (it != target_field_map.end()) {
            const auto & target_field = target_fields[it->second];
            uint8_t * ptr = reinterpret_cast<uint8_t *>(&point) + target_field.offset;

            // Parse value based on target type
            switch (target_field.datatype) {
              case PointField::DataType::Float32:
                *reinterpret_cast<float *>(ptr) = detail::parse_value<float>(values[value_idx]);
                break;
              case PointField::DataType::Float64:
                *reinterpret_cast<double *>(ptr) = detail::parse_value<double>(values[value_idx]);
                break;
              case PointField::DataType::Int8:
                *reinterpret_cast<int8_t *>(ptr) = detail::parse_value<int8_t>(values[value_idx]);
                break;
              case PointField::DataType::UInt8:
                *reinterpret_cast<uint8_t *>(ptr) = detail::parse_value<uint8_t>(values[value_idx]);
                break;
              case PointField::DataType::Int16:
                *reinterpret_cast<int16_t *>(ptr) = detail::parse_value<int16_t>(values[value_idx]);
                break;
              case PointField::DataType::UInt16:
                *reinterpret_cast<uint16_t *>(ptr) =
                  detail::parse_value<uint16_t>(values[value_idx]);
                break;
              case PointField::DataType::Int32:
                *reinterpret_cast<int32_t *>(ptr) = detail::parse_value<int32_t>(values[value_idx]);
                break;
              case PointField::DataType::UInt32:
                *reinterpret_cast<uint32_t *>(ptr) =
                  detail::parse_value<uint32_t>(values[value_idx]);
                break;
            }
          }
          value_idx += pcd_field.count;
        }
        cloud.push_back(point);
      }
    } else if (data_format == "binary") {
      // Read binary data
      std::vector<uint8_t> pcd_point_buffer(pcd_point_size);

      for (uint32_t i = 0; i < points; ++i) {
        file.read(reinterpret_cast<char *>(pcd_point_buffer.data()), pcd_point_size);
        if (!file) {
          throw std::runtime_error("Unexpected end of file while reading binary data");
        }

        PointT point{};
        for (const auto & pcd_field : pcd_fields) {
          auto it = target_field_map.find(pcd_field.name);
          if (it != target_field_map.end()) {
            const auto & target_field = target_fields[it->second];
            uint8_t * dst = reinterpret_cast<uint8_t *>(&point) + target_field.offset;
            const uint8_t * src = pcd_point_buffer.data() + pcd_field.offset;

            // Copy the minimum of source and destination sizes
            uint32_t copy_size = std::min(
              pcd_field.size * pcd_field.count,
              detail::datatype_size(target_field.datatype) * target_field.count);
            std::memcpy(dst, src, copy_size);
          }
        }
        cloud.push_back(point);
      }
    } else if (data_format == "binary_compressed") {
      throw std::runtime_error("Compressed PCD format is not supported");
    } else {
      throw std::runtime_error("Unknown DATA format: " + data_format);
    }

    return cloud;
  }
};

/// @brief PCD file writer supporting binary format only
class PcdWriter
{
public:
  /// @brief Write a PointCloud to a binary PCD file
  /// @tparam PointT The point type (must have a static fields() method)
  /// @param filename Path to the output PCD file
  /// @param cloud The point cloud to write
  /// @throws std::runtime_error if the file cannot be written
  template <typename PointT>
  static void write_binary(const std::string & filename, const PointCloud<PointT> & cloud)
  {
    static_assert(IsPointType<PointT>::value, "PointT must have a fields() method");
    static_assert(std::is_pod_v<PointT>, "PointT must be a POD type");

    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open PCD file for writing: " + filename);
    }

    const auto fields = PointT::fields();

    // Write header
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";

    // FIELDS
    file << "FIELDS";
    for (const auto & field : fields) {
      file << " " << field.name;
    }
    file << "\n";

    // SIZE
    file << "SIZE";
    for (const auto & field : fields) {
      file << " " << detail::datatype_size(field.datatype);
    }
    file << "\n";

    // TYPE
    file << "TYPE";
    for (const auto & field : fields) {
      file << " " << detail::datatype_to_pcd_type(field.datatype);
    }
    file << "\n";

    // COUNT
    file << "COUNT";
    for (const auto & field : fields) {
      file << " " << field.count;
    }
    file << "\n";

    // Dimensions
    file << "WIDTH " << cloud.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << cloud.size() << "\n";
    file << "DATA binary\n";

    // Write binary data
    // Note: We write the entire point structure, which may include padding bytes.
    // This is consistent with how PCL writes binary PCD files.
    if (!cloud.empty()) {
      file.write(reinterpret_cast<const char *>(cloud.data()), cloud.size() * sizeof(PointT));
    }

    if (!file) {
      throw std::runtime_error("Error writing PCD file: " + filename);
    }
  }

  /// @brief Write a shared_ptr PointCloud to a binary PCD file
  /// @tparam PointT The point type (must have a static fields() method)
  /// @param filename Path to the output PCD file
  /// @param cloud The point cloud to write
  /// @throws std::runtime_error if the file cannot be written
  template <typename PointT>
  static void write_binary(
    const std::string & filename, const std::shared_ptr<PointCloud<PointT>> & cloud)
  {
    if (!cloud) {
      throw std::runtime_error("Cannot write null point cloud");
    }
    write_binary(filename, *cloud);
  }

  /// @brief Write a shared_ptr to const PointCloud to a binary PCD file
  /// @tparam PointT The point type (must have a static fields() method)
  /// @param filename Path to the output PCD file
  /// @param cloud The point cloud to write
  /// @throws std::runtime_error if the file cannot be written
  template <typename PointT>
  static void write_binary(
    const std::string & filename, const std::shared_ptr<const PointCloud<PointT>> & cloud)
  {
    if (!cloud) {
      throw std::runtime_error("Cannot write null point cloud");
    }
    write_binary(filename, *cloud);
  }
};

}  // namespace nebula::drivers::io
