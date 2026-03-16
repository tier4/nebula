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

#pragma once

#include <nebula_core_common/point_cloud.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
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
  /// @throws std::runtime_error if the file cannot be opened, the header/data are malformed,
  ///         the data is truncated, or the PCD DATA format is unsupported.
  /// @throws std::invalid_argument if numeric tokens in header/data cannot be parsed.
  /// @throws std::out_of_range if numeric tokens in header/data exceed target numeric range.
  template <typename PointT>
  static PointCloud<PointT> read(const std::string & filename)
  {
    static_assert(IsPointType<PointT>::value, "PointT must have a fields() method");
    static_assert(std::is_pod_v<PointT>, "PointT must be a POD type");

    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open PCD file: " + filename);
    }

    const auto header = parse_header(file);

    // Build mapping from PCD fields to target point type fields
    const auto target_fields = PointT::fields();
    const auto target_field_indices = build_target_field_indices(header.fields, target_fields);

    if (header.data_format == "ascii") {
      return read_ascii_points<PointT>(file, header, target_fields, target_field_indices);
    }
    if (header.data_format == "binary") {
      return read_binary_points<PointT>(file, header, target_fields, target_field_indices);
    }
    if (header.data_format == "binary_compressed") {
      throw std::runtime_error("Compressed PCD format is not supported");
    }
    throw std::runtime_error("Unknown DATA format: " + header.data_format);
  }

private:
  struct PcdFieldInfo
  {
    std::string name;
    uint32_t size{0};
    char type{'\0'};
    uint32_t count{1};
    uint32_t offset{0};
  };

  struct PcdHeader
  {
    std::vector<PcdFieldInfo> fields;
    uint32_t width{0};
    uint32_t height{1};
    uint32_t points{0};
    std::string data_format;
    uint32_t point_size{0};
  };

  static void validate_field_value_count(
    const std::vector<std::string> & tokens, size_t field_count, const char * keyword)
  {
    if (tokens.size() - 1 != field_count) {
      throw std::runtime_error(std::string(keyword) + " count does not match FIELDS count");
    }
  }

  static void validate_minimum_token_count(
    const std::vector<std::string> & tokens, size_t min_tokens, const char * keyword)
  {
    if (tokens.size() < min_tokens) {
      throw std::runtime_error(std::string("Malformed PCD header line: ") + keyword);
    }
  }

  static std::vector<PcdFieldInfo> parse_fields(const std::vector<std::string> & tokens)
  {
    validate_minimum_token_count(tokens, 2, "FIELDS");
    std::vector<PcdFieldInfo> fields;
    fields.reserve(tokens.size() - 1);
    for (size_t i = 1; i < tokens.size(); ++i) {
      fields.push_back(PcdFieldInfo{tokens[i]});
    }
    return fields;
  }

  static void parse_sizes(
    const std::vector<std::string> & tokens, std::vector<PcdFieldInfo> & fields)
  {
    validate_minimum_token_count(tokens, 2, "SIZE");
    validate_field_value_count(tokens, fields.size(), "SIZE");
    for (size_t i = 1; i < tokens.size(); ++i) {
      fields[i - 1].size = static_cast<uint32_t>(std::stoul(tokens[i]));
    }
  }

  static void parse_types(
    const std::vector<std::string> & tokens, std::vector<PcdFieldInfo> & fields)
  {
    validate_minimum_token_count(tokens, 2, "TYPE");
    validate_field_value_count(tokens, fields.size(), "TYPE");
    for (size_t i = 1; i < tokens.size(); ++i) {
      fields[i - 1].type = tokens[i][0];
    }
  }

  static void parse_counts(
    const std::vector<std::string> & tokens, std::vector<PcdFieldInfo> & fields)
  {
    validate_minimum_token_count(tokens, 2, "COUNT");
    validate_field_value_count(tokens, fields.size(), "COUNT");
    for (size_t i = 1; i < tokens.size(); ++i) {
      fields[i - 1].count = static_cast<uint32_t>(std::stoul(tokens[i]));
    }
  }

  static uint32_t assign_field_offsets(std::vector<PcdFieldInfo> & fields)
  {
    uint32_t offset = 0;
    for (auto & field : fields) {
      field.offset = offset;
      offset += field.size * field.count;
    }
    return offset;
  }

  static void validate_header(const PcdHeader & header)
  {
    if (header.data_format.empty()) {
      throw std::runtime_error("Malformed PCD header: missing DATA line");
    }
    if (header.fields.empty()) {
      throw std::runtime_error("Malformed PCD header: missing FIELDS line");
    }
    for (const auto & field : header.fields) {
      if (field.size == 0) {
        throw std::runtime_error("Malformed PCD header: missing SIZE line");
      }
      if (field.type == '\0') {
        throw std::runtime_error("Malformed PCD header: missing TYPE line");
      }
      if (field.count == 0) {
        throw std::runtime_error("Malformed PCD header: invalid COUNT value");
      }
    }
  }

  static PcdHeader parse_header(std::istream & file)
  {
    PcdHeader header;
    std::string line;
    while (std::getline(file, line)) {
      line = detail::trim(line);
      if (line.empty() || line[0] == '#') continue;

      const auto tokens = detail::split(line);
      if (tokens.empty()) continue;

      const auto & keyword = tokens[0];
      if (keyword == "VERSION" || keyword == "VIEWPOINT") {
        continue;
      }
      if (keyword == "FIELDS") {
        header.fields = parse_fields(tokens);
      } else if (keyword == "SIZE") {
        parse_sizes(tokens, header.fields);
      } else if (keyword == "TYPE") {
        parse_types(tokens, header.fields);
      } else if (keyword == "COUNT") {
        parse_counts(tokens, header.fields);
      } else if (keyword == "WIDTH") {
        validate_minimum_token_count(tokens, 2, "WIDTH");
        header.width = static_cast<uint32_t>(std::stoul(tokens[1]));
      } else if (keyword == "HEIGHT") {
        validate_minimum_token_count(tokens, 2, "HEIGHT");
        header.height = static_cast<uint32_t>(std::stoul(tokens[1]));
      } else if (keyword == "POINTS") {
        validate_minimum_token_count(tokens, 2, "POINTS");
        header.points = static_cast<uint32_t>(std::stoul(tokens[1]));
      } else if (keyword == "DATA") {
        validate_minimum_token_count(tokens, 2, "DATA");
        header.data_format = tokens[1];
        break;
      }
    }

    validate_header(header);

    header.point_size = assign_field_offsets(header.fields);
    if (header.points == 0) {
      header.points = header.width * header.height;
    }
    return header;
  }

  template <typename PcdFieldsT, typename FieldsT>
  static std::vector<int32_t> build_target_field_indices(
    const PcdFieldsT & pcd_fields, const FieldsT & target_fields)
  {
    std::vector<int32_t> target_field_indices(pcd_fields.size(), -1);
    for (size_t pcd_idx = 0; pcd_idx < pcd_fields.size(); ++pcd_idx) {
      for (size_t target_idx = 0; target_idx < target_fields.size(); ++target_idx) {
        if (pcd_fields[pcd_idx].name == target_fields[target_idx].name) {
          target_field_indices[pcd_idx] = static_cast<int32_t>(target_idx);
          break;
        }
      }
    }
    return target_field_indices;
  }

  static size_t scalar_count(const std::vector<PcdFieldInfo> & pcd_fields)
  {
    size_t count = 0;
    for (const auto & field : pcd_fields) {
      count += field.count;
    }
    return count;
  }

  static void parse_ascii_scalar(
    uint8_t * dst, PointField::DataType datatype, const std::string & value)
  {
    switch (datatype) {
      case PointField::DataType::Float32:
        *reinterpret_cast<float *>(dst) = detail::parse_value<float>(value);
        break;
      case PointField::DataType::Float64:
        *reinterpret_cast<double *>(dst) = detail::parse_value<double>(value);
        break;
      case PointField::DataType::Int8:
        *reinterpret_cast<int8_t *>(dst) = detail::parse_value<int8_t>(value);
        break;
      case PointField::DataType::UInt8:
        *reinterpret_cast<uint8_t *>(dst) = detail::parse_value<uint8_t>(value);
        break;
      case PointField::DataType::Int16:
        *reinterpret_cast<int16_t *>(dst) = detail::parse_value<int16_t>(value);
        break;
      case PointField::DataType::UInt16:
        *reinterpret_cast<uint16_t *>(dst) = detail::parse_value<uint16_t>(value);
        break;
      case PointField::DataType::Int32:
        *reinterpret_cast<int32_t *>(dst) = detail::parse_value<int32_t>(value);
        break;
      case PointField::DataType::UInt32:
        *reinterpret_cast<uint32_t *>(dst) = detail::parse_value<uint32_t>(value);
        break;
    }
  }

  template <typename PointT, typename FieldsT>
  static PointT parse_ascii_point(
    const std::string & line, const std::vector<PcdFieldInfo> & pcd_fields,
    const FieldsT & target_fields, const std::vector<int32_t> & target_field_indices)
  {
    const auto values = detail::split(line);
    if (values.size() < scalar_count(pcd_fields)) {
      throw std::runtime_error("Not enough values in ASCII line");
    }

    PointT point{};
    size_t value_idx = 0;
    for (size_t pcd_idx = 0; pcd_idx < pcd_fields.size(); ++pcd_idx) {
      const auto & pcd_field = pcd_fields[pcd_idx];
      const int32_t target_idx = target_field_indices[pcd_idx];
      if (target_idx >= 0) {
        const auto & target_field = target_fields[static_cast<size_t>(target_idx)];
        auto * const ptr = reinterpret_cast<uint8_t *>(&point) + target_field.offset;
        parse_ascii_scalar(ptr, target_field.datatype, values[value_idx]);
      }
      value_idx += pcd_field.count;
    }
    return point;
  }

  template <typename PointT, typename FieldsT>
  static PointT parse_binary_point(
    const uint8_t * pcd_point_data, const std::vector<PcdFieldInfo> & pcd_fields,
    const FieldsT & target_fields, const std::vector<int32_t> & target_field_indices)
  {
    PointT point{};
    for (size_t pcd_idx = 0; pcd_idx < pcd_fields.size(); ++pcd_idx) {
      const auto & pcd_field = pcd_fields[pcd_idx];
      const int32_t target_idx = target_field_indices[pcd_idx];
      if (target_idx < 0) continue;

      const auto & target_field = target_fields[static_cast<size_t>(target_idx)];
      auto * const dst = reinterpret_cast<uint8_t *>(&point) + target_field.offset;
      const auto * const src = pcd_point_data + pcd_field.offset;
      const auto copy_size = std::min(
        pcd_field.size * pcd_field.count,
        detail::datatype_size(target_field.datatype) * target_field.count);
      std::memcpy(dst, src, copy_size);
    }
    return point;
  }

  template <typename PointT, typename FieldsT>
  static PointCloud<PointT> read_ascii_points(
    std::istream & file, const PcdHeader & header, const FieldsT & target_fields,
    const std::vector<int32_t> & target_field_indices)
  {
    PointCloud<PointT> cloud;
    cloud.reserve(header.points);

    std::string line;
    for (uint32_t i = 0; i < header.points; ++i) {
      if (!std::getline(file, line)) {
        throw std::runtime_error("Unexpected end of file while reading ASCII data");
      }
      cloud.push_back(
        parse_ascii_point<PointT>(line, header.fields, target_fields, target_field_indices));
    }
    return cloud;
  }

  template <typename PointT, typename FieldsT>
  static PointCloud<PointT> read_binary_points(
    std::istream & file, const PcdHeader & header, const FieldsT & target_fields,
    const std::vector<int32_t> & target_field_indices)
  {
    PointCloud<PointT> cloud;
    cloud.reserve(header.points);

    std::vector<uint8_t> pcd_point_buffer(header.point_size);
    for (uint32_t i = 0; i < header.points; ++i) {
      file.read(reinterpret_cast<char *>(pcd_point_buffer.data()), header.point_size);
      if (!file) {
        throw std::runtime_error("Unexpected end of file while reading binary data");
      }
      cloud.push_back(
        parse_binary_point<PointT>(
          pcd_point_buffer.data(), header.fields, target_fields, target_field_indices));
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
  /// @throws std::runtime_error if the output file cannot be opened or written.
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
    write_binary_header(file, fields, cloud.size(), sizeof(PointT));

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

private:
  template <typename FieldsT, typename ValueFn>
  static void write_field_line(
    std::ostream & file, const char * key, const FieldsT & fields, ValueFn && value_fn)
  {
    file << key;
    for (const auto & field : fields) {
      file << " " << value_fn(field);
    }
    file << "\n";
  }

  template <typename FieldsT>
  static auto build_binary_header_fields(const FieldsT & fields, size_t point_size)
  {
    using FieldT = std::decay_t<decltype(*std::begin(fields))>;
    static_assert(std::is_same_v<FieldT, PointField>, "FieldsT must be a container of PointField");

    auto padding_field = [](uint32_t offset, uint32_t size) {
      return PointField{"_", offset, PointField::DataType::UInt8, size};
    };

    std::vector<PointField> ordered_fields(std::begin(fields), std::end(fields));
    std::sort(ordered_fields.begin(), ordered_fields.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.offset < rhs.offset;
    });

    std::vector<PointField> header_fields;
    header_fields.reserve(ordered_fields.size() + 1U);

    uint32_t current_offset = 0;
    for (const auto & field : ordered_fields) {
      if (field.offset < current_offset) {
        throw std::runtime_error("PointField offsets overlap, cannot write binary PCD");
      }
      if (field.offset > current_offset) {
        header_fields.push_back(padding_field(current_offset, field.offset - current_offset));
        current_offset = field.offset;
      }

      header_fields.push_back(field);
      uint32_t field_size = detail::datatype_size(field.datatype) * field.count;
      current_offset = std::max(current_offset, field.offset + field_size);
    }

    if (current_offset < point_size) {
      header_fields.push_back(padding_field(current_offset, point_size - current_offset));
    }
    return header_fields;
  }

  template <typename FieldsT>
  static void write_binary_header(
    std::ostream & file, const FieldsT & fields, size_t point_count, size_t point_size)
  {
    const auto padded_fields = build_binary_header_fields(fields, point_size);

    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    write_field_line(file, "FIELDS", padded_fields, [](const auto & field) { return field.name; });
    write_field_line(file, "SIZE", padded_fields, [](const auto & field) {
      return detail::datatype_size(field.datatype);
    });
    write_field_line(file, "TYPE", padded_fields, [](const auto & field) {
      return detail::datatype_to_pcd_type(field.datatype);
    });
    write_field_line(file, "COUNT", padded_fields, [](const auto & field) { return field.count; });
    file << "WIDTH " << point_count << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << point_count << "\n";
    file << "DATA binary\n";
  }
};

}  // namespace nebula::drivers::io
