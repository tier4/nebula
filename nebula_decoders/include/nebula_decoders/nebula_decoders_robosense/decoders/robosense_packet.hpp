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

#include <nebula_common/robosense/robosense_common.hpp>

#include <boost/endian/buffers.hpp>

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

using namespace boost::endian;  // NOLINT(build/namespaces)

namespace nebula::drivers::robosense_packet
{

#pragma pack(push, 1)

struct Timestamp
{
  big_uint48_buf_t seconds;
  big_uint32_buf_t microseconds;

  uint64_t get_time_in_ns() const
  {
    constexpr uint64_t ns_in_second = 1000000000ULL;
    constexpr uint64_t ns_in_microsecond = 1000ULL;

    uint64_t total_nanoseconds = seconds.value() * ns_in_second;
    total_nanoseconds += microseconds.value() * ns_in_microsecond;

    return total_nanoseconds;
  }
};

struct Unit
{
  big_uint16_buf_t distance;
  big_uint8_buf_t reflectivity;
};

template <typename UnitT, size_t UnitN>
struct Block
{
  big_uint16_buf_t flag;
  big_uint16_buf_t azimuth;
  UnitT units[UnitN];
  typedef UnitT unit_t;

  [[nodiscard]] uint16_t get_azimuth() const { return azimuth.value(); }
};

template <typename BlockT, size_t BlockN>
struct Body
{
  typedef BlockT block_t;
  BlockT blocks[BlockN];
};

/// @brief Base struct for all Robosense packets. This struct is not allowed to have any non-static
/// members, otherwise memory layout is not guaranteed for the derived structs.
/// @tparam nBlocks The number of blocks in the packet
/// @tparam nChannels The number of channels per block
/// @tparam maxReturns The maximum number of returns, e.g. 2 for dual return
/// @tparam degreeSubdivisions The resolution of the azimuth angle in the packet, e.g. 100 if packet
/// azimuth is given in 1/100th of a degree
template <size_t nBlocks, size_t nChannels, size_t maxReturns, size_t degreeSubdivisions>
struct PacketBase
{
  static constexpr size_t n_blocks = nBlocks;
  static constexpr size_t n_channels = nChannels;
  static constexpr size_t max_returns = maxReturns;
  static constexpr size_t degree_subdivisions = degreeSubdivisions;
};

struct IpAddress
{
  big_uint8_buf_t first_octet;
  big_uint8_buf_t second_octet;
  big_uint8_buf_t third_octet;
  big_uint8_buf_t fourth_octet;

  [[nodiscard]] std::string to_string() const
  {
    return std::to_string(first_octet.value()) + "." + std::to_string(second_octet.value()) + "." +
           std::to_string(third_octet.value()) + "." + std::to_string(fourth_octet.value());
  }
};

struct MacAddress
{
  big_uint8_buf_t first_octet;
  big_uint8_buf_t second_octet;
  big_uint8_buf_t third_octet;
  big_uint8_buf_t fourth_octet;
  big_uint8_buf_t fifth_octet;
  big_uint8_buf_t sixth_octet;

  [[nodiscard]] std::string to_string() const
  {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(first_octet.value())
       << ":" << std::setw(2) << static_cast<int>(second_octet.value()) << ":" << std::setw(2)
       << static_cast<int>(third_octet.value()) << ":" << std::setw(2)
       << static_cast<int>(fourth_octet.value()) << ":" << std::setw(2)
       << static_cast<int>(fifth_octet.value()) << ":" << std::setw(2)
       << static_cast<int>(sixth_octet.value());
    return ss.str();
  }
};

struct Ethernet
{
  IpAddress lidar_ip;
  IpAddress dest_pc_ip;
  MacAddress mac_addr;
  big_uint16_buf_t lidar_out_msop_port;
  big_uint16_buf_t pc_dest_msop_port;
  big_uint16_buf_t lidar_out_difop_port;
  big_uint16_buf_t pc_dest_difop_port;
};

struct FovSetting
{
  big_uint16_buf_t fov_start;
  big_uint16_buf_t fov_end;
};

constexpr uint8_t angle_sign_flag = 0x00;

struct ChannelAngleCorrection
{
  big_uint8_buf_t sign;
  big_uint16_buf_t angle;

  [[nodiscard]] float get_angle() const
  {
    return sign.value() == angle_sign_flag ? static_cast<float>(angle.value()) / 100.0f
                                           : static_cast<float>(angle.value()) / -100.0f;
  }
};

struct CorrectedVerticalAngle
{
  ChannelAngleCorrection angles[32];
};

struct CorrectedHorizontalAngle
{
  ChannelAngleCorrection angles[32];
};

struct SensorCalibration
{
  CorrectedVerticalAngle corrected_vertical_angle;
  CorrectedHorizontalAngle corrected_horizontal_angle;

  RobosenseCalibrationConfiguration get_calibration() const
  {
    RobosenseCalibrationConfiguration calibration;
    for (size_t i = 0; i < 32; ++i) {
      ChannelCorrection channel_correction;
      channel_correction.azimuth = corrected_horizontal_angle.angles[i].get_angle();
      channel_correction.elevation = corrected_vertical_angle.angles[i].get_angle();
      calibration.calibration.push_back(channel_correction);
    }
    return calibration;
  }
};

struct FirmwareVersion
{
  big_uint8_buf_t first_octet;
  big_uint8_buf_t second_octet;
  big_uint8_buf_t third_octet;
  big_uint8_buf_t fourth_octet;
  big_uint8_buf_t fifth_octet;

  [[nodiscard]] std::string to_string() const
  {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(first_octet.value())
       << std::setw(2) << static_cast<int>(second_octet.value()) << std::setw(2)
       << static_cast<int>(third_octet.value()) << std::setw(2)
       << static_cast<int>(fourth_octet.value()) << std::setw(2)
       << static_cast<int>(fifth_octet.value());
    return ss.str();
  }
};

struct SerialNumber
{
  big_uint8_buf_t first_octet;
  big_uint8_buf_t second_octet;
  big_uint8_buf_t third_octet;
  big_uint8_buf_t fourth_octet;
  big_uint8_buf_t fifth_octet;
  big_uint8_buf_t sixth_octet;

  [[nodiscard]] std::string to_string() const
  {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(first_octet.value())
       << std::setw(2) << static_cast<int>(second_octet.value()) << std::setw(2)
       << static_cast<int>(third_octet.value()) << std::setw(2)
       << static_cast<int>(fourth_octet.value()) << std::setw(2)
       << static_cast<int>(fifth_octet.value()) << std::setw(2)
       << static_cast<int>(sixth_octet.value());
    return ss.str();
  }
};

#pragma pack(pop)

/// @brief Get the number of returns for a given return mode
/// @param return_mode The return mode
/// @return The number of returns
inline size_t get_n_returns(ReturnMode return_mode)
{
  if (return_mode == ReturnMode::DUAL) {
    return 2;
  }
  return 1;
}

/// @brief Get timestamp from packet in nanoseconds
/// @tparam PacketT The packet type
/// @param packet The packet to get the timestamp from
/// @return The timestamp in nanoseconds
template <typename PacketT>
uint64_t get_timestamp_ns(const PacketT & packet)
{
  return packet.header.timestamp.get_time_in_ns();
}

/// @brief Get the distance unit of the given packet type in meters. Distance values in the packet,
/// multiplied by this value, yield the distance in meters.
/// @tparam PacketT The packet type
/// @param packet The packet to get the distance unit from
/// @return The distance unit in meters
template <typename PacketT>
double get_dis_unit(const PacketT & packet)
{
  // Packets define distance unit in millimeters, convert to meters here
  const uint8_t range_resolution = packet.header.range_resolution.value();
  if (range_resolution == 0) {
    return 0.0050;
  } else if (range_resolution == 1) {
    return 0.0025;
  }
  throw std::runtime_error("Unknown range resolution");
}

/// @brief Convert raw angle value from packet to std::string
/// @param raw_angle The raw angle value from the packet
/// @return The angle as std::string
inline std::string get_float_value(const uint16_t & raw_angle)
{
  return std::to_string(static_cast<float>(raw_angle) / 100.0f);
}

}  // namespace nebula::drivers::robosense_packet
