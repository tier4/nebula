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

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <stdexcept>
namespace nebula::drivers::hesai_packet
{

// FIXME(mojomex) This is a workaround for the compiler being pedantic about casting `enum class`s
// to their underlying type
namespace return_mode
{
enum ReturnMode {
  SINGLE_FIRST = 0x33,
  SINGLE_SECOND = 0x34,
  SINGLE_STRONGEST = 0x37,
  SINGLE_LAST = 0x38,
  DUAL_LAST_STRONGEST = 0x39,
  DUAL_FIRST_SECOND = 0x3a,
  DUAL_FIRST_LAST = 0x3b,
  DUAL_FIRST_STRONGEST = 0x3c,
  TRIPLE_FIRST_LAST_STRONGEST = 0x3d,
  DUAL_STRONGEST_SECONDSTRONGEST = 0x3e,
};
}  // namespace return_mode

#pragma pack(push, 1)

/// @brief DateTime struct for Hesai packets
/// @tparam YearOffset like std::tm, the Hesai format has a year offset that is applied to the raw
/// year value. For most protocol versions it is 1900 (like std::tm), for some it is 2000.
template <int YearOffset>
struct DateTime
{
  /// @brief Year - YearOffset (e.g. for YearOffset=1900 value 100 corresponds to year 2000)
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  /// @brief Get seconds since epoch
  /// @return Whole seconds since epoch
  [[nodiscard]] uint64_t get_seconds() const
  {
    std::tm tm{};
    tm.tm_year = year - 1900 + YearOffset;
    tm.tm_mon = month - 1;  // starts from 0 in C
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = second;
    return timegm(&tm);
  }
};

struct SecondsSinceEpoch
{
  uint8_t zero;
  /// @brief Seconds since epoch, in big-endian format
  uint8_t seconds[5];

  /// @brief Get seconds since epoch
  /// @return Whole seconds since epoch
  [[nodiscard]] uint64_t get_seconds() const
  {
    uint64_t seconds = 0;
    for (unsigned char second : this->seconds) {
      seconds = (seconds << 8) | second;
    }
    return seconds;
  }
};

struct FunctionalSafety
{
  uint8_t fs_version;
  uint8_t lidar_state;
  uint8_t fault_code_id;
  uint16_t fault_code;
  uint8_t reserved1[8];
  uint32_t crc_fs;
};

struct Header12B
{
  uint16_t sop;
  uint8_t protocol_major;
  uint8_t protocol_minor;
  uint8_t reserved1[2];

  uint8_t laser_num;
  uint8_t block_num;
  uint8_t reserved2;
  uint8_t dis_unit;
  uint8_t return_num;
  uint8_t flags;
};

struct Header8B
{
  /// @brief Start of Packet, 0xEEFF
  uint16_t sop;

  uint8_t laser_num;
  uint8_t block_num;
  uint8_t reserved1;
  uint8_t dis_unit;
  uint8_t reserved2[2];
};

struct Unit3B
{
  uint16_t distance;
  uint8_t reflectivity;
};

struct Unit4B
{
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t confidence_or_reserved;
};

template <typename UnitT, size_t UnitN>
struct Block
{
  uint16_t azimuth;
  UnitT units[UnitN];
  using unit_t = UnitT;

  [[nodiscard]] uint32_t get_azimuth() const { return azimuth; }
};

template <typename UnitT, size_t UnitN>
struct FineAzimuthBlock
{
  using unit_t = UnitT;
  uint16_t azimuth;
  uint8_t fine_azimuth;
  UnitT units[UnitN];

  [[nodiscard]] uint32_t get_azimuth() const { return (azimuth << 8) + fine_azimuth; }
};

template <typename UnitT, size_t UnitN>
struct SOBBlock
{
  using unit_t = UnitT;

  /// @brief Start of Block, 0xFFEE
  uint16_t sob;
  uint16_t azimuth;
  UnitT units[UnitN];

  [[nodiscard]] uint32_t get_azimuth() const { return azimuth; }
};

template <typename BlockT, size_t BlockN>
struct Body
{
  using block_t = BlockT;
  BlockT blocks[BlockN];
};

/// @brief Base struct for all Hesai packets. This struct is not allowed to have any non-static
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

#pragma pack(pop)

/// @brief Get the number of returns for a given return mode
/// @param return_mode The return mode
/// @return The number of returns
inline int get_n_returns(uint8_t return_mode)
{
  switch (return_mode) {
    case return_mode::SINGLE_FIRST:
    case return_mode::SINGLE_SECOND:
    case return_mode::SINGLE_STRONGEST:
    case return_mode::SINGLE_LAST:
      return 1;
    case return_mode::DUAL_LAST_STRONGEST:
    case return_mode::DUAL_FIRST_SECOND:
    case return_mode::DUAL_FIRST_LAST:
    case return_mode::DUAL_FIRST_STRONGEST:
    case return_mode::DUAL_STRONGEST_SECONDSTRONGEST:
      return 2;
    case return_mode::TRIPLE_FIRST_LAST_STRONGEST:
      return 3;
    default:
      throw std::runtime_error("Unknown return mode");
  }
}

/// @brief Get timestamp from packet in nanoseconds
/// @tparam PacketT The packet type
/// @param packet The packet to get the timestamp from
/// @return The timestamp in nanoseconds
template <typename PacketT>
uint64_t get_timestamp_ns(const PacketT & packet)
{
  return packet.tail.date_time.get_seconds() * 1000000000 + packet.tail.timestamp * 1000;
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
  return packet.header.dis_unit / 1000.;
}

}  // namespace nebula::drivers::hesai_packet
