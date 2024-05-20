#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>

namespace nebula
{
namespace drivers
{
namespace seyond_packet
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
  DUAL_STRONGEST_SECONDSTRONGEST = 0x3,
};
}  // namespace return_mode

#pragma pack(push, 1)

/// @brief DateTime struct for Seyond packets
/// @tparam YearOffset like std::tm, the Seyond format has a year offset that is applied to the raw
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
  uint64_t get_seconds() const
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
  uint64_t get_seconds() const
  {
    uint64_t seconds = 0;
    for (int i = 0; i < 5; ++i) {
      seconds = (seconds << 8) | this->seconds[i];
    }
    return seconds;
  }
};

template <typename BlockT, size_t BlockN>
struct Body
{
  typedef BlockT block_t;
  BlockT blocks[BlockN];
};

/// @brief Base struct for all Seyond packets. This struct is not allowed to have any non-static
/// members, otherwise memory layout is not guaranteed for the derived structs.
/// @tparam nBlocks The number of blocks in the packet
/// @tparam nChannels The number of channels per block
/// @tparam maxReturns The maximum number of returns, e.g. 2 for dual return
/// @tparam degreeSubdivisions The resolution of the azimuth angle in the packet, e.g. 100 if packet
/// azimuth is given in 1/100th of a degree
template <size_t nBlocks, size_t nChannels, size_t maxReturns, size_t degreeSubdivisions>
struct PacketBase
{
  static constexpr size_t N_BLOCKS = nBlocks;
  static constexpr size_t N_CHANNELS = nChannels;
  static constexpr size_t MAX_RETURNS = maxReturns;
  static constexpr size_t DEGREE_SUBDIVISIONS = degreeSubdivisions;
};

}  // namespace seyond_packet
}  // namespace drivers
}  // namespace nebula
