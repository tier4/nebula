#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <stdexcept>

namespace nebula
{
namespace drivers
{
namespace robosense_packet
{

struct Unit
{
  uint16_t distance;
  uint8_t reflectivity;
};

template <typename UnitT, size_t UnitN>
struct Block
{
  uint16_t flag;
  uint16_t azimuth;
  UnitT units[UnitN];
  typedef UnitT unit_t;

  uint32_t get_azimuth() const { return azimuth; }
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
  static constexpr size_t N_BLOCKS = nBlocks;
  static constexpr size_t N_CHANNELS = nChannels;
  static constexpr size_t MAX_RETURNS = maxReturns;
  static constexpr size_t DEGREE_SUBDIVISIONS = degreeSubdivisions;
};

/// @brief Get timestamp from packet in nanoseconds
/// @tparam PacketT The packet type
/// @param packet The packet to get the timestamp from
/// @return The timestamp in nanoseconds
template <typename PacketT>
uint64_t get_timestamp_ns(const PacketT & packet)
{
  return packet.header.timestamp.get_time_in_ns();
  //  return packet.tail.date_time.get_seconds() * 1000000000 + packet.tail.timestamp * 1000;
}

/// @brief Get the distance unit of the given packet type in meters. Distance values in the packet, multiplied by this value, yield the distance in meters.
/// @tparam PacketT The packet type
/// @param packet The packet to get the distance unit from
/// @return The distance unit in meters
template <typename PacketT>
double get_dis_unit(const PacketT & packet)
{
  // Packets define distance unit in millimeters, convert to meters here
  return 0.0025;
//  return packet.header.dis_unit / 1000.;
}

}  // namespace robosense_packet
}  // namespace drivers
}  // namespace nebula