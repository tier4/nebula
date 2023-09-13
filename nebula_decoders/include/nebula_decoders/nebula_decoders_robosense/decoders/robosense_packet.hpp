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

}  // namespace robosense_packet
}  // namespace drivers
}  // namespace nebula