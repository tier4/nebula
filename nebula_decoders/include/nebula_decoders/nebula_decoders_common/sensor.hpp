#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_decoders/nebula_decoders_common/util.hpp"

#include <cstddef>
#include <cstdint>

namespace nebula
{
namespace drivers
{

template <typename PacketT>
class SensorBase
{
private:
public:
  typedef PacketT packet_t;
  typedef typename packet_t::body_t body_t;
  typedef typename body_t::block_t block_t;
  typedef typename block_t::unit_t unit_t;

  SensorBase() = default;
  virtual ~SensorBase() = default;

  /// @brief Decide whereto to decode the given raw packet. This is only non-zero for sensors that
  /// split return groups across multiple packets
  /// @param raw_packet The raw bytes to be decoded
  /// @return The index [0, num_decode_groups) within the decode group to which the packet belongs
  virtual size_t getDecodeGroupIndex(const uint8_t * const /* raw_packet */) const { return 0; }

  /// @brief For a given start block index, find the earliest (lowest) relative time offset of any
  /// point in the packet in or after the start block
  /// @param start_block_id The index of the block in and after which to consider points
  /// @param sensor_configuration The sensor configuration
  /// @return The lowest point time offset (relative to the packet timestamp) of any point in or
  /// after the start block, in nanoseconds
  int getEarliestPointTimeOffsetForBlock(
    uint32_t /* start_block_id */, const std::shared_ptr<const SensorConfigurationBase> & /* sensor_configuration */)
  {
    return 0;  // TODO(mojomex): implement
  }

  /// @brief Whether the unit given by return_idx is a duplicate of any other unit in return_units
  /// @param return_idx The unit's index in the return_units vector
  /// @param return_units The vector of all the units corresponding to the same return group (i.e.
  /// length 2 for dual-return with both units having the same channel but coming from different
  /// blocks)
  /// @return true if the unit is identical to any other one in return_units, false otherwise
  template <typename CollectionT>
  static bool isDuplicate(const CollectionT & return_units, const size_t return_idx)
  {
    for (size_t i = 0; i < return_units.size(); ++i) {
      if (i == return_idx) {
        continue;
      }

      if (
        return_units[return_idx]->distance.value() == return_units[i]->distance.value() &&
        return_units[return_idx]->reflectivity.value() == return_units[i]->reflectivity.value()) {
        return true;
      }
    }

    return false;
  }

  /// @brief Get the return type of the point given by return_idx
  ///
  /// @param return_units The units corresponding to all the returns in the group. These are usually
  /// from the same column across adjascent blocks.
  /// @param return_idx The block index of the point within the group of blocks that make up the
  /// return group (e.g. either 0 or 1 for dual return)
  /// @param return_mode The sensor's currently active return mode
  /// @return The return type of the point
  template <typename CollectionT>
  static ReturnType getReturnType(
    const CollectionT & return_units, const size_t return_idx, const ReturnMode & return_mode)
  {
    if (isDuplicate(return_units, return_idx)) {
      return ReturnType::IDENTICAL;
    }

    switch (return_mode) {
      case ReturnMode::SINGLE_FIRST:
        return ReturnType::FIRST;
      case ReturnMode::SINGLE_LAST:
        return ReturnType::LAST;
      case ReturnMode::SINGLE_STRONGEST:
        return ReturnType::STRONGEST;
      case ReturnMode::DUAL:
        if (return_idx == 0) {
          return ReturnType::STRONGEST;
        } else {
          return ReturnType::LAST;
        }
      default:
        return ReturnType::UNKNOWN;
    }
  }
};

}  // namespace drivers
}  // namespace nebula