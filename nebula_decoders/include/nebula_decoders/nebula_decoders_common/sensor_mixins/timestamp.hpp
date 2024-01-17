#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_decoders/nebula_decoders_common/util.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace nebula
{
namespace drivers
{
namespace sensor_mixins
{

/// @brief Retrieves a packet's timestamp
template <typename PacketT>
struct PacketTimestampMixin
{
  /// @brief Returns the timestamp of the packet in nanoseconds
  virtual uint64_t getPacketTimestamp(const PacketT & packet) const = 0;
};

/// @brief Retrieves a point's timestamp relative to the packet's timestamp
template <typename PacketT>
struct PointTimestampMixin
{
  /// @brief Returns the packet-relative timestamp of the given unit, in nanoseconds
  virtual int32_t getPacketRelativeTimestamp(
    const PacketT & packet, const size_t block_id, const size_t channel_id,
    const ReturnMode return_mode) const = 0;

  /// @brief For a given start block index, find the earliest (lowest) relative time offset of any
  /// point in the packet in or after the start block, in nanoseconds
  virtual int32_t getEarliestPointTimeOffsetForScan(
    const PacketT & packet, const size_t block_id, const ReturnMode return_mode) const = 0;
};

/// @brief Determine the start timestamp of the next scan based on the lowest point timestamp in the
/// next return group. This mixin checks all point timestamps in the return group starting at
/// `block_id` and returns the lowest one.
template <typename PacketT>
struct BlockUnitIdBasedPointTimestampMixin : public PointTimestampMixin<PacketT>
{
  virtual int32_t getPacketRelativeTimestamp(
    const PacketT & packet, const size_t block_id, const size_t channel_id,
    const ReturnMode return_mode) const = 0;

  int32_t getEarliestPointTimeOffsetForScan(
    const PacketT & packet, const size_t block_id, const ReturnMode return_mode) const override
  {
    int32_t t_min = std::numeric_limits<int>::max();
    auto n_returns = ReturnModeToNReturns(return_mode);
    for (size_t blk = block_id; blk < block_id + n_returns; ++blk) {
      for (size_t ch = 0; ch < PacketT::N_CHANNELS; ++ch) {
        t_min = std::min(t_min, getPacketRelativeTimestamp(packet, blk, ch, return_mode));
      }
    }

    return t_min;
  }
};

}  // namespace sensor_mixins
}  // namespace drivers
}  // namespace nebula