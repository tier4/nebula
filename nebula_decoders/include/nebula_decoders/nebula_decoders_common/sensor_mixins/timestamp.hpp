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
};

/// @brief Return the timestamp field of each block as the timestamp of the block's units
template <typename PacketT>
struct BlockTimestampUsMixin: public PointTimestampMixin<PacketT>
{
  int32_t getPacketRelativeTimestamp(
    const PacketT & packet, const size_t block_id, const size_t /* channel_id */,
    const ReturnMode /* return_mode */) const override
  {
    const auto * block = getBlock(packet, block_id);
    return static_cast<int32_t>(getFieldValue(block->timestamp)) * 1000;
  }
};

}  // namespace sensor_mixins
}  // namespace drivers
}  // namespace nebula