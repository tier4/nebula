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
namespace point_accessors
{

template <typename PacketT>
struct PacketTimestampMixin
{
  virtual uint64_t getPacketTimestamp(const PacketT & packet) = 0;
};

template <typename PacketT>
struct PointTimestampMixin
{
  virtual int32_t getPacketRelativeTimestamp(
    const PacketT & packet, const size_t block_id, const size_t channel_id,
    const ReturnMode return_mode) = 0;
};

template <typename PacketT>
struct BlockTimestampUsMixin: public PointTimestampMixin<PacketT>
{
  /// @brief Returns the timestamp of the point identified by block_id and channel_id relative to
  /// the packet's timestamp, in nanoseconds
  int32_t getPacketRelativeTimestamp(
    const PacketT & packet, const size_t block_id, const size_t /* channel_id */,
    const ReturnMode /* return_mode */) override
  {
    const auto * block = getBlock(packet, block_id);
    return static_cast<int32_t>(getFieldValue(block->timestamp)) * 1000;
  }
};

}  // namespace point_accessors
}  // namespace drivers
}  // namespace nebula