#pragma once

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
struct ChannelMixin
{
  virtual uint8_t getChannel(
    const PacketT & packet, const size_t block_id, const size_t unit_id) = 0;
};

template <typename PacketT>
struct ChannelIsUnitMixin: public ChannelMixin<PacketT>
{
  /// @brief Returns the channel of the point identified by block_id and unit_id.
  /// A channel refers to a single laser source in the sensor.
  uint8_t getChannel(
    const PacketT & /* packet */, const size_t /* block_id */, const size_t unit_id) override
  {
    return unit_id;
  }
};

}  // namespace point_accessors
}  // namespace drivers
}  // namespace nebula