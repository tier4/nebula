#pragma once

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

/// @brief Retrieves the channel ID (laser ID) of a given unit
template <typename PacketT>
struct ChannelMixin
{
  /// @brief Retrieves the channel ID (laser ID) of a given unit
  virtual uint8_t getChannel(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const = 0;
};

/// @brief Returns the given unit ID as channel ID
template <typename PacketT>
struct ChannelIsUnitMixin: public ChannelMixin<PacketT>
{
  uint8_t getChannel(
    const PacketT & /* packet */, const size_t /* block_id */, const size_t unit_id) const override
  {
    return unit_id;
  }
};

}  // namespace sensor_mixins
}  // namespace drivers
}  // namespace nebula