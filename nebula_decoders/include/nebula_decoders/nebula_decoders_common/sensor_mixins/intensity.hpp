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
struct IntensityMixin
{
  virtual uint8_t getIntensity(
    const PacketT & packet, const size_t block_id, const size_t unit_id) = 0;
};

template <typename PacketT>
struct BasicReflectivityMixin : public IntensityMixin<PacketT>
{
  /// @brief Returns the intensity of the point identified by block_id and channel_id.
  /// Intensities from 0-100 refer to diffuse reflections whereas intensities from 101-255
  /// refer to retroreflections.
  uint8_t getIntensity(
    const PacketT & packet, const size_t block_id, const size_t unit_id) override
  {
    const auto * unit = getUnit(packet, block_id, unit_id);
    return static_cast<uint8_t>(getFieldValue(unit->reflectivity));
  }
};

}  // namespace point_accessors
}  // namespace drivers
}  // namespace nebula