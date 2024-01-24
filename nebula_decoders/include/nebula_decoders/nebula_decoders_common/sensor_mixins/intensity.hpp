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

/// @brief Retrieves and, if needed, converts the intensity of a given unit to the range [0, 255].
/// Intensities from 0-100 refer to diffuse reflections whereas intensities from 101-255
/// refer to retroreflections.
template <typename PacketT>
struct IntensityMixin
{
  /// @brief Returns the intensity of the given unit.
  virtual uint8_t getIntensity(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const = 0;
};

/// @brief Reads the intensity from a unit's `reflectivity` field and does no correction.
template <typename PacketT>
struct BasicReflectivityMixin : public IntensityMixin<PacketT>
{
  uint8_t getIntensity(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const override
  {
    const auto * unit = getUnit(packet, block_id, unit_id);
    return static_cast<uint8_t>(getFieldValue(unit->reflectivity));
  }
};

}  // namespace sensor_mixins
}  // namespace drivers
}  // namespace nebula