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

/// @brief Retrieves the packet's distance unit and calculates unit distances in meters
template <typename PacketT>
struct DistanceMixin
{
  /// @brief Returns the value to multiply distance values in the packet with to get meters.
  virtual double getDistanceUnit(const PacketT & packet) const = 0;

  /// @brief Returns the distance value of the given unit in meters.
  virtual double getDistance(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const = 0;
};

/// @brief Reads the distance unit from the packet header and distance from each unit's `distance`
/// field.
template <typename PacketT>
struct HeaderMmDisUnitMixin : public DistanceMixin<PacketT>
{
  double getDistanceUnit(const PacketT & packet) const override
  {
    return packet.header.dis_unit / 1000.;
  }

  double getDistance(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const override
  {
    const auto * unit = getUnit(packet, block_id, unit_id);
    return getFieldValue(unit->distance) * getDistanceUnit(packet);
  }
};

}  // namespace sensor_mixins
}  // namespace drivers
}  // namespace nebula