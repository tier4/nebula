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

template <typename PacketT>
struct DistanceMixin
{
  virtual double getDistanceUnit(const PacketT & packet) const = 0;
  virtual double getDistance(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const = 0;
};

template <typename PacketT>
struct HeaderMmDisUnitMixin: public DistanceMixin<PacketT>
{
  /// @brief Get the distance unit of the given packet type in meters.
  /// Distance values in the packet, multiplied by this value, yield the distance in meters.
  double getDistanceUnit(const PacketT & packet) const override
  {
    return packet.header.dis_unit / 1000.;
  }

  /// @brief Get the distance value of the given unit in meters.
  double getDistance(const PacketT & packet, const size_t block_id, const size_t unit_id) const override
  {
    const auto * unit = getUnit(packet, block_id, unit_id);
    return getFieldValue(unit->distance) * getDistanceUnit(packet);
  }
};

}  // namespace sensor_mixins
}  // namespace drivers
}  // namespace nebula