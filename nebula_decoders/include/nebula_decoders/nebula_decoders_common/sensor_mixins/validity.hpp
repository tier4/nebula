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

/// @brief Briefly checks if a given unit is valid and should be decoded.
/// This check is designed to be fast and should not perform any expensive calculations.
template <typename PacketT>
struct ValidityMixin
{
  /// @brief Checks if the given unit can/should be decoded (e.g. if not all fields are 0)
  virtual bool isValid(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const = 0;
};

/// @brief Reports units as valid if their distance field is non-zero.
template <typename PacketT>
struct NonZeroDistanceIsValidMixin : public ValidityMixin<PacketT>
{
  bool isValid(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const override
  {
    const auto * unit = getUnit(packet, block_id, unit_id);
    return getFieldValue(unit->distance) != 0;
  }
};

}  // namespace sensor_mixins
}  // namespace drivers
}  // namespace nebula