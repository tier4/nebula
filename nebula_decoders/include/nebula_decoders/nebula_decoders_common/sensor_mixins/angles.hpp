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

struct CorrectedAngleData
{
  float azimuth_rad;
  float elevation_rad;
  float sin_azimuth;
  float cos_azimuth;
  float sin_elevation;
  float cos_elevation;
};

template <typename PacketT>
struct AnglesMixin
{
  virtual int32_t getRawAzimuth(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const = 0;
  virtual int32_t getRawElevation(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const = 0;
};

template <typename PacketT>
struct AngleCorrectorMixin
{
  virtual CorrectedAngleData getCorrectedAngleData(int32_t raw_azimuth, int32_t raw_elevation) const = 0;
};

template <typename PacketT>
struct BlockAziChannelElevMixin : public AnglesMixin<PacketT>
{
  int32_t getRawAzimuth(
    const PacketT & packet, const size_t block_id, const size_t /* unit_id */) const override
  {
    const auto * block = getBlock(packet, block_id);
    return getFieldValue(block->azimuth);
  }

  int32_t getRawElevation(
    const PacketT & /* packet */, const size_t /* block_id */, const size_t unit_id) const override
  {
    return static_cast<int32_t>(unit_id);
  }
};

template <typename PacketT>
struct AnglesInUnitMixin : public AnglesMixin<PacketT>
{
  int32_t getRawAzimuth(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const override
  {
    const auto * unit = getUnit(packet, block_id, unit_id);
    return getFieldValue(unit->azimuth);
  }

  int32_t getRawElevation(
    const PacketT & packet, const size_t block_id, const size_t unit_id) const override
  {
    const auto * unit = getUnit(packet, block_id, unit_id);
    return getFieldValue(unit->elevation);
  }
};

}  // namespace sensor_mixins
}  // namespace drivers
}  // namespace nebula