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

template <typename PacketT>
struct ReturnModeMixin
{
  virtual ReturnMode getReturnMode(
    const PacketT & packet, const SensorConfigurationBase & config) const = 0;
};

template <typename PacketT>
struct ReturnModeFromConfigMixin : public ReturnModeMixin<PacketT>
{
  /// @brief Retrieves the return mode from the given sensor configuration
  ReturnMode getReturnMode(
    const PacketT & /* packet */, const SensorConfigurationBase & config) const override
  {
    return config.return_mode;
  }
};

}  // namespace sensor_mixins
}  // namespace drivers
}  // namespace nebula