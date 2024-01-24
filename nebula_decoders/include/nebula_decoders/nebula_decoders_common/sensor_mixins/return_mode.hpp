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

/// @brief Retrieves the current return mode of the sensor
template <typename PacketT>
struct ReturnModeMixin
{
  /// @brief Retrieves the current return mode from a given packet and/or sensor configuration
  virtual ReturnMode getReturnMode(
    const PacketT & packet, const SensorConfigurationBase & config) const = 0;
};

/// @brief Returns the return mode from the sensor configuration
template <typename PacketT>
struct ReturnModeFromConfigMixin : public ReturnModeMixin<PacketT>
{
  ReturnMode getReturnMode(
    const PacketT & /* packet */, const SensorConfigurationBase & config) const override
  {
    return config.return_mode;
  }
};

}  // namespace sensor_mixins
}  // namespace drivers
}  // namespace nebula