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
namespace point_accessors
{

template <typename SensorT>
struct ReturnModeMixin {
  virtual ReturnMode getReturnMode(
    const SensorT::packet_t & packet, const SensorConfigurationBase & config) = 0;

};

template <typename SensorT>
struct ReturnModeFromConfigMixin: public ReturnModeMixin<SensorT>
{
  /// @brief Retrieves the return mode from the given sensor configuration
  ReturnMode getReturnMode(const PacketT & /* packet */, const SensorConfigurationBase & config) override
  {
    return config.return_mode;
  }
};

}  // namespace point_accessors
}  // namespace drivers
}  // namespace nebula