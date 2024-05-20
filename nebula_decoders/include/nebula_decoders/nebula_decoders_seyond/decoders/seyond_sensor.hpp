#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_decoders/nebula_decoders_seyond/decoders/seyond_packet.hpp"

#include <type_traits>

namespace nebula
{
namespace drivers
{

/// @brief Base class for all sensor definitions
/// @tparam PacketT The packet type of the sensor
template <typename PacketT>
class SeyondSensor
{
private:

public:
  typedef PacketT packet_t;

  SeyondSensor() = default;
  virtual ~SeyondSensor() = default;
};

}  // namespace drivers
}  // namespace nebula