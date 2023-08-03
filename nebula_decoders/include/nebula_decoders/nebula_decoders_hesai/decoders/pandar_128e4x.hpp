#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_sensor.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_128e3x.hpp"

namespace nebula
{
namespace drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

typedef Packet128E3X Packet128E4X;

#pragma pack(pop)

}  // namespace hesai_packet

// FIXME(mojomex) support high resolution mode
class Pandar128E4X : public HesaiSensor
{
private:
  static constexpr int firing_time_offset_ns_[128] = {
    49758, 43224, 36690, 30156, 21980, 15446, 8912,  2378,  49758, 43224, 36690, 30156, 2378,
    15446, 8912,  21980, 43224, 30156, 49758, 15446, 36690, 2378,  21980, 8912,  34312, 45002,
    38468, 40846, 40846, 34312, 51536, 47380, 31934, 47380, 31934, 51536, 38468, 27778, 27778,
    45002, 38468, 40846, 51536, 45002, 31934, 47380, 34312, 27778, 38468, 40846, 51536, 45002,
    31934, 34312, 27778, 38468, 40846, 51536, 45002, 31934, 47380, 34312, 27778, 45002, 45002,
    51536, 38468, 40846, 47380, 40846, 31934, 45002, 27778, 38468, 40846, 31934, 27778, 38468,
    34312, 34312, 34312, 47380, 51536, 31934, 51536, 47380, 27778, 49758, 21980, 43224, 15446,
    43224, 36690, 8912,  30156, 2378,  49758, 21980, 36690, 15446, 8912,  30156, 2378,  49758,
    43224, 36690, 30156, 21980, 15446, 8912,  2378,  43224, 49758, 30156, 36690, 21980, 15446,
    2378,  8912,  49758, 43224, 36690, 30156, 21980, 15446, 8912,  2378,  30156};

public:
  typedef hesai_packet::Packet128E4X packet_t;

  int getChannelTimeOffset(uint32_t channel_id) override
  {
    return 43346 + firing_time_offset_ns_[channel_id];
  }

  int getBlockTimeOffset(uint32_t block_id, uint32_t n_returns) override
  {
    if (n_returns == 1) {
      return -27778 * 2 * (2 - block_id - 1);
    }

    return 0;
  }
};

}  // namespace drivers
}  // namespace nebula