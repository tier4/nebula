#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_sensor.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xt32.hpp"

namespace nebula
{
namespace drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

typedef TailXT32 TailXT32M2X;
struct PacketXT32M2X : public PacketBase<6, 32, 3, 100>
{
  typedef Body<Block<Unit4B, PacketXT32M2X::N_CHANNELS>, PacketXT32M2X::N_BLOCKS> body_t;
  typedef AngleCorrectorCalibrationBased<
    PacketXT32M2X::N_CHANNELS, PacketXT32M2X::DEGREE_SUBDIVISIONS>
    angle_decoder_t;
  Header12B header;
  body_t body;
  TailXT32M2X tail;
  uint32_t udp_sequence;
};

#pragma pack(pop)

}  // namespace hesai_packet

class PandarXT32M : public HesaiSensor
{
public:
  typedef hesai_packet::PacketXT32M2X packet_t;

  int getChannelTimeOffset(uint32_t channel_id) override
  {
    if (channel_id >= 16) {
      channel_id -= 16;
    }
    return 368 + 2888 * channel_id;
  }

  int getBlockTimeOffset(uint32_t block_id, uint32_t n_returns) override
  {
    if (n_returns == 1) {
      return 5632 - 50000 * (8 - block_id - 1);
    }

    if (n_returns == 2) {
      return 5632 - 50000 * ((8 - block_id - 1) / 2);
    }

    // n_returns == 3
    return 5632 - 50000 * ((6 - block_id - 1) / 3);
  }
};

}  // namespace drivers
}  // namespace nebula