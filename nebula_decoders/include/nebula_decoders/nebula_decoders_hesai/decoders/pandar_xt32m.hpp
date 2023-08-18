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
  Header12B header;
  body_t body;
  TailXT32M2X tail;
  uint32_t udp_sequence;
};

#pragma pack(pop)

}  // namespace hesai_packet

class PandarXT32M : public HesaiSensor<hesai_packet::PacketXT32M2X>
{
public:
  static constexpr float MIN_RANGE = 0.5f;
  static constexpr float MAX_RANGE = 300.0f;
  static constexpr size_t MAX_SCAN_BUFFER_POINTS = 384000;

  int getPacketRelativePointTimeOffset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns;
    if (n_returns < 3) {
      block_offset_ns = 5632 - 50000 * ((8 - block_id - 1) / n_returns);
    } else /* n_returns == 3 */ {
      block_offset_ns = 5632 - 50000 * ((6 - block_id - 1) / 3);
    }

    if (channel_id >= 16) {
      channel_id -= 16;
    }
    int channel_offset_ns = 368 + 2888 * channel_id;

    return block_offset_ns + channel_offset_ns;
  }
};

}  // namespace drivers
}  // namespace nebula