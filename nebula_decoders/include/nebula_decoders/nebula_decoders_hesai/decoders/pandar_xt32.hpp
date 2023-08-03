#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_sensor.hpp"

namespace nebula
{
namespace drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

struct TailXT32
{
  uint8_t reserved1[10];
  uint8_t return_mode;
  uint16_t motor_speed;
  DateTime<1900> date_time;
  uint32_t timestamp;
  uint8_t factory_information;
};

struct PacketXT32 : public PacketBase<8, 32, 2, 100>
{
  typedef Body<Block<Unit4B, PacketXT32::N_CHANNELS>, PacketXT32::N_BLOCKS> body_t;
  typedef AngleCorrectorCalibrationBased<PacketXT32::N_CHANNELS, PacketXT32::DEGREE_SUBDIVISIONS>
    angle_decoder_t;
  Header12B header;
  body_t body;
  TailXT32 tail;
  uint32_t udp_sequence;
};

#pragma pack(pop)

}  // namespace hesai_packet

class PandarXT32 : public HesaiSensor
{
public:
  typedef hesai_packet::PacketXT32 packet_t;

  int getChannelTimeOffset(uint32_t channel_id) override
  {
    return 368 + 1512 * channel_id;
  }

  int getBlockTimeOffset(uint32_t block_id, uint32_t n_returns) override
  {
    if (n_returns == 1) {
      return 5632 - 50000 * (8 - block_id - 1);
    }

    // The integer division is intentional here, cf. datasheet
    return 5632 - 50000 * ((8 - block_id - 1) / 2);
  }
};

}  // namespace drivers
}  // namespace nebula