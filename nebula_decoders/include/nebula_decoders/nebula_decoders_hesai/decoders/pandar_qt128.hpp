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

struct TailQT128C2X
{
  uint8_t reserved1[5];
  uint8_t mode_flag;
  uint8_t reserved2[6];
  uint8_t return_mode;
  uint16_t motor_speed;
  DateTime<1900> date_time;
  uint32_t timestamp;
  uint8_t factory_information;
  
  /* Ignored optional fields */

  //uint32_t udp_sequence;
  //uint32_t crc_tail;
};

struct PacketQT128C2X : public PacketBase<2, 128, 2, 100>
{
  typedef Body<Block<Unit4B, PacketQT128C2X::N_CHANNELS>, PacketQT128C2X::N_BLOCKS> body_t;
  typedef AngleCorrectorCalibrationBased<
    PacketQT128C2X::N_CHANNELS, PacketQT128C2X::DEGREE_SUBDIVISIONS>
    angle_decoder_t;
  Header12B header;
  body_t body;
  uint32_t crc_body;
  FunctionalSafety fs;
  TailQT128C2X tail;

  /* Ignored optional fields */
  
  //uint8_t cyber_security[32];
};

#pragma pack(pop)

}  // namespace hesai_packet

class PandarQT128 : public HesaiSensor
{
public:
  typedef hesai_packet::PacketQT128C2X packet_t;

  int getChannelTimeOffset(uint32_t channel_id) override
  {
    return 0; // FIXME(mojomex) implement firetime correction file & mode flag support 
  }

  int getBlockTimeOffset(uint32_t block_id, uint32_t n_returns) override
  {
    if (n_returns == 1) {
      return 9000 + 111110 * (2 - block_id - 1);
    }

    return 9000;
  }
};

}  // namespace drivers
}  // namespace nebula