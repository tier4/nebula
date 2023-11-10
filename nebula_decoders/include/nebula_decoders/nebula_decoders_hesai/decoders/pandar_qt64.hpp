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

struct TailQT64
{
  uint8_t reserved1[10];
  uint16_t motor_speed;
  uint32_t timestamp;
  uint8_t return_mode;
  uint8_t factory_information;
  DateTime<1900> date_time;
};

struct PacketQT64 : public PacketBase<4, 64, 2, 100>
{
  typedef Body<Block<Unit4B, PacketQT64::N_CHANNELS>, PacketQT64::N_BLOCKS> body_t;
  Header12B header;
  body_t body;
  TailQT64 tail;
  uint32_t udp_sequence;
};

#pragma pack(pop)

}  // namespace hesai_packet

class PandarQT64 : public HesaiSensor<hesai_packet::PacketQT64>
{
private:
  static constexpr int firing_time_offset_ns_[64] = {
    12310,  14370,  16430,  18490,  20540,  22600,  24660,  26710,  29160,  31220,  33280,
    35340,  37390,  39450,  41500,  43560,  46610,  48670,  50730,  52780,  54840,  56900,
    58950,  61010,  63450,  65520,  67580,  69630,  71690,  73740,  75800,  77860,  80900,
    82970,  85020,  87080,  89140,  91190,  93250,  95300,  97750,  99820,  101870, 103930,
    105980, 108040, 110100, 112150, 115200, 117260, 119320, 121380, 123430, 125490, 127540,
    12960,  132050, 134110, 136170, 138220, 140280, 142340, 144390, 146450};

public:
  static constexpr float MIN_RANGE = 0.1f;
  static constexpr float MAX_RANGE = 60.f;
  static constexpr size_t MAX_SCAN_BUFFER_POINTS = 76800;

  int getPacketRelativePointTimeOffset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns = 25710 + (500000 * (block_id / n_returns)) / 3;
    return block_offset_ns + firing_time_offset_ns_[channel_id];
  }
};

}  // namespace drivers
}  // namespace nebula