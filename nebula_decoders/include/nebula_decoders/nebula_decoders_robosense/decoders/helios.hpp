#pragma once

#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_sensor.hpp"

#include <cstddef>
#include <cstdint>

namespace nebula
{
namespace drivers
{
namespace robosense_packet
{

struct Header
{
  uint32_t header_id;
  uint16_t protocol_version;
  uint16_t reserved_first;
  uint32_t top_packet_count;
  uint32_t bottom_packet_count;
  uint8_t reserved_second;
  uint8_t range_resolution;
  uint16_t angle_interval_count;
  uint8_t timestamp[10];
  uint8_t reserved_third;
  uint8_t lidar_type;
  uint8_t lidar_model;
  uint8_t reserved_fourth[9];
};

struct PacketHelios : public PacketBase<12, 32, 2, 100>
{
  typedef Body<Block<Unit, PacketHelios::N_CHANNELS>, PacketHelios::N_BLOCKS> body_t;
  Header header;
  body_t body;
  uint8_t tail[6];
  uint32_t udp_sequence;
};

#pragma pack(pop)

}  // namespace robosense_packet

class Helios : public RobosenseSensor<robosense_packet::PacketHelios>
{
private:
  static constexpr int firing_time_offset_ns_[384] = {
    -23180, -21876, -20572, -19268, -17964, -16660, -11444, -46796, -7532,  -36956, -50732,
    -54668, -40892, -44828, -31052, -34988, -48764, -52700, -38924, -42860, -29084, -33020,
    -46796, -25148, -36956, -50732, -27116, -40892, -44828, -31052, -34988, -48764, -25148,
    -38924, -42860, -29084, -33020, -52700, -6228,  -54668, -15356, -27116, -10140, -23180,
    -4924,  -21876, -14052, -17964, -8836,  -19268, -3620,  -20572, -12748, -16660, -7532,
    -11444, -6228,  -15356, -10140, -4924,  -3620,  -14052, -8836,  -12748};

public:
  static constexpr float MIN_RANGE = 0.2f;
  static constexpr float MAX_RANGE = 150.f;
  static constexpr size_t MAX_SCAN_BUFFER_POINTS = 230400; /////

  int getPacketRelativePointTimeOffset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns = -42580 - 55560 * ((6 - block_id - 1) / n_returns);
    return block_offset_ns + firing_time_offset_ns_[channel_id];
  }
};

}  // namespace drivers
}  // namespace nebula