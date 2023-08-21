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
class Pandar128E4X : public HesaiSensor<hesai_packet::Packet128E4X>
{
private:
  enum OperationalState { HIGH_RESOLUTION = 0, STANDARD = 1 };

  static constexpr int firing_time_offset_static_ns_[128] = {
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

  static constexpr int firing_time_offset_as0_ns_[128] = {
    -1,    -1,    -1,    -1,    21980, 15446, 8912,  2378,  -1,    -1,    -1,    -1,    2378,
    15446, 8912,  21980, -1,    2378,  21980, 8912,  6534,  17224, 10690, 13068, 13068, 6534,
    23758, 19602, 4156,  19602, 4156,  23758, 13068, 13068, 23758, 10690, 4156,  19602, 19602,
    0,     10690, 6534,  23758, 17224, 23758, 19602, 6534,  17224, 4156,  0,     6534,  0,
    17224, 10690, 17224, 17224, 23758, 23758, 10690, 13068, 13068, 13068, 19602, 19602, 6534,
    13068, 4156,  4156,  17224, 17224, 17224, 0,     10690, 10690, 13068, 13068, 4156,  0,
    10690, 6534,  6534,  6534,  19602, 23758, 4156,  23758, 19602, 0,     -1,    21980, -1,
    15446, -1,    8912,  -1,    2378,  -1,    -1,    21980, -1,    15446, 8912,  -1,    2378,
    -1,    -1,    -1,    -1,    21980, 15446, 8912,  2378,  -1,    -1,    -1,    -1,    21980,
    15446, 2378,  8912,  -1,    -1,    -1,    -1,    21980, 15446, 8912,  2378};

  static constexpr int firing_time_offset_as1_ns_[128] = {
    21980, 15446, 8912,  2378,  -1,    -1,    -1,    -1,    21980, 15446, 8912,  2378,  -1,
    -1,    -1,    -1,    8912,  -1,    -1,    -1,    6534,  17224, 10690, 13068, 13068, 6534,
    23758, 19602, 4156,  19602, 4156,  23758, 13068, 13068, 23758, 10690, 4156,  19602, 19602,
    0,     10690, 6534,  23758, 17224, 23758, 19602, 6534,  17224, 4156,  0,     6534,  0,
    17224, 10690, 17224, 17224, 23758, 23758, 10690, 13068, 13068, 13068, 19602, 19602, 6534,
    13068, 4156,  4156,  17224, 17224, 17224, 0,     10690, 10690, 13068, 13068, 4156,  0,
    10690, 6534,  6534,  6534,  19602, 23758, 4156,  23758, 19602, 0,     21980, -1,    15446,
    -1,    8912,  -1,    2378,  -1,    21980, 15446, -1,    8912,  -1,    -1,    2378,  -1,
    21980, 15446, 8912,  2378,  -1,    -1,    -1,    -1,    15446, 21980, 2378,  8912,  -1,
    -1,    -1,    -1,    21980, 15446, 8912,  2378,  -1,    -1,    -1,    -1};

public:
  static constexpr float MIN_RANGE = 0.1;
  static constexpr float MAX_RANGE = 230.0;
  static constexpr size_t MAX_SCAN_BUFFER_POINTS = 691200;

  int getPacketRelativePointTimeOffset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet)
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns;
    if (n_returns == 1) {
      block_offset_ns = -27778 * 2 * (2 - block_id - 1);
    } else {
      block_offset_ns = 0;
    }

    int channel_offset_ns;
    bool is_hires_mode = packet.tail.operational_state == OperationalState::HIGH_RESOLUTION;
    auto azimuth_state = packet.tail.geAzimuthState(block_id);

    if (!is_hires_mode) {
      channel_offset_ns = firing_time_offset_static_ns_[channel_id];
    } else {
      if (azimuth_state == 0) {
        channel_offset_ns = firing_time_offset_as0_ns_[channel_id];
      } else /* azimuth_state == 1 */ {
        channel_offset_ns = firing_time_offset_as1_ns_[channel_id];
      }
    }

    return block_offset_ns + 43346 + channel_offset_ns;
  }

  ReturnType getReturnType(
    hesai_packet::return_mode::ReturnMode return_mode, unsigned int return_idx,
    const std::vector<const typename packet_t::body_t::block_t::unit_t *> & return_units) override
  {
    auto return_type = HesaiSensor<packet_t>::getReturnType(return_mode, return_idx, return_units);
    if (return_type == ReturnType::IDENTICAL) {
      return return_type;
    }

    // This sensor orders returns in the opposite order, so the return_type needs to be flipped too
    if (return_mode == hesai_packet::return_mode::DUAL_FIRST_LAST) {
      if (return_type == ReturnType::FIRST)
        return_type = ReturnType::LAST;
      else if (return_type == ReturnType::LAST)
        return_type = ReturnType::FIRST;
    }

    return return_type;
  }
};

}  // namespace drivers
}  // namespace nebula