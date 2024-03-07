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

class Pandar128E4X : public HesaiSensor<hesai_packet::Packet128E4X>
{
private:
enum OperationalState { HIGH_RESOLUTION = 0, SHUTDOWN = 1, STANDARD = 2 };

  static constexpr int hires_as0_time_offset_ns_[128] = {
    -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185,
    14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  0,     18867,
    12578, 18867, 0,     6289,  16549, 10260, 16549, 10260, 12578, 3971,  3971,  6289,  22838,
    22838, 16549, 18867, 0,     10260, 12578, 18867, 3971,  6289,  12578, 22838, 16549, 6289,
    0,     10260, 16549, 18867, 3971,  10260, 12578, 22838, 3971,  6289,  0,     22838, 16549,
    18867, 0,     10260, 12578, 18867, 3971,  6289,  12578, 22838, 16549, 6289,  0,     10260,
    16549, 18867, 3971,  10260, 12578, 22838, 3971,  6289,  0,     22838, -1,    -1,    -1,
    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,
    -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185,
    14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318};

  static constexpr int hires_as1_time_offset_ns_[128] = {
    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,
    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    0,     18867,
    12578, 18867, 0,     6289,  16549, 10260, 16549, 10260, 12578, 3971,  3971,  6289,  22838,
    22838, 16549, 18867, 0,     10260, 12578, 18867, 3971,  6289,  12578, 22838, 16549, 6289,
    0,     10260, 16549, 18867, 3971,  10260, 12578, 22838, 3971,  6289,  0,     22838, 16549,
    18867, 0,     10260, 12578, 18867, 3971,  6289,  12578, 22838, 16549, 6289,  0,     10260,
    16549, 18867, 3971,  10260, 12578, 22838, 3971,  6289,  0,     22838, 21185, 14896, 8607,
    2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,
    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,
    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1};

  static constexpr int hires_as2_time_offset_ns_[128] = {
    -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185,
    14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  0,     18867,
    12578, 18867, 0,     6289,  16549, 10260, 16549, 10260, 12578, 3971,  3971,  6289,  22838,
    22838, 16549, 18867, 0,     10260, 12578, 18867, 3971,  6289,  12578, 22838, 16549, 6289,
    0,     10260, 16549, 18867, 3971,  10260, 12578, 22838, 3971,  6289,  0,     22838, 16549,
    18867, 0,     10260, 12578, 18867, 3971,  6289,  12578, 22838, 16549, 6289,  0,     10260,
    16549, 18867, 3971,  10260, 12578, 22838, 3971,  6289,  0,     22838, -1,    -1,    -1,
    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,
    -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185,
    14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318};

  static constexpr int hires_as3_time_offset_ns_[128] = {
    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,
    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    0,     18867,
    12578, 18867, 0,     6289,  16549, 10260, 16549, 10260, 12578, 3971,  3971,  6289,  22838,
    22838, 16549, 18867, 0,     10260, 12578, 18867, 3971,  6289,  12578, 22838, 16549, 6289,
    0,     10260, 16549, 18867, 3971,  10260, 12578, 22838, 3971,  6289,  0,     22838, 16549,
    18867, 0,     10260, 12578, 18867, 3971,  6289,  12578, 22838, 16549, 6289,  0,     10260,
    16549, 18867, 3971,  10260, 12578, 22838, 3971,  6289,  0,     22838, 21185, 14896, 8607,
    2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,
    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,
    -1,    -1,    -1,    21185, 14896, 8607,  2318,  -1,    -1,    -1,    -1};

  static constexpr int standard_time_offset_ns_[128] = {
    48963, 42674, 36385, 30096, 21185, 14896, 8607,  2318,  48963, 42674, 36385, 30096, 21185,
    14896, 8607,  2318,  48963, 42674, 36385, 30096, 21185, 14896, 8607,  2318,  27778, 46645,
    40356, 46645, 27778, 34067, 44327, 38038, 44327, 38038, 40356, 31749, 31749, 34067, 50616,
    50616, 44327, 46645, 27778, 38038, 40356, 46645, 31749, 34067, 40356, 50616, 44327, 34067,
    27778, 38038, 44327, 46645, 31749, 38038, 40356, 50616, 31749, 34067, 27778, 50616, 44327,
    46645, 27778, 38038, 40356, 46645, 31749, 34067, 40356, 50616, 44327, 34067, 27778, 38038,
    44327, 46645, 31749, 38038, 40356, 50616, 31749, 34067, 27778, 50616, 48963, 42674, 36385,
    30096, 21185, 14896, 8607,  2318,  48963, 42674, 36385, 30096, 21185, 14896, 8607,  2318,
    48963, 42674, 36385, 30096, 21185, 14896, 8607,  2318,  48963, 42674, 36385, 30096, 21185,
    14896, 8607,  2318,  48963, 42674, 36385, 30096, 21185, 14896, 8607,  2318};

public:
  static constexpr float MIN_RANGE = 0.1;
  static constexpr float MAX_RANGE = 230.0;
  static constexpr size_t MAX_SCAN_BUFFER_POINTS = 691200;

  int getPacketRelativePointTimeOffset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet)
  {
    auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);
    int block_offset_ns = -27778 * (2 - block_id - 1) / n_returns;

    int channel_offset_ns;
    bool is_hires_mode = packet.tail.operational_state == OperationalState::HIGH_RESOLUTION;
    if (!is_hires_mode) block_offset_ns *= 2;

    auto azimuth_state = packet.tail.geAzimuthState(block_id);

    if (is_hires_mode && azimuth_state == 0)
      channel_offset_ns = hires_as0_time_offset_ns_[channel_id];
    else if (is_hires_mode && azimuth_state == 1)
      channel_offset_ns = hires_as1_time_offset_ns_[channel_id];
    else if (is_hires_mode && azimuth_state == 2)
      channel_offset_ns = hires_as2_time_offset_ns_[channel_id];
    else if (is_hires_mode && azimuth_state == 3)
      channel_offset_ns = hires_as3_time_offset_ns_[channel_id];
    else if (!is_hires_mode)
      channel_offset_ns = standard_time_offset_ns_[channel_id];
    else
      throw std::runtime_error("Invalid combination of operational state and azimuth state");

    return block_offset_ns + channel_offset_ns;
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