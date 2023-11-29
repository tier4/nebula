#pragma once
#include "robosense_msgs/msg/robosense_packet.hpp"

#include <vector>

namespace nebula
{
namespace drivers
{

template <typename SensorT>
class PacketParser
{
public:
  static constexpr size_t max_packets_per_decode_group =
    std::max(SensorT::RETURN_GROUP_STRIDE[0] * SensorT::packet_t::MAX_RETURNS, 1);
  typedef std::vector<typename SensorT::packet_t, max_packets_per_decode_group> decode_group_t;

  /// @brief Parse and validate an MSOP packet. Currently only checks size, not checksums etc.
  /// @param msop_packet The incoming MsopPacket
  /// @return Whether the packet was parsed successfully and is ready for decoding
  virtual bool parsePacket(
    const robosense_msgs::msg::RobosensePacket & in_msop_packet,
    decode_group_t & out_decode_group) = 0;
};
}  // namespace drivers
}  // namespace nebula
