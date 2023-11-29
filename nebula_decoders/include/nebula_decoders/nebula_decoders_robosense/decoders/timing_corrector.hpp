#pragma once

namespace nebula
{
namespace drivers
{
template <typename SensorT>
class TimingCorrector
{
public:
  /// @brief Computes the exact relative time between the timestamp of the given packet and the one
  /// of the point identified by the given block and channel, in nanoseconds
  /// @param block_id The point's block id within the packet
  /// @param channel_id The point's channel id
  /// @param packet The packet
  /// @return The relative time offset in nanoseconds (allowed to be negative)
  virtual int64_t getPacketRelativePointTimeOffset(
    const typename SensorT::packet_t & packet, const size_t block_id, const size_t channel_id) = 0;
};
}  // namespace drivers
}  // namespace nebula
