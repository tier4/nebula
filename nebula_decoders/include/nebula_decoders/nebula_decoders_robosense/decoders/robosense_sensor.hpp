#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/angle_corrector_calibration_based.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"

#include <cstdint>
#include <type_traits>

namespace nebula
{
namespace drivers
{

/// @brief Base class for all sensor definitions
/// @tparam PacketT The packet type of the sensor
template <typename PacketT>
class RobosenseSensor
{
private:
public:
  typedef PacketT packet_t;
  typedef class AngleCorrectorCalibrationBased<PacketT::N_CHANNELS, PacketT::DEGREE_SUBDIVISIONS>
    angle_corrector_t;

  RobosenseSensor() = default;
  virtual ~RobosenseSensor() = default;

  /// @brief Computes the exact relative time between the timestamp of the given packet and the one
  /// of the point identified by the given block and channel, in nanoseconds
  /// @param block_id The point's block id
  /// @param channel_id The point's channel id
  /// @param packet The packet
  /// @return The relative time offset in nanoseconds
  virtual int getPacketRelativePointTimeOffset(
    uint32_t block_id, uint32_t channel_id,
    const std::shared_ptr<RobosenseSensorConfiguration> & sensor_configuration) = 0;

  /// @brief For a given start block index, find the earliest (lowest) relative time offset of any
  /// point in the packet in or after the start block
  /// @param start_block_id The index of the block in and after which to consider points
  /// @param packet The packet
  /// @return The lowest point time offset (relative to the packet timestamp) of any point in or
  /// after the start block, in nanoseconds
  int getEarliestPointTimeOffsetForBlock(
    uint32_t start_block_id,
    const std::shared_ptr<RobosenseSensorConfiguration> & sensor_configuration)
  {
    //    unsigned int n_returns = robosense_packet::get_n_returns(packet.tail.return_mode);
    unsigned int n_returns = 1;
    int min_offset_ns = 0xFFFFFFFF;  // MAXINT

    for (uint32_t block_id = start_block_id; block_id < start_block_id + n_returns; ++block_id) {
      for (uint32_t channel_id = 0; channel_id < PacketT::N_CHANNELS; ++channel_id) {
        min_offset_ns = std::min(
          min_offset_ns,
          getPacketRelativePointTimeOffset(block_id, channel_id, sensor_configuration));
      }
    }

    return min_offset_ns;
  }
};

}  // namespace drivers
}  // namespace nebula