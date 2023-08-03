#pragma once

#include <array>

namespace nebula
{
namespace drivers
{

/// @brief Base class for all sensor definitions. Sensors have to `typedef` a `packet_t` type
class HesaiSensor
{
public:
  HesaiSensor() = default;
  virtual ~HesaiSensor() = default;

  /// @brief Calculate/lookup the time offset of the given channel to its block
  /// @param channel_id The channel to look up
  /// @return The time offset in nanoseconds
  virtual int getChannelTimeOffset(uint32_t channel_id) = 0;

  /// @brief Calculate/lookup the time offset of the given block to its packet
  /// @param block_id The block to look up
  /// @param n_returns The number of returns of the currently active return mode
  /// @return The time offset in nanoseconds
  virtual int getBlockTimeOffset(uint32_t block_id, uint32_t n_returns) = 0;
};

}  // namespace drivers
}  // namespace nebula