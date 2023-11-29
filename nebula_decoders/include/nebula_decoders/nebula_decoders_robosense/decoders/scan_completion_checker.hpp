#pragma once

namespace nebula
{
namespace drivers
{
template <typename PacketT>
class ScanCompletionChecker
{
public:
  /// @brief Given a packet and a block index within it, returns whether the given block is the
  /// beginning of a new scan
  /// @param packet The packet
  /// @param block_id The block in question
  /// @return Whether the given block is in a different scan than the last processed block, i.e.
  /// whether the last scan is complete and a new one has started
  virtual bool isScanComplete(const PacketT & packet, const size_t block_id) = 0;
};
}  // namespace drivers
}  // namespace nebula
