#pragma once

#include <cstdint>

namespace nebula
{
namespace drivers
{

class TimeCorrector
{
public:

  uint64_t getPointTimestamp(uint64_t scan_ns, uint64_t packet_ns, uint32_t block_id, uint32_t channel_id, uint32_t n_returns) const {
    return packet_ns - scan_ns 
  }
};

}  // namespace drivers
}  // namespace nebula