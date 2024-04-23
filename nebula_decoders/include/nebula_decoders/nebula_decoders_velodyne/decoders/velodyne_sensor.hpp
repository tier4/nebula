#pragma once
#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_scan_decoder.hpp"

#include <cstdint>

namespace nebula
{
namespace drivers
{
class VelodyneSensor
{
public:
  virtual int getNumPaddingBlocks(bool dual_return) { return 0; }

  virtual bool fillAzimuthCache() { return false; }

  virtual uint16_t getAzimuthCorrected(
    uint16_t azimuth, float azimuth_diff, int firing_sequence, int firing_order) = 0;
};
}  // namespace drivers
}  // namespace nebula