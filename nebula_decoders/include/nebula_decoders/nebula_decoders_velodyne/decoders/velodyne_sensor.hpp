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
  // To ignore an empty data blocks which is created by only VLS128 dual return mode case
  /// @brief each VLP lidars packat structure in user manual. If you know details, see commens in each <vlp_list>.hpp file.
  virtual int getNumPaddingBlocks(bool /* dual_return */) { return 0; }

  // calculate and stack the firing timing for each laser timeing used in getAzimuthCorrected to calculate the corrected azimuth
  /// @brief each VLP lidar laser timing in user manual. If you know details, see commens in each <vlp_list>.hpp file.
  virtual bool fillAzimuthCache() { return false; }

  // calculate the corrected azimuth from each firing timing.
  /// @brief each VLP calculating sample code and formula in user manual. If you know details, see commens in each <vlp_list>.hpp file.
  virtual uint16_t getAzimuthCorrected(
    uint16_t azimuth, float azimuth_diff, int firing_sequence, int firing_order) = 0;
};
}  // namespace drivers
}  // namespace nebula