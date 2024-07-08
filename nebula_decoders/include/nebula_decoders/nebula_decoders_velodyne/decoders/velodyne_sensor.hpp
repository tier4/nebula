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
  /// @brief each VLP lidars packat structure in user manual. If you know details, see commens in each <vlp_list>.hpp file.
  /// To ignore an empty data blocks which is created by only VLS128 dual return mode case
  virtual int getNumPaddingBlocks(bool /* dual_return */) { return 0; }

  /// @brief each VLP lidar laser timing in user manual. If you know details, see commens in each <vlp_list>.hpp file.
  /// calculate and stack the firing timing for each laser timeing used in getAzimuthCorrected to calculate the corrected azimuth
  virtual bool fillAzimuthCache() { return false; }

  /// @brief VSL128User manual p. Packet structure
  virtual uint getBank(uint bank, uint /* header */) { return bank; }

  /// @brief each VLP calculating sample code and formula in user manual. If you know details, see commens in each <vlp_list>.hpp file.
  /// calculate the corrected azimuth from each firing timing.
  virtual uint16_t getAzimuthCorrected(
    uint16_t azimuth, float azimuth_diff, int firing_sequence, int firing_order) = 0;

  /// @brief each VLP calculating sample code and formula in user manual. Check packet structure.
  /// Get a correct firing order
  virtual int getFiringOrder(int /* channels_per_block */, int /* scans_per_firing */) { return 0; }

  /// @brief each VLP calculating sample code and formula in user manual. Check packet structure.
  /// Get a correct channel number
  virtual int getChannelNumber(int /* unit_idx */) { return 0; }

};
}  // namespace drivers
}  // namespace nebula