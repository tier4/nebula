#pragma once
#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_sensor.hpp"

namespace nebula
{
namespace drivers
{

class VLS128 : public VelodyneSensor
{
public:
  // To ignore an empty data blocks in VLS128 case
  /// @brief VLS128 Dual return mode data structure in VLS128 User manual p.57
  int getNumPaddingBlocks(bool dual_return)
  {
    if (dual_return) return 4;
    return 0;
  }

// calculate and stack the firing timing for each laser timeing
/// @brief laser timing for VLS128 from VLS128 User manual in p.61
  bool fillAzimuthCache()
  {
    for (uint8_t i = 0; i < 16; i++) {
      laser_azimuth_cache_[i] = (VLS128_CHANNEL_DURATION / VLS128_SEQ_DURATION) * (i + i / 8);
    }
    return true;
  }

  /// @brief fomula from VLS128 User manual in p.65
  /// @param azimuth Azimuth angle
  /// @param azimuth_diff Azimuth difference
  /// @param firing_order Firing order
  /// @return Corrected azimuth
  uint16_t getAzimuthCorrected(
    uint16_t azimuth, float azimuth_diff, int /* firing_sequence */, int firing_order)
  {
    float azimuth_corrected = azimuth + (azimuth_diff * laser_azimuth_cache_[firing_order]);

    return static_cast<uint16_t>(round(azimuth_corrected)) % 36000;
  }

  uint16_t getTrueRotation(uint16_t azimuth_corrected, uint16_t current_block_rotation)
  {
    return azimuth_corrected;
  }

  constexpr static int num_maintenance_periods = 1;

  constexpr static int num_simultaneous_firings = 8;

  constexpr static double firing_sequences_per_block = 0.25;

  constexpr static int channels_per_firing_sequence = 128;

  constexpr static float distance_resolution_m = 0.004f;

  constexpr static double full_firing_cycle_s = 53.3 * 1e-6;

  constexpr static double single_firing_s = 2.665 * 1e-6;

  constexpr static double offset_packet_time = 8.7 * 1e-6;

private:
  float laser_azimuth_cache_[16];
};
}  // namespace drivers
}  // namespace nebula