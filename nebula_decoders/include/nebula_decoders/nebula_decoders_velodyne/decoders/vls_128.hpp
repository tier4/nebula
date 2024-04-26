#pragma once
#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_sensor.hpp"

namespace nebula
{
namespace drivers
{

class VLS128 : public VelodyneSensor
{
public:
  int getNumPaddingBlocks(bool dual_return)
  {
    if (dual_return) return 4;
    return 0;
  }

  bool fillAzimuthCache()
  {
    for (uint8_t i = 0; i < 16; i++) {
      laser_azimuth_cache_[i] = (VLS128_CHANNEL_DURATION / VLS128_SEQ_DURATION) * (i + i / 8);
    }
    return true;
  }

  uint16_t getAzimuthCorrected(
    uint16_t azimuth, float azimuth_diff, int firing_sequence, int firing_order)
  {
    float azimuth_corrected = azimuth + (azimuth_diff * laser_azimuth_cache_[firing_order]);

    return static_cast<uint16_t>(round(azimuth_corrected)) % 36000;
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