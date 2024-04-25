#pragma once

#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_sensor.hpp"

namespace nebula
{
namespace drivers
{
class VLP32 : public VelodyneSensor
{
public:
  bool fillAzimuthCache()
  {
    for (uint8_t i = 0; i < 16; i++) {
      laser_azimuth_cache_[i] = (VLP32_CHANNEL_DURATION / VLP32_SEQ_DURATION) * (i + i / 8);
    }
    return true;
  }

  // TODO: implement this function
  uint16_t getAzimuthCorrected(
    uint16_t azimuth, float azimuth_diff, int firing_sequence, int firing_order)
  {
    float azimuth_corrected = azimuth + (azimuth_diff * laser_azimuth_cache_[firing_order]);

    return static_cast<uint16_t>(round(azimuth_corrected)) % 36000;
  }

  constexpr static int num_maintenance_periods = 0;

  constexpr static int num_simultaneous_firings = 2;

  constexpr static double firing_sequences_per_block = 1.0;

  constexpr static int channels_per_firing_sequence = 32;

  constexpr static float distance_resolution_m = 0.004f;  // TODO: double check this

private:
  float laser_azimuth_cache_[16];
};
}  // namespace drivers
}  // namespace nebula