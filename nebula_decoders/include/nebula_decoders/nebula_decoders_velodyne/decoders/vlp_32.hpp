#pragma once

#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_sensor.hpp"

namespace nebula
{
namespace drivers
{
class VLP32 : public VelodyneSensor
{
public:
// calculate and stack the firing timing for each laser timeing
/// @brief laser timing for VLP32 from VLP32 User manual in p.61
  bool fillAzimuthCache()
  {
    for (uint8_t i = 0; i < 16; i++) {
      laser_azimuth_cache_[i] = (VLP32_CHANNEL_DURATION / VLP32_SEQ_DURATION) * (i + i / 2);
    }
    return true;
  }

  /// @brief formula from VLP32 User manual in p.62
  /// @param azimuth Azimuth angle
  /// @param azimuth_diff Azimuth difference between a current azimuth and a next azimuth
  /// @param firing_order Firing order
  /// @return Corrected azimuth
  uint16_t getAzimuthCorrected(
    uint16_t azimuth, float azimuth_diff, int /* firing_sequence */, int firing_order)
  {
    float azimuth_corrected = azimuth + (azimuth_diff * laser_azimuth_cache_[firing_order]);

    return static_cast<uint16_t>(round(azimuth_corrected)) % 36000;
  }

  // Choose the correct azimuth from the 2 azimuths
  uint16_t getTrueRotation(uint16_t azimuth_corrected, uint16_t current_block_rotation)
  {
    return current_block_rotation;
  }

  constexpr static int num_maintenance_periods = 0;

  constexpr static int num_simultaneous_firings = 2;

  constexpr static double firing_sequences_per_block = 1.0;

  constexpr static int channels_per_firing_sequence = 32;

  constexpr static float distance_resolution_m = 0.004f;

  constexpr static double full_firing_cycle_s = 55.296 * 1e-6;

  constexpr static double single_firing_s = 2.304 * 1e-6;

  constexpr static double offset_packet_time = 0;

private:
  float laser_azimuth_cache_[16];
};
}  // namespace drivers
}  // namespace nebula