#pragma once
#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_sensor.hpp"

namespace nebula
{
namespace drivers
{

class VLP16 : public VelodyneSensor
{
public:
  uint16_t getAzimuthCorrected(
    uint16_t azimuth, float azimuth_diff, int firing_sequence, int firing_order)
  {
    float azimuth_corrected =
      azimuth + (azimuth_diff *
                 ((firing_order * VLP16_DSR_TOFFSET) + (firing_sequence * VLP16_FIRING_TOFFSET)) /
                 VLP16_BLOCK_DURATION);

    return static_cast<uint16_t>(round(azimuth_corrected)) % 36000;
  }

  constexpr static int num_maintenance_periods = 0;

  constexpr static int num_simultaneous_firings = 1;

  constexpr static double firing_sequences_per_block = 2.0;

  constexpr static int channels_per_firing_sequence = 16;

  constexpr static float distance_resolution_m = 0.002f;

  constexpr static double full_firing_cycle_s = 55.296 * 1e-6;

  constexpr static double single_firing_s = 2.304 * 1e-6;

  constexpr static double offset_packet_time = 0;
};
}  // namespace drivers
}  // namespace nebula