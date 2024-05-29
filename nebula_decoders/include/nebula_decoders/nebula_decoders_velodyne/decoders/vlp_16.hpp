#pragma once
#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_sensor.hpp"

namespace nebula
{
namespace drivers
{

class VLP16 : public VelodyneSensor
{
public:
  /// @brief formula from VLP16 User manual in p.64
  /// @param azimuth Azimuth angle
  /// @param azimuth_diff Azimuth difference
  /// @param firing_sequence Firing sequence
  /// @param firing_order Firing order
  /// @return Corrected azimuth
  uint16_t getAzimuthCorrected(
    uint16_t azimuth, float azimuth_diff, int firing_sequence, int firing_order)
  {
    float azimuth_corrected =
      azimuth + (azimuth_diff *
                 ((firing_order * VLP16_DSR_TOFFSET) + (firing_sequence * VLP16_FIRING_TOFFSET)) /
                 VLP16_BLOCK_DURATION);

    return static_cast<uint16_t>(round(azimuth_corrected)) % 36000;
  }

  // Choose the correct azimuth from the 2 azimuths
  uint16_t getTrueRotation(uint16_t azimuth_corrected, uint16_t current_block_rotation)
  {
    return azimuth_corrected;
  }

  constexpr static int num_maintenance_periods = 0;

  constexpr static int num_simultaneous_firings = 1;

  constexpr static double firing_sequences_per_block = 2.0;

  constexpr static int channels_per_firing_sequence = 16;

  constexpr static float distance_resolution_m = 0.002f;

  constexpr static double full_firing_cycle_s = 55.296 * 1e-6;

  constexpr static double single_firing_s = 2.304 * 1e-6;

  constexpr static double offset_packet_time = 0;

  /** Special Defines for VLP16 support **/
  constexpr static const int VLP16_FIRINGS_PER_BLOCK = 2;
  constexpr static const int VLP16_SCANS_PER_FIRING = 16;
  constexpr static const float VLP16_BLOCK_DURATION = 110.592f;  // [µs]
  constexpr static const float VLP16_DSR_TOFFSET = 2.304f;       // [µs]
  constexpr static const float VLP16_FIRING_TOFFSET = 55.296f;   // [µs]
};
}  // namespace drivers
}  // namespace nebula