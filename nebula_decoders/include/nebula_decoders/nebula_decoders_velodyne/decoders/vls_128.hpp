#pragma once

#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_sensor.hpp"

namespace nebula
{
namespace drivers
{

class VLS128
{
public:
  int num_maintenance_periods = 1;

  int num_simultaneous_firings = 8;

  double firing_sequences_per_block = 0.25;

  int channels_per_firing_sequence = 128;

  float distance_resolution_m = 0.002f;

  char sensor_model[16] = "vls128";

  // unused
  float VLP16_BLOCK_DURATION = 110.592f;  // [µs]
  float VLP16_DSR_TOFFSET = 2.304f;       // [µs]
  float VLP16_FIRING_TOFFSET = 55.296f;   // [µs]
};
}  // namespace drivers
}  // namespace nebula