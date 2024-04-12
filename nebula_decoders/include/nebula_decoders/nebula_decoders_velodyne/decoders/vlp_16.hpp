#pragma once

#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_sensor.hpp"

namespace nebula
{
namespace drivers
{

class VLP16
{
public:
  int blocks_per_packet = 12;

  int channels_per_block = 32;

  int num_maintenance_periods = 0;

  int num_simultaneous_firings = 1;

  double firing_sequences_per_block = 2.0;

  int channels_per_firing_sequence = 16;

  float distance_resolution_m = 0.002f;

  char sensor_model[16] = "vlp16";

  float VLP16_BLOCK_DURATION = 110.592f;  // [µs]
  float VLP16_DSR_TOFFSET = 2.304f;       // [µs]
  float VLP16_FIRING_TOFFSET = 55.296f;   // [µs]
};
}  // namespace drivers
}  // namespace nebula