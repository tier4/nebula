#pragma once

namespace nebula
{
namespace drivers
{

class VLP16
{
public:
  constexpr static int num_maintenance_periods = 0;

  constexpr static int num_simultaneous_firings = 1;

  constexpr static double firing_sequences_per_block = 2.0;

  constexpr static int channels_per_firing_sequence = 16;

  constexpr static float distance_resolution_m = 0.002f;

  constexpr static char sensor_model[16] = "vlp16";
};
}  // namespace drivers
}  // namespace nebula