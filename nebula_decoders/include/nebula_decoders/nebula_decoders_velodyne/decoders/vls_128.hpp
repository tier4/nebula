#pragma once

namespace nebula
{
namespace drivers
{

class VLS128
{
public:
  constexpr static int num_maintenance_periods = 1;

  constexpr static int num_simultaneous_firings = 8;

  constexpr static double firing_sequences_per_block = 0.25;

  constexpr static int channels_per_firing_sequence = 128;

  constexpr static float distance_resolution_m = 0.004f;

  constexpr static char sensor_model[16] = "vls128";
};
}  // namespace drivers
}  // namespace nebula