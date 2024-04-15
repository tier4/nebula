#pragma once

#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_sensor.hpp"

namespace nebula
{
namespace drivers
{

class VLS128
{
public:
  constexpr static int raw_channel_size = 3;

  constexpr static int blocks_per_packet = 12;

  constexpr static int channels_per_block = 32;

  constexpr static int num_maintenance_periods = 1;

  constexpr static int num_simultaneous_firings = 8;

  constexpr static double firing_sequences_per_block = 0.25;

  constexpr static int channels_per_firing_sequence = 128;

  constexpr static float distance_resolution_m = 0.004f;

  constexpr static char sensor_model[16] = "vls128";

  constexpr static int BANK_1 = 0xeeff;
  constexpr static int BANK_2 = 0xddff;
  constexpr static int BANK_3 = 0xccff;
  constexpr static int BANK_4 = 0xbbff;

  // unused
  constexpr static float VLP16_BLOCK_DURATION = 110.592f;  // [µs]
  constexpr static float VLP16_DSR_TOFFSET = 2.304f;       // [µs]
  constexpr static float VLP16_FIRING_TOFFSET = 55.296f;   // [µs]
};
}  // namespace drivers
}  // namespace nebula