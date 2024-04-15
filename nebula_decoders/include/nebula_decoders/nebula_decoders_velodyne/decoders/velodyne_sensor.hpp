#pragma once  // why this

// NOTES:
// how does "compute timing offsets" part of decoder work?

namespace nebula
{
namespace drivers
{

class VelodyneSensor
{
public:
  double full_firing_cycle_s;
  double single_firing_s;
  int scans_per_packet;

  int blocks_per_packet;
  int scan_per_blocks;

  // int firings_per_block;

  int channels_per_firing_sequence;

  int num_channels;
  int num_simultaneous_firings;

  // only vls128, others should be 0
  double offset_packet_time;

  // only vlp16
  int firings_per_scan;

private:
};
}  // namespace drivers
}  // namespace nebula

// TODO: VLS128 only supports single return mode, yes? Should be error checking if dual_return is
// set