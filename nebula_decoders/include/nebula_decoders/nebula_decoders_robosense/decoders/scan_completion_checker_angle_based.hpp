#pragma once

#include "nebula_decoders/nebula_decoders_robosense/decoders/scan_completion_checker.hpp"
namespace nebula
{
namespace drivers
{
template <typename PacketT>
class ScanCompletionCheckerAngleBased : public ScanCompletionChecker<PacketT>
{
public:
  bool isScanComplete(const PacketT & packet, const size_t block_id)
  {
    uint32_t current_azimuth =
      (360 * PacketT::DEGREE_SUBDIVISIONS + packet.body.blocks[block_id].get_azimuth() -
       static_cast<int>(sensor_configuration_->scan_phase * PacketT::DEGREE_SUBDIVISIONS)) %
      (360 * PacketT::DEGREE_SUBDIVISIONS);

    bool scan_complete = current_azimuth < last_azimuth_;

    last_azimuth_ = current_azimuth;
    return scan_complete;
  }

private:
  uint32_t last_azimuth_{};
};
}  // namespace drivers
}  // namespace nebula
