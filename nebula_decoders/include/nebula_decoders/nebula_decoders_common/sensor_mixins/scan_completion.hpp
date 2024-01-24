#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_decoders/nebula_decoders_common/util.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace nebula
{
namespace drivers
{
namespace sensor_mixins
{

/// @brief Checks whether a given block is the start of a new scan.
template <typename PacketT>
struct ScanCompletionMixin
{
  /// @brief Returns whether the given block is the start of a new scan.
  virtual bool checkScanCompleted(const PacketT & packet, const size_t block_id) = 0;
};

/// @brief Check scan completion based on the azimuth of the current block.
/// This mixin also reacts to `scan_phase` configuration, scans will be cut at the given
/// `scan_phase` angle.
template <typename PacketT, typename ConfigT>
struct AngleBasedScanCompletionMixin : public ScanCompletionMixin<PacketT>
{
  AngleBasedScanCompletionMixin(const std::shared_ptr<const ConfigT> & config)
  : sensor_configuration_(config)
  {
  }

  bool checkScanCompleted(const PacketT & packet, const size_t block_id) override
  {
    const auto * block = getBlock(packet, block_id);
    uint32_t current_azimuth =
      (360 * PacketT::DEGREE_SUBDIVISIONS + block->get_azimuth() -
       static_cast<int>(sensor_configuration_->scan_phase * PacketT::DEGREE_SUBDIVISIONS)) %
      (360 * PacketT::DEGREE_SUBDIVISIONS);

    bool scan_complete = current_azimuth < last_azimuth_;

    last_azimuth_ = current_azimuth;
    return scan_complete;
  }

private:
  uint32_t last_azimuth_{};
  const std::shared_ptr<const ConfigT> sensor_configuration_;
};

}  // namespace sensor_mixins
}  // namespace drivers
}  // namespace nebula