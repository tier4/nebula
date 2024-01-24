#pragma once

#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/channel.hpp"

#include <cstddef>
#include <cstdint>
#include <sstream>

using namespace nebula::drivers::sensor_mixins;

namespace nebula
{
namespace drivers
{

template <typename PacketT>
struct BpearlChannelMixin : public ChannelMixin<PacketT>
{
  BpearlChannelMixin(const std::shared_ptr<const RobosenseCalibrationConfiguration> & calibration)
  {
    if (calibration == nullptr) {
      throw std::runtime_error("Cannot instantiate BpearlChannelMixin without calibration data");
    }

    if (calibration->calibration.size() != 32) {
      std::stringstream ss;
      ss << "Calibration has wrong number of channels: expected 32, got "
         << calibration->calibration.size();
      throw std::runtime_error(ss.str());
    }

    for (uint8_t channel = 0; channel < 32; ++channel) {
      const auto & correction = calibration->GetCorrection(channel);
      uint8_t remap_to_channel = 0;
      for (const auto & other_correction : calibration->calibration) {
        if (other_correction.elevation < correction.elevation) ++remap_to_channel;
      }
      channel_map_[channel] = remap_to_channel;
    }
  }

  uint8_t getChannel(
    const PacketT & /* packet */, const size_t /* block_id */, const size_t unit_id) const override
  {
    return channel_map_[unit_id];
  }

private:
  std::array<uint8_t, 32> channel_map_;
};

}  // namespace drivers
}  // namespace nebula