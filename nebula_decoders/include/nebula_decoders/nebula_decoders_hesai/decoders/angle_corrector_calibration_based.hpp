#pragma once

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector.hpp"

#include <cstdint>

namespace nebula
{
namespace drivers
{

template <size_t ChannelN, size_t AngleUnit>
class AngleCorrectorCalibrationBased : public AngleCorrector<AngleUnit * 360>
{
private:
  static constexpr size_t INTERNAL_RESOLUTION_PER_DEG = 1000;
  static constexpr size_t INTERNAL_MAX_AZIMUTH_LENGTH = 360 * INTERNAL_RESOLUTION_PER_DEG;
  static constexpr size_t SENSOR_MAX_AZIMUTH_LENGTH = 360 * AngleUnit;

  std::array<int32_t, ChannelN> elevation_angle_{};
  std::array<int32_t, ChannelN> azimuth_offset_{};
  std::array<float, ChannelN> elevation_angle_rad_{};
  std::array<float, ChannelN> azimuth_offset_rad_{};
  std::array<float, 360 * AngleUnit> block_azimuth_rad_{};

public:
  AngleCorrectorCalibrationBased(
    const std::shared_ptr<HesaiCalibrationConfiguration> & sensor_calibration,
    const std::shared_ptr<HesaiCorrection> & sensor_correction);

  std::tuple<uint32_t, uint32_t, float, float> getCorrectedAzimuthAndElevation(
    uint32_t block_azimuth, uint32_t channel_id) override;
};

}  // namespace drivers
}  // namespace nebula