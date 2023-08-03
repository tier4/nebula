#pragma once

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector.hpp"

#include <cstdint>

namespace nebula
{
namespace drivers
{

template <size_t ChannelN, size_t AngleUnit>
class AngleCorrectorCalibrationBased : public AngleCorrector<1000 * 360>
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
    const std::shared_ptr<HesaiCorrection> & sensor_correction)
  : AngleCorrector<1000 * 360>(sensor_calibration, sensor_correction)
  {
    if (sensor_calibration == nullptr) {
      throw std::runtime_error(
        "Cannot instantiate AngleCorrectorCalibrationBased without calibration data");
    }

    for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
      float elevation_angle = sensor_calibration->elev_angle_map[channel_id];
      float azimuth_offset = sensor_calibration->azimuth_offset_map[channel_id];
      elevation_angle_[channel_id] =
        static_cast<int32_t>(elevation_angle * INTERNAL_RESOLUTION_PER_DEG);
      azimuth_offset_[channel_id] =
        static_cast<int32_t>(azimuth_offset * INTERNAL_RESOLUTION_PER_DEG);
      elevation_angle_rad_[channel_id] = deg2rad(elevation_angle);
      azimuth_offset_rad_[channel_id] = deg2rad(azimuth_offset);
    }

    for (uint32_t i = 0; i < AngleUnit * 360; i++) {
      block_azimuth_rad_[i] = deg2rad(i / static_cast<float>(AngleUnit));
    }
  }

  std::tuple<uint32_t, uint32_t, float, float> getCorrectedAzimuthAndElevation(
    uint32_t block_azimuth, uint32_t channel_id) override
  {
    int32_t corrected_azimuth = block_azimuth * INTERNAL_RESOLUTION_PER_DEG / AngleUnit;
    corrected_azimuth += azimuth_offset_[channel_id];
    if (corrected_azimuth < 0) {
      corrected_azimuth += INTERNAL_MAX_AZIMUTH_LENGTH;
    } else if (corrected_azimuth >= INTERNAL_MAX_AZIMUTH_LENGTH) {
      corrected_azimuth -= INTERNAL_MAX_AZIMUTH_LENGTH;
    }

    int32_t corrected_elevation = elevation_angle_[channel_id];
    if (corrected_elevation < 0) {
      corrected_elevation += INTERNAL_MAX_AZIMUTH_LENGTH;
    } else if (corrected_elevation >= INTERNAL_MAX_AZIMUTH_LENGTH) {
      corrected_elevation -= INTERNAL_MAX_AZIMUTH_LENGTH;
    }

    return std::make_tuple(
      static_cast<uint32_t>(corrected_azimuth), static_cast<uint32_t>(corrected_elevation),
      block_azimuth_rad_[block_azimuth] + azimuth_offset_rad_[channel_id],
      elevation_angle_rad_[channel_id]);
  }

  bool hasScanned(int current_azimuth, int last_azimuth) override
  {
    return current_azimuth < last_azimuth;
  }
};

}  // namespace drivers
}  // namespace nebula