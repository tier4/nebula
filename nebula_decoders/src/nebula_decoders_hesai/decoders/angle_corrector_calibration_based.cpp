#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector_calibration_based.hpp"

namespace nebula
{
namespace drivers
{

template <size_t ChannelN, size_t AngleUnit>
AngleCorrectorCalibrationBased<ChannelN, AngleUnit>::AngleCorrectorCalibrationBased(
  const std::shared_ptr<HesaiCalibrationConfiguration> & sensor_calibration,
  const std::shared_ptr<HesaiCorrection> & sensor_correction)
: AngleCorrector<AngleUnit * 360>(sensor_calibration, sensor_correction)
{
  const auto & calibration = AngleCorrector<SENSOR_MAX_AZIMUTH_LENGTH>::sensor_calibration_;

  if (sensor_calibration == nullptr) {
    throw std::runtime_error(
      "Cannot instantiate AngleCorrectorCalibrationBased without calibration data");
  }

  for (size_t channel_id = 0; channel_id < ChannelN; ++channel_id) {
    float elevation_angle = calibration->elev_angle_map[channel_id];
    float azimuth_offset = calibration->azimuth_offset_map[channel_id];
    elevation_angle_[channel_id] =
      static_cast<int32_t>(elevation_angle * INTERNAL_RESOLUTION_PER_DEG);
    azimuth_offset_[channel_id] =
      static_cast<int32_t>(azimuth_offset * INTERNAL_RESOLUTION_PER_DEG);
    elevation_angle_rad_[channel_id] = deg2rad(elevation_angle);
    azimuth_offset_rad_[channel_id] = deg2rad(azimuth_offset);
  }

  for (uint32_t i = 0; i < AngleUnit * 360; i++) {
    block_azimuth_rad_[i] = deg2rad(i / AngleUnit);
  }
}

template <size_t ChannelN, size_t AngleUnit>
std::tuple<uint32_t, uint32_t, float, float>
AngleCorrectorCalibrationBased<ChannelN, AngleUnit>::getCorrectedAzimuthAndElevation(
  uint32_t block_azimuth, uint32_t channel_id)
{
  int32_t corrected_azimuth = block_azimuth / AngleUnit * INTERNAL_RESOLUTION_PER_DEG;
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

// Explicit template instantiation to prevent linker errors
template class AngleCorrectorCalibrationBased<32, 100>;
template class AngleCorrectorCalibrationBased<40, 100>;
template class AngleCorrectorCalibrationBased<64, 100>;
template class AngleCorrectorCalibrationBased<128, 100>;

}  // namespace drivers
}  // namespace nebula