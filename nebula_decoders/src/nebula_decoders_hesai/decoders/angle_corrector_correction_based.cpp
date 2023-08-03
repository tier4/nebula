#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector_correction_based.hpp"

namespace nebula
{
namespace drivers
{

template <size_t ChannelN, size_t AngleUnit>
AngleCorrectorCorrectionBased<ChannelN, AngleUnit>::AngleCorrectorCorrectionBased(
  const std::shared_ptr<HesaiCalibrationConfiguration> & sensor_calibration,
  const std::shared_ptr<HesaiCorrection> & sensor_correction)
: AngleCorrector<AngleUnit * 360>(sensor_calibration, sensor_correction),
  logger_(rclcpp::get_logger("AngleCorrectorCorrectionBased"))
{
  if (sensor_correction == nullptr) {
    throw std::runtime_error(
      "Cannot instantiate AngleCorrectorCalibrationBased without calibration data");
  }

  logger_.set_level(rclcpp::Logger::Level::Debug);
}

template <size_t ChannelN, size_t AngleUnit>
std::tuple<uint32_t, uint32_t, float, float>
AngleCorrectorCorrectionBased<ChannelN, AngleUnit>::getCorrectedAzimuthAndElevation(
  uint32_t block_azimuth, uint32_t channel_id)
{
  const auto & correction = AngleCorrector<AngleUnit * 360>::sensor_correction_;
  int field = findField(block_azimuth);

  auto elevation = correction->elevation[channel_id] +
                   correction->getElevationAdjustV3(channel_id, block_azimuth) * (AngleUnit / 100);

  auto azimuth = (block_azimuth + MAX_AZIMUTH_LENGTH - correction->startFrame[field]) * 2 -
                 correction->azimuth[channel_id] +
                 correction->getAzimuthAdjustV3(channel_id, block_azimuth) * (AngleUnit / 100);
  azimuth = (MAX_AZIMUTH_LENGTH + azimuth) % MAX_AZIMUTH_LENGTH;

  float azimuth_rad = 2.f * azimuth * M_PI / MAX_AZIMUTH_LENGTH;
  float elevation_rad = 2.f * elevation * M_PI / MAX_AZIMUTH_LENGTH;

  // The point elevation above is allowed to be negative but the corrected_elevation for use as an
  // index is not
  elevation = (MAX_AZIMUTH_LENGTH + elevation) % MAX_AZIMUTH_LENGTH;

  return std::make_tuple(
    static_cast<uint32_t>(azimuth), static_cast<uint32_t>(elevation), azimuth_rad, elevation_rad);
}

// Explicit template instantiation to prevent linker errors
template class AngleCorrectorCorrectionBased<128, 100 * 256>;

}  // namespace drivers
}  // namespace nebula