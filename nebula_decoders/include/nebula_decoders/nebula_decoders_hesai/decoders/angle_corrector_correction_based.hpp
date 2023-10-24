#pragma once

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector.hpp"

#include <cstdint>

#define _(x) '"' << #x << "\": " << x << ", "

namespace nebula
{
namespace drivers
{

template <size_t ChannelN, size_t AngleUnit>
class AngleCorrectorCorrectionBased : public AngleCorrector
{
private:
  static constexpr size_t MAX_AZIMUTH_LENGTH = 360 * AngleUnit;
  rclcpp::Logger logger_;

  std::array<float, MAX_AZIMUTH_LENGTH> cos_{};
  std::array<float, MAX_AZIMUTH_LENGTH> sin_{};

  /// @brief For a given azimuth value, find its corresponding output field
  /// @param azimuth The azimuth to get the field for
  /// @return The correct output field, as specified in @ref HesaiCorrection
  int findField(uint32_t azimuth)
  {
    // Assumes that:
    // * none of the startFrames are defined as > 360 deg (< 0 not possible since they are unsigned)
    // * the fields are arranged in ascending order (e.g. field 1: 20-140deg, field 2: 140-260deg
    // etc.) These assumptions hold for AT128E2X.
    int field = sensor_correction_->frameNumber - 1;
    for (size_t i = 0; i < sensor_correction_->frameNumber; ++i) {
      if (azimuth < sensor_correction_->startFrame[i]) return field;
      field = i;
    }

    // This is never reached if sensor_correction_ is correct
    return field;
  }

public:
  AngleCorrectorCorrectionBased(
    const std::shared_ptr<HesaiCalibrationConfiguration> & sensor_calibration,
    const std::shared_ptr<HesaiCorrection> & sensor_correction)
  : AngleCorrector(sensor_calibration, sensor_correction),
    logger_(rclcpp::get_logger("AngleCorrectorCorrectionBased"))
  {
    if (sensor_correction == nullptr) {
      throw std::runtime_error(
        "Cannot instantiate AngleCorrectorCalibrationBased without calibration data");
    }

    logger_.set_level(rclcpp::Logger::Level::Debug);

    for (size_t i = 0; i < MAX_AZIMUTH_LENGTH; ++i) {
      float rad = 2.f * i * M_PI / MAX_AZIMUTH_LENGTH;
      cos_[i] = cosf(rad);
      sin_[i] = sinf(rad);
    }
  }

  CorrectedAngleData getCorrectedAngleData(uint32_t block_azimuth, uint32_t channel_id) override
  {
    const auto & correction = AngleCorrector::sensor_correction_;
    int field = findField(block_azimuth);

    auto elevation =
      correction->elevation[channel_id] +
      correction->getElevationAdjustV3(channel_id, block_azimuth) * (AngleUnit / 100);
    elevation = (MAX_AZIMUTH_LENGTH + elevation) % MAX_AZIMUTH_LENGTH;

    auto azimuth = (block_azimuth + MAX_AZIMUTH_LENGTH - correction->startFrame[field]) * 2 -
                   correction->azimuth[channel_id] +
                   correction->getAzimuthAdjustV3(channel_id, block_azimuth) * (AngleUnit / 100);
    azimuth = (MAX_AZIMUTH_LENGTH + azimuth) % MAX_AZIMUTH_LENGTH;

    float azimuth_rad = 2.f * azimuth * M_PI / MAX_AZIMUTH_LENGTH;
    float elevation_rad = 2.f * elevation * M_PI / MAX_AZIMUTH_LENGTH;

    return {azimuth_rad,     elevation_rad,   sin_[azimuth], cos_[azimuth],
            sin_[elevation], cos_[elevation], field};
  }

  bool hasScanned(uint32_t current_azimuth, uint32_t last_azimuth, uint32_t sync_azimuth) override
  {
    const auto & correction = AngleCorrector::sensor_correction_;

    for (size_t i = 0; i < correction->frameNumber; ++i) {
      uint32_t encoder_sync_angle = correction->outputAngleToEncoderAngle(sync_azimuth, i);

      if (last_azimuth <= encoder_sync_angle &&
          encoder_sync_angle <= current_azimuth) {
        return true;
      }
    }

    return false;
  }
};

}  // namespace drivers
}  // namespace nebula
