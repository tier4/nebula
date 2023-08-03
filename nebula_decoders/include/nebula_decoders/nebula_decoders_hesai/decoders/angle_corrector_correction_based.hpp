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
class AngleCorrectorCorrectionBased : public AngleCorrector<AngleUnit * 360>
{
private:
  static constexpr size_t MAX_AZIMUTH_LENGTH = 360 * AngleUnit;
  rclcpp::Logger logger_;

  int findField(int azimuth)
  {
    const auto & correction = AngleCorrector<AngleUnit * 360>::sensor_correction_;

    int count = 0, field = 0;
    while (count < correction->frameNumber &&
           (((azimuth + MAX_AZIMUTH_LENGTH - correction->startFrame[field]) % MAX_AZIMUTH_LENGTH +
             (correction->endFrame[field] + MAX_AZIMUTH_LENGTH - azimuth) % MAX_AZIMUTH_LENGTH) !=
            (correction->endFrame[field] + MAX_AZIMUTH_LENGTH - correction->startFrame[field]) %
              MAX_AZIMUTH_LENGTH)) {
      field = (field + 1) % correction->frameNumber;
      count++;
    }

    return field;
  }

public:
  AngleCorrectorCorrectionBased(
    const std::shared_ptr<HesaiCalibrationConfiguration> & sensor_calibration,
    const std::shared_ptr<HesaiCorrection> & sensor_correction);

  std::tuple<uint32_t, uint32_t, float, float> getCorrectedAzimuthAndElevation(
    uint32_t block_azimuth, uint32_t channel_id) override;

  bool hasScanned(int current_azimuth, int last_azimuth) override
  {
    int field = findField(current_azimuth);
    int last_field = findField(last_azimuth);

    RCLCPP_DEBUG_STREAM(
      logger_, '{' << _(field) << _(last_field) << _(current_azimuth) << _(last_azimuth) << '}');
    return last_field != field;
  }
};

}  // namespace drivers
}  // namespace nebula