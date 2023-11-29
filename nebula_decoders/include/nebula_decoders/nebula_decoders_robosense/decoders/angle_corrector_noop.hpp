#pragma once

#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/angle_corrector.hpp"

#include <cstdint>

namespace nebula
{
namespace drivers
{

template <size_t ChannelN, size_t AngleUnit>
class AngleCorrectorCalibrationBased : public AngleCorrector
{
private:
  static constexpr size_t MAX_AZIMUTH_LEN = 360 * AngleUnit;

public:
  explicit AngleCorrectorCalibrationBased(
    const std::shared_ptr<RobosenseCalibrationConfiguration> & sensor_calibration)
  : AngleCorrector(sensor_calibration)
  {
    if (sensor_calibration != nullptr) {
      throw std::runtime_error(
        "No-op angle corrector does not support calibration but non-null calibration was passed to constructor.");
    }
  }

  CorrectedAngleData getCorrectedAngleData(uint32_t azimuth, uint32_t elevation) override
  {
    
    return {
      azimuth_rad,
      elevation_rad,
      azimuth_sin_[azimuth][elevation],
      azimuth_cos_[azimuth][elevation],
      elevation_sin_[elevation],
      elevation_cos_[elevation]};
  }

  bool hasScanned(int current_azimuth, int last_azimuth) override
  {
    return current_azimuth < last_azimuth;
  }
};

}  // namespace drivers
}  // namespace nebula