#pragma once

#include "nebula_common/hesai/hesai_common.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>

namespace nebula
{
namespace drivers
{

template <size_t LookupTableSize>
class AngleCorrector
{
private:
  /// @brief Calculate a lookup table for sin or cos
  /// @param is_cos If true, calculate a cos lookup table, otherwise calculate a sin lookup table
  /// @return The lookup table
  static std::array<float, LookupTableSize> calculateLUT(bool is_cos)
  {
    std::array<float, LookupTableSize> lut{};
    for (uint32_t i = 0; i < LookupTableSize; i++) {
      float angle_rad = i / static_cast<float>(LookupTableSize) * 2 * M_PI;
      lut[i] = is_cos ? cosf(angle_rad) : sinf(angle_rad);
    }
    return lut;
  }

protected:
  const std::shared_ptr<HesaiCalibrationConfiguration> sensor_calibration_;
  const std::shared_ptr<HesaiCorrection> sensor_correction_;

public:
  const std::array<float, LookupTableSize> cos_map_{};
  const std::array<float, LookupTableSize> sin_map_{};

  AngleCorrector(
    const std::shared_ptr<HesaiCalibrationConfiguration> & sensor_calibration,
    const std::shared_ptr<HesaiCorrection> & sensor_correction)
  : sensor_calibration_(sensor_calibration),
    sensor_correction_(sensor_correction),
    cos_map_(calculateLUT(true)),
    sin_map_(calculateLUT(false))
  {
  }

  /// @brief Get the corrected azimuth and elevation for a given block and channel
  /// @param block_azimuth The block's azimuth (including optional fine azimuth), in the sensor's
  /// angle unit
  /// @param channel_id The laser channel's id
  /// @return The corrected angles (azimuth, elevation) in the sensor's angle unit, followed by
  /// (azimuth, elevation) in radians
  virtual std::tuple<uint32_t, uint32_t, float, float> getCorrectedAzimuthAndElevation(
    uint32_t block_azimuth, uint32_t channel_id) = 0;

  virtual bool hasScanned(int current_azimuth, int last_azimuth) = 0;
};

}  // namespace drivers
}  // namespace nebula