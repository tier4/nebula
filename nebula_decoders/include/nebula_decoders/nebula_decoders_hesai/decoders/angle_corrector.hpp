#pragma once

#include "nebula_common/hesai/hesai_common.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>

namespace nebula
{
namespace drivers
{

struct CorrectedAngleData
{
  float azimuth_rad;
  float elevation_rad;
  float sin_azimuth;
  float cos_azimuth;
  float sin_elevation;
  float cos_elevation;
};

/// @brief Handles angle correction for given azimuth/channel combinations, as well as trigonometry
/// lookup tables
class AngleCorrector
{
protected:
  const std::shared_ptr<HesaiCalibrationConfiguration> sensor_calibration_;
  const std::shared_ptr<HesaiCorrection> sensor_correction_;

public:
  AngleCorrector(
    const std::shared_ptr<HesaiCalibrationConfiguration> & sensor_calibration,
    const std::shared_ptr<HesaiCorrection> & sensor_correction)
  : sensor_calibration_(sensor_calibration), sensor_correction_(sensor_correction)
  {
  }

  /// @brief Get the corrected azimuth and elevation for a given block and channel, along with their
  /// sin/cos values.
  /// @param block_azimuth The block's azimuth (including optional fine azimuth), in the sensor's
  /// angle unit
  /// @param channel_id The laser channel's id
  /// @return The corrected angles (azimuth, elevation) in radians and their sin/cos values
  virtual CorrectedAngleData getCorrectedAngleData(uint32_t block_azimuth, uint32_t channel_id) = 0;

  /// @brief Returns true if the current azimuth lies in a different (new) scan compared to the last
  /// azimuth
  /// @param current_azimuth The current azimuth value in the sensor's angle resolution
  /// @param last_azimuth The last azimuth in the sensor's angle resolution
  /// @param sync_azimuth The azimuth set in the sensor configuration, for which the
  /// timestamp is aligned to the full second
  /// @return true if the current azimuth is in a different scan than the last one, false otherwise
  virtual bool hasScanned(uint32_t current_azimuth, uint32_t last_azimuth, uint32_t sync_azimuth) = 0;
};

}  // namespace drivers
}  // namespace nebula