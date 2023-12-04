#pragma once

#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/angles.hpp"

#include <cstdint>

namespace nebula
{
namespace drivers
{

template <typename PacketT>
class AngleCorrectorCalibrationBased : public sensor_mixins::AngleCorrectorMixin<PacketT>
{
private:
  static constexpr size_t N_CHANNELS = PacketT::N_CHANNELS;
  static constexpr size_t ANGLE_UNIT = PacketT::DEGREE_SUBDIVISIONS;
  static constexpr size_t MAX_AZIMUTH_LEN = 360 * ANGLE_UNIT;

  std::array<float, N_CHANNELS> elevation_angle_rad_{};
  std::array<float, N_CHANNELS> azimuth_offset_rad_{};
  std::array<float, MAX_AZIMUTH_LEN> block_azimuth_rad_{};

  std::array<float, N_CHANNELS> elevation_cos_{};
  std::array<float, N_CHANNELS> elevation_sin_{};
  std::array<std::array<float, N_CHANNELS>, MAX_AZIMUTH_LEN> azimuth_cos_{};
  std::array<std::array<float, N_CHANNELS>, MAX_AZIMUTH_LEN> azimuth_sin_{};

public:
  explicit AngleCorrectorCalibrationBased(
    const std::shared_ptr<const RobosenseCalibrationConfiguration> & sensor_calibration)
  {
    if (sensor_calibration == nullptr) {
      throw std::runtime_error(
        "Cannot instantiate AngleCorrectorCalibrationBased without calibration data");
    }

    if (sensor_calibration->calibration.size() == 0) {
      throw std::runtime_error(
        "Cannot instantiate AngleCorrectorCalibrationBased with empty calibration data");
    }

    for (size_t channel_id = 0; channel_id < N_CHANNELS; ++channel_id) {
      const auto correction = sensor_calibration->GetCorrection(channel_id);
      float elevation_angle_deg = correction.elevation;
      float azimuth_offset_deg = correction.azimuth;

      elevation_angle_rad_[channel_id] = deg2rad(elevation_angle_deg);
      azimuth_offset_rad_[channel_id] = deg2rad(azimuth_offset_deg);

      elevation_cos_[channel_id] = cosf(elevation_angle_rad_[channel_id]);
      elevation_sin_[channel_id] = sinf(elevation_angle_rad_[channel_id]);
    }

    for (size_t block_azimuth = 0; block_azimuth < MAX_AZIMUTH_LEN; block_azimuth++) {
      block_azimuth_rad_[block_azimuth] = deg2rad(block_azimuth / static_cast<double>(ANGLE_UNIT));

      for (size_t channel_id = 0; channel_id < N_CHANNELS; ++channel_id) {
        float precision_azimuth =
          block_azimuth_rad_[block_azimuth] + azimuth_offset_rad_[channel_id];

          // Robosense azimuths are clockwise, math is counter-clockwise
          precision_azimuth = -precision_azimuth;

        azimuth_cos_[block_azimuth][channel_id] = cosf(precision_azimuth);
        azimuth_sin_[block_azimuth][channel_id] = sinf(precision_azimuth);
      }
    }
  }

  sensor_mixins::CorrectedAngleData getCorrectedAngleData(int32_t raw_azimuth, int32_t raw_elevation) const override
  {
    float azimuth_rad = block_azimuth_rad_[raw_azimuth] + azimuth_offset_rad_[raw_elevation];
    float elevation_rad = elevation_angle_rad_[raw_elevation];

    return {
      azimuth_rad,
      elevation_rad,
      azimuth_sin_[raw_azimuth][raw_elevation],
      azimuth_cos_[raw_azimuth][raw_elevation],
      elevation_sin_[raw_elevation],
      elevation_cos_[raw_elevation]};
  }
};

}  // namespace drivers
}  // namespace nebula