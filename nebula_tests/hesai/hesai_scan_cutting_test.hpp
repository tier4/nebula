#pragma once

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector_calibration_based.hpp>
#include <nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector_correction_based.hpp>

#include <boost/algorithm/string/predicate.hpp>

#include <gtest/gtest.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#ifndef _SRC_CALIBRATION_DIR_PATH
#define _SRC_CALIBRATION_DIR_PATH ""
#endif

namespace nebula::test
{

using drivers::AngleCorrectorCalibrationBased;
using drivers::AngleCorrectorCorrectionBased;
using drivers::deg2rad;
using drivers::HesaiCalibrationConfiguration;
using drivers::HesaiCalibrationConfigurationBase;
using drivers::HesaiCorrection;

struct HesaiScanCuttingTestParams
{
  std::string calibration_file_path;
  uint16_t cloud_min_angle;
  uint16_t cloud_max_angle;
  uint16_t scan_phase;
};

inline std::ostream & operator<<(std::ostream & os, HesaiScanCuttingTestParams const & arg)
{
  return os << "calibration_file_path=" << arg.calibration_file_path << ", "
            << "cloud_min_angle=" << arg.cloud_min_angle << ", "
            << "cloud_max_angle=" << arg.cloud_max_angle << ", "
            << "scan_phase=" << arg.scan_phase;
}

inline float canonical_angle(float angle_deg)
{
  while (angle_deg > 360) angle_deg -= 360;
  while (angle_deg < 0) angle_deg += 360;
  return angle_deg;
}

class AngleCorrectorWrapper
{
public:
  virtual bool blockCompletesScan(uint32_t block_azimuth, uint32_t last_azimuth) = 0;

  virtual float getCorrectedAzimuth(uint32_t block_azimuth, uint32_t channel_id) = 0;
};

template <typename T>
class AngleCorrectorWrapperImpl final : public AngleCorrectorWrapper
{
public:
  AngleCorrectorWrapperImpl(
    const std::shared_ptr<const typename T::correction_data_t> & sensor_calibration,
    float scan_cut_azimuth_rad)
  : angle_corrector_(sensor_calibration, scan_cut_azimuth_rad)
  {
  }

  bool blockCompletesScan(uint32_t block_azimuth, uint32_t last_azimuth) override
  {
    return angle_corrector_.blockCompletesScan(block_azimuth, last_azimuth);
  }

  float getCorrectedAzimuth(uint32_t block_azimuth, uint32_t channel_id) override
  {
    return angle_corrector_.getCorrectedAngleData(block_azimuth, channel_id).azimuth_rad;
  }

private:
  T angle_corrector_;
};

class ScanCuttingTest : public ::testing::TestWithParam<nebula::test::HesaiScanCuttingTestParams>
{
protected:
  void SetUp() override
  {
    auto params = GetParam();

    uint16_t cut_angle_deg{};
    if (params.cloud_min_angle == 0 && params.cloud_max_angle == 360) {
      cut_angle_deg = params.scan_phase;
    } else {
      cut_angle_deg = params.cloud_max_angle;
    }

    cut_angle_rad_ = deg2rad(cut_angle_deg);

    auto absolute_path = _SRC_CALIBRATION_DIR_PATH "hesai/" + params.calibration_file_path;

    if (boost::ends_with(absolute_path, ".csv")) {
      angle_unit_ = 100;
      n_expected_cuts_ = 1;

      auto calibration = std::make_shared<HesaiCalibrationConfiguration>();
      auto status = calibration->LoadFromFile(absolute_path);

      if (status != Status::OK) {
        throw std::runtime_error(
          "Could not load calibration from " + absolute_path + ": " +
          (std::stringstream{} << status).str());
      }

      n_channels_ = calibration->elev_angle_map.size();
      switch (n_channels_) {
        case 32:
          angle_corrector_ =
            std::make_shared<AngleCorrectorWrapperImpl<AngleCorrectorCalibrationBased<32, 100>>>(
              calibration, cut_angle_rad_);
          break;
        case 40:
          angle_corrector_ =
            std::make_shared<AngleCorrectorWrapperImpl<AngleCorrectorCalibrationBased<40, 100>>>(
              calibration, cut_angle_rad_);
          break;
        case 64:
          angle_corrector_ =
            std::make_shared<AngleCorrectorWrapperImpl<AngleCorrectorCalibrationBased<64, 100>>>(
              calibration, cut_angle_rad_);
          break;
        case 128:
          angle_corrector_ =
            std::make_shared<AngleCorrectorWrapperImpl<AngleCorrectorCalibrationBased<128, 100>>>(
              calibration, cut_angle_rad_);
          break;
        default:
          throw std::runtime_error(
            "Expected 32, 40, 64, or 128 channels but got " + std::to_string(n_channels_));
      }
    } else {
      angle_unit_ = 25600;
      n_channels_ = 128;
      n_expected_cuts_ = 3;

      auto correction = std::make_shared<HesaiCorrection>();
      auto status = correction->LoadFromFile(absolute_path);

      if (status != Status::OK) {
        throw std::runtime_error(
          "Could not load correction from " + absolute_path + ": " +
          (std::stringstream{} << status).str());
      }

      angle_corrector_ =
        std::make_shared<AngleCorrectorWrapperImpl<AngleCorrectorCorrectionBased<128, 25600>>>(
          correction, deg2rad(params.cloud_max_angle));
    }

    assert(angle_corrector_);
  }

  void TearDown() override {}

  std::shared_ptr<AngleCorrectorWrapper> angle_corrector_;
  float cut_angle_rad_;
  int angle_unit_;
  size_t n_channels_;
  size_t n_expected_cuts_;
};

}  // namespace nebula::test
