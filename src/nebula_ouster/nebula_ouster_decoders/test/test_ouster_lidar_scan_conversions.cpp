// Copyright 2026 TIER IV, Inc.

#include "nebula_ouster_decoders/ouster_lidar_scan_conversions.hpp"

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_decoders/angles.hpp>

#include <gtest/gtest.h>
#include <ouster/core/chanfield.h>
#include <ouster/core/lidar_frame.h>
#include <ouster/core/sensor_info.h>
#include <ouster/core/types.h>
#include <ouster/core/xyzlut.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace nebula::drivers
{

using ouster::sdk::core::ChanFieldType;
using ouster::sdk::core::FieldType;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::SensorInfo;
namespace ChanField = ouster::sdk::core::ChanField;

namespace
{

constexpr uint64_t g_base_ts_ns = 1'000'000'000ULL;
constexpr uint64_t g_col_period_ns = 50'000ULL;

FieldOfView<float, Degrees> full_fov()
{
  FieldOfView<float, Degrees> fov{};
  fov.azimuth.start = 0.F;
  fov.azimuth.end = 360.F;
  fov.elevation.start = -90.F;
  fov.elevation.end = 90.F;
  return fov;
}

bool is_valid_column(Eigen::Index col)
{
  return col % 7 != 0;
}

bool has_return(Eigen::Index row, Eigen::Index col)
{
  return (row + col) % 11 != 0;
}

/// Fill a frame with a deterministic pattern: columns divisible by 7 are invalid, pixels with
/// (row + col) % 11 == 0 have no return, all other pixels have range base_range_mm + row + col.
void fill_frame(LidarFrame & frame, uint32_t base_range_mm)
{
  auto range = frame.field<uint32_t>(ChanField::RANGE);
  const bool has_refl = frame.has_field(ChanField::REFLECTIVITY);
  const bool has_signal = frame.has_field(ChanField::SIGNAL);
  for (Eigen::Index col = 0; col < static_cast<Eigen::Index>(frame.w); ++col) {
    frame.status()(col) = is_valid_column(col) ? 1U : 0U;
    frame.timestamp()(col) = g_base_ts_ns + static_cast<uint64_t>(col) * g_col_period_ns;
    for (Eigen::Index row = 0; row < static_cast<Eigen::Index>(frame.h); ++row) {
      range(row, col) =
        has_return(row, col) ? base_range_mm + static_cast<uint32_t>(row + col) : 0U;
      if (has_refl) {
        frame.field<uint8_t>(ChanField::REFLECTIVITY)(row, col) =
          static_cast<uint8_t>((row + col) & 0xFF);
      }
      if (has_signal) {
        frame.field<uint16_t>(ChanField::SIGNAL)(row, col) =
          static_cast<uint16_t>((row * 37 + col * 5) & 0xFFFF);
      }
    }
  }
}

}  // namespace

class TestOusterLidarScanConversions : public ::testing::Test
{
protected:
  void SetUp() override
  {
    sensor_info_ = SensorInfo::from_default(ouster::sdk::core::LidarMode::_1024x10);
  }

  [[nodiscard]] std::unique_ptr<LidarFrame> make_frame(
    const std::vector<FieldType> & fields, uint32_t base_range_mm = 20000U) const
  {
    auto frame = std::make_unique<LidarFrame>(sensor_info_, fields);
    fill_frame(*frame, base_range_mm);
    return frame;
  }

  [[nodiscard]] std::unique_ptr<LidarFrame> make_default_frame(
    uint32_t base_range_mm = 20000U) const
  {
    auto frame = std::make_unique<LidarFrame>(sensor_info_);
    fill_frame(*frame, base_range_mm);
    return frame;
  }

  std::shared_ptr<SensorInfo> sensor_info_;
};

TEST_F(TestOusterLidarScanConversions, MatchesSdkProjection)
{
  const auto frame = make_default_frame();
  ASSERT_TRUE(frame->has_field(ChanField::REFLECTIVITY));
  const OusterPointCloudConverter converter(*sensor_info_, full_fov());
  const auto cloud = converter.convert(*frame);

  const ouster::sdk::core::XYZLut lut(*sensor_info_, false);
  const auto range = frame->field<uint32_t>(ChanField::RANGE);
  const ouster::sdk::core::PointCloudXYZd xyz = lut(range);
  const auto refl = frame->field<uint8_t>(ChanField::REFLECTIVITY);

  const auto h = static_cast<Eigen::Index>(frame->h);
  const auto w = static_cast<Eigen::Index>(frame->w);
  size_t k = 0;
  for (Eigen::Index row = 0; row < h; ++row) {
    for (Eigen::Index col = 0; col < w; ++col) {
      if (!is_valid_column(col) || !has_return(row, col)) {
        continue;
      }
      ASSERT_LT(k, cloud->size());
      const auto & p = (*cloud)[k++];
      const Eigen::Index i = row * w + col;
      EXPECT_NEAR(p.x, xyz(i, 0), 1e-4);
      EXPECT_NEAR(p.y, xyz(i, 1), 1e-4);
      EXPECT_NEAR(p.z, xyz(i, 2), 1e-4);
      EXPECT_EQ(p.channel, row);
      EXPECT_EQ(p.intensity, refl(row, col));
      EXPECT_FLOAT_EQ(p.distance, static_cast<float>(range(row, col)) * 1e-3F);
      EXPECT_FLOAT_EQ(
        p.elevation, deg2rad(static_cast<float>(sensor_info_->beam_altitude_angles[row])));
      EXPECT_EQ(p.time_stamp, static_cast<uint32_t>((col - 1) * g_col_period_ns));
      EXPECT_EQ(p.return_type, static_cast<uint8_t>(ReturnType::STRONGEST));

      // At long range, the beam azimuth matches the azimuth of the projected point up to the
      // beam origin offset.
      const double point_azimuth = normalize_angle(std::atan2(xyz(i, 1), xyz(i, 0)), 2 * M_PI);
      double diff = std::abs(point_azimuth - p.azimuth);
      diff = std::min(diff, 2 * M_PI - diff);
      EXPECT_LT(diff, deg2rad(0.1));
    }
  }
  EXPECT_EQ(k, cloud->size());
}

TEST_F(TestOusterLidarScanConversions, SignalUsedWhenNoReflectivity)
{
  const auto frame = make_frame(
    {FieldType(ChanField::RANGE, ChanFieldType::UINT32),
     FieldType(ChanField::SIGNAL, ChanFieldType::UINT16)});
  const OusterPointCloudConverter converter(*sensor_info_, full_fov());
  const auto cloud = converter.convert(*frame);
  ASSERT_FALSE(cloud->empty());

  const auto signal = frame->field<uint16_t>(ChanField::SIGNAL);
  // First valid pixel is row 0, col 1.
  const auto & p = cloud->front();
  EXPECT_EQ(p.channel, 0);
  EXPECT_EQ(p.intensity, std::min<uint16_t>(255U, signal(0, 1) >> 4));
  // Saturated values are clamped.
  bool saw_saturated = false;
  for (const auto & q : *cloud) {
    saw_saturated |= q.intensity == 255U;
  }
  EXPECT_TRUE(saw_saturated);
}

TEST_F(TestOusterLidarScanConversions, ZeroIntensityWithoutIntensityFields)
{
  const auto frame = make_frame({FieldType(ChanField::RANGE, ChanFieldType::UINT32)});
  const OusterPointCloudConverter converter(*sensor_info_, full_fov());
  const auto cloud = converter.convert(*frame);
  ASSERT_FALSE(cloud->empty());
  for (const auto & p : *cloud) {
    EXPECT_EQ(p.intensity, 0U);
  }
}

TEST_F(TestOusterLidarScanConversions, FovCropsAzimuthAndElevation)
{
  const auto frame = make_default_frame();
  FieldOfView<float, Degrees> fov{};
  fov.azimuth.start = 90.F;
  fov.azimuth.end = 180.F;
  fov.elevation.start = -5.F;
  fov.elevation.end = 5.F;

  const OusterPointCloudConverter full(*sensor_info_, full_fov());
  const OusterPointCloudConverter cropped(*sensor_info_, fov);
  const auto full_cloud = full.convert(*frame);
  const auto cloud = cropped.convert(*frame);
  ASSERT_FALSE(cloud->empty());
  EXPECT_LT(cloud->size(), full_cloud->size());
  for (const auto & p : *cloud) {
    EXPECT_GE(p.azimuth, deg2rad(fov.azimuth.start));
    EXPECT_LE(p.azimuth, deg2rad(fov.azimuth.end));
    EXPECT_GE(p.elevation, deg2rad(fov.elevation.start));
    EXPECT_LE(p.elevation, deg2rad(fov.elevation.end));
  }
}

TEST_F(TestOusterLidarScanConversions, EmptyForUnusableFrames)
{
  const OusterPointCloudConverter converter(*sensor_info_, full_fov());

  // Dimensions differing from the converter's sensor.
  const LidarFrame wrong_size(SensorInfo::from_default(ouster::sdk::core::LidarMode::_512x10));
  EXPECT_TRUE(converter.convert(wrong_size)->empty());

  // No RANGE field.
  const LidarFrame no_range(sensor_info_, {FieldType(ChanField::SIGNAL, ChanFieldType::UINT16)});
  EXPECT_TRUE(converter.convert(no_range)->empty());

  // No valid columns.
  auto no_valid = make_default_frame();
  no_valid->status().setZero();
  EXPECT_TRUE(converter.convert(*no_valid)->empty());
}

TEST_F(TestOusterLidarScanConversions, DualReturnSensorSetsReturnType)
{
  // RNG19_RFL8_SIG16_NIR16_DUAL is a dual-return profile.
  sensor_info_->format.udp_profile_lidar =
    ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL;
  const auto frame = make_default_frame();
  const OusterPointCloudConverter converter(*sensor_info_, full_fov());
  const auto cloud = converter.convert(*frame);
  ASSERT_FALSE(cloud->empty());
  EXPECT_EQ(cloud->front().return_type, static_cast<uint8_t>(ReturnType::LAST));
}

}  // namespace nebula::drivers
