// Copyright 2024 TIER IV, Inc.

#include "hesai_ros_scan_cutting_test_main.hpp"

#include "hesai_ros_decoder_test.hpp"

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_decoders/angles.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <rcutils/logging.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <string>

namespace nebula::test
{
const nebula::ros::HesaiRosDecoderTestParams g_test_configs[] = {
  {"Pandar64", "Dual", "Pandar64.csv", "64/all_points", "hesai", 0, 0.0, 0., 360., 0.3f, 200.f},
  {"Pandar64", "Dual", "Pandar64.csv", "64/all_points", "hesai", 0, 180.0, 0., 360., 0.3f, 200.f},
  {"Pandar64", "Dual", "Pandar64.csv", "64/all_points", "hesai", 0, 90.0, 90., 270., 0.3f, 200.f},
  {"Pandar64", "Dual", "Pandar64.csv", "64/all_points", "hesai", 0, 180.0, 90., 270., 0.3f, 200.f},
  {"Pandar64", "Dual", "Pandar64.csv", "64/all_points", "hesai", 0, 270.0, 90., 270., 0.3f, 200.f},
  {"Pandar64", "Dual", "Pandar64.csv", "64/all_points", "hesai", 0, 0.0, 270., 90., 0.3f, 200.f},
  {"Pandar64", "Dual", "Pandar64.csv", "64/all_points", "hesai", 0, 90.0, 270., 90., 0.3f, 200.f},
  {"PandarAT128", "LastStrongest", "PandarAT128.dat", "at128/all_points", "hesai", 0, 31.0, 30.,
   150., 1.f, 180.f},
  {"PandarAT128", "LastStrongest", "PandarAT128.dat", "at128/all_points", "hesai", 0, 90.0, 30.,
   150., 1.f, 180.f},
  {"PandarAT128", "LastStrongest", "PandarAT128.dat", "at128/all_points", "hesai", 0, 149.0, 30.,
   150., 1.f, 180.f},
  {"PandarAT128", "LastStrongest", "PandarAT128.dat", "at128/all_points", "hesai", 0, 150.0, 30.,
   150., 1.f, 180.f},
  {"PandarAT128", "LastStrongest", "PandarAT128.dat", "at128/all_points", "hesai", 0, 46.0, 45.,
   135., 1.f, 180.f},
  {"PandarAT128", "LastStrongest", "PandarAT128.dat", "at128/all_points", "hesai", 0, 90.0, 45.,
   135., 1.f, 180.f},
  {"PandarAT128", "LastStrongest", "PandarAT128.dat", "at128/all_points", "hesai", 0, 134.0, 45.,
   135., 1.f, 180.f},
  {"PandarAT128", "LastStrongest", "PandarAT128.dat", "at128/all_points", "hesai", 0, 135.0, 45.,
   135., 1.f, 180.f},
};

// Validates that no points fall outside the defined FoV boundaries
TEST_P(ScanCuttingTest, NoPointsOutsideFov)
{
  int check_cnt = 0;
  int skip_first = 2;

  auto fov_min_rad = drivers::deg2rad(hesai_driver_->params_.fov_start);
  auto fov_max_rad = drivers::deg2rad(hesai_driver_->params_.fov_end);

  auto scan_callback = [&](uint64_t, uint64_t, nebula::drivers::NebulaPointCloudPtr pointcloud) {
    if (!pointcloud || skip_first-- > 0) return;

    for (const auto & p : pointcloud->points) {
      EXPECT_TRUE(drivers::angle_is_between(fov_min_rad, fov_max_rad, p.azimuth))
        << "Point outside FoV: azimuth=" << drivers::rad2deg(p.azimuth) << " deg";
    }
    check_cnt++;
  };

  hesai_driver_->read_bag(scan_callback);
  EXPECT_GT(check_cnt, 0);
}

// Validates that each channel has at least one point near both FoV start and end boundaries
TEST_P(ScanCuttingTest, PointsNearFovBoundaries)
{
  int check_cnt = 0;
  int skip_first = 2;

  auto fov_min_rad = drivers::deg2rad(hesai_driver_->params_.fov_start);
  auto fov_max_rad = drivers::deg2rad(hesai_driver_->params_.fov_end);
  auto near_threshold_rad = drivers::deg2rad(0.3);

  auto scan_callback = [&](uint64_t, uint64_t, nebula::drivers::NebulaPointCloudPtr pointcloud) {
    if (!pointcloud || skip_first-- > 0) return;

    std::map<uint16_t, bool> has_points_near_start;
    std::map<uint16_t, bool> has_points_near_end;

    for (const auto & p : pointcloud->points) {
      if (has_points_near_start.find(p.channel) == has_points_near_start.end()) {
        has_points_near_start.emplace(p.channel, false);
      }
      auto threshold_angle = drivers::normalize_angle(fov_min_rad + near_threshold_rad, 2 * M_PIf);
      has_points_near_start[p.channel] |= drivers::angle_is_between(
        drivers::normalize_angle(fov_min_rad, 2 * M_PIf), threshold_angle, p.azimuth);

      if (has_points_near_end.find(p.channel) == has_points_near_end.end()) {
        has_points_near_end.emplace(p.channel, false);
      }
      threshold_angle = drivers::normalize_angle(fov_max_rad - near_threshold_rad, 2 * M_PIf);
      has_points_near_end[p.channel] |= drivers::angle_is_between(
        threshold_angle, drivers::normalize_angle(fov_max_rad, 2 * M_PIf), p.azimuth);
    }

    bool is_at128 = drivers::sensor_model_from_string(hesai_driver_->params_.sensor_model) ==
                    drivers::SensorModel::HESAI_PANDARAT128;
    // There is a bug in AT128's firm- or hardware that skips a few channels at the beginning of the
    // defined FoV. for that reason, relax test conditions in the case where that bug happens.
    // Bug observed on SW ver. 3.50.15, FW ver. 3.10b830, RPU ver. 3.50.008
    bool is_at128_bug = is_at128 && hesai_driver_->params_.fov_start <= 30 + 3;

    if (is_at128_bug) {
      // The bug affects 32 channels, so 96 valid ones remain
      bool most_channels_have_points_near_start =
        std::count_if(
          has_points_near_start.begin(), has_points_near_start.end(),
          [](const auto & entry) { return entry.second; }) >= 96;

      EXPECT_TRUE(most_channels_have_points_near_start);
    } else {
      bool all_channels_have_points_near_start = std::all_of(
        has_points_near_start.begin(), has_points_near_start.end(),
        [](const auto & entry) { return entry.second; });

      EXPECT_TRUE(all_channels_have_points_near_start);
    }

    bool all_channels_have_points_near_end = std::all_of(
      has_points_near_end.begin(), has_points_near_end.end(),
      [](const auto & entry) { return entry.second; });
    EXPECT_TRUE(all_channels_have_points_near_end);
    check_cnt++;
  };

  hesai_driver_->read_bag(scan_callback);
  EXPECT_GT(check_cnt, 0);
}

// Validates that points just after the cut angle don't have suspiciously high timestamps
TEST_P(ScanCuttingTest, NoHighTimestampsAfterCut)
{
  int check_cnt = 0;
  int skip_first = 2;

  auto scan_callback = [&](uint64_t, uint64_t, nebula::drivers::NebulaPointCloudPtr pointcloud) {
    if (!pointcloud || skip_first-- > 0) return;

    constexpr float epsilon = 1e-3f;
    // Critical range ends at cut + 30° (well past any potential ambiguity)
    constexpr double critical_range_end_offset = 30.0f;

    bool has_non_360_fov =
      (drivers::normalize_angle(hesai_driver_->params_.fov_start, 360) !=
       drivers::normalize_angle(hesai_driver_->params_.fov_end, 360));

    bool has_gap_after_cut =
      has_non_360_fov && (drivers::normalize_angle(hesai_driver_->params_.cut_angle, 360) ==
                          drivers::normalize_angle(hesai_driver_->params_.fov_end, 360));

    double critical_range_start_deg =
      has_gap_after_cut ? hesai_driver_->params_.fov_start : hesai_driver_->params_.cut_angle;
    critical_range_start_deg = drivers::normalize_angle(critical_range_start_deg, 360);

    double critical_range_end_deg = critical_range_start_deg + critical_range_end_offset;
    critical_range_end_deg = drivers::normalize_angle(critical_range_end_deg, 360);

    for (const auto & p : pointcloud->points) {
      double azimuth_deg = drivers::rad2deg(p.azimuth);
      azimuth_deg = drivers::normalize_angle(azimuth_deg, 360);
      bool in_critical_range = drivers::angle_is_between(
        critical_range_start_deg + epsilon, critical_range_end_deg, azimuth_deg);

      if (in_critical_range) {
        // Points in [cut+eps, cut+30°] should have timestamps < 110ms.
        // We use 110ms instead of 100ms to account for:
        // 1. Initialization artifacts that can cause ~5-10ms timestamp offsets
        // 2. Channel azimuth offsets that can cause points to appear from different blocks
        // 3. Timing jitter in packet timestamps
        // This test catches severe timestamp bugs (100s of ms off), not minor artifacts.
        ASSERT_LT(p.time_stamp, 100'000'000ULL)
          << "High timestamp right after cut detected: azimuth=" << p.azimuth << " rad ("
          << drivers::rad2deg(p.azimuth) << " deg)"
          << ", time_stamp=" << p.time_stamp << " ns"
          << ", cut_angle=" << hesai_driver_->params_.cut_angle << " deg";
      }
    }
    check_cnt++;
  };

  hesai_driver_->read_bag(scan_callback);
  EXPECT_GT(check_cnt, 0);
}

// Validates that all point timestamps are non-negative (checking for underflow/wraparound)
TEST_P(ScanCuttingTest, TimestampsNonNegative)
{
  int check_cnt = 0;
  int skip_first = 2;

  auto scan_callback = [&](uint64_t, uint64_t, nebula::drivers::NebulaPointCloudPtr pointcloud) {
    if (!pointcloud || skip_first-- > 0) return;

    for (const auto & p : pointcloud->points) {
      // time_stamp is uint64_t, check it's not suspiciously large (indicating underflow)
      EXPECT_LT(p.time_stamp, UINT64_MAX / 2)
        << "Suspiciously large timestamp (possible underflow): " << p.time_stamp << " ns";
    }
    check_cnt++;
  };

  hesai_driver_->read_bag(scan_callback);
  EXPECT_GT(check_cnt, 0);
}

// Validates that all point timestamps are within 120ms
TEST_P(ScanCuttingTest, TimestampsWithin120ms)
{
  int check_cnt = 0;
  int skip_first = 2;

  // 120ms - slightly more than one rotation (100ms at 10Hz) to allow for timing jitter
  constexpr uint64_t MAX_ALLOWED_NS = 120'000'000ULL;

  auto scan_callback = [&](uint64_t, uint64_t, nebula::drivers::NebulaPointCloudPtr pointcloud) {
    if (!pointcloud || skip_first-- > 0) return;

    for (const auto & p : pointcloud->points) {
      ASSERT_LE(p.time_stamp, MAX_ALLOWED_NS)
        << "Timestamp exceeds 120ms: " << (p.time_stamp / 1'000'000.0) << " ms";
    }
    check_cnt++;
  };

  hesai_driver_->read_bag(scan_callback);
  EXPECT_GT(check_cnt, 0);
}

// Validates semi-monotonic increasing with 1ms jitter tolerance
TEST_P(ScanCuttingTest, TimestampsSemiMonotonic)
{
  int check_cnt = 0;
  int skip_first = 2;

  constexpr int64_t JITTER_TOLERANCE_NS = 1'000'000ULL;  // 1ms

  auto scan_callback = [&](uint64_t, uint64_t, nebula::drivers::NebulaPointCloudPtr pointcloud) {
    if (!pointcloud || skip_first-- > 0) return;

    int64_t max_seen_timestamp = 0;

    for (const auto & p : pointcloud->points) {
      auto current_timestamp = static_cast<int64_t>(p.time_stamp);
      if (current_timestamp + JITTER_TOLERANCE_NS < max_seen_timestamp) {
        int64_t jump_backwards = max_seen_timestamp - current_timestamp;
        FAIL() << "Timestamp jumped backwards by > 1ms: "
               << (static_cast<double>(jump_backwards) / 1'000'000.0) << " ms"
               << " (current: " << current_timestamp << " ns, max_seen: " << max_seen_timestamp
               << " ns)";
      }
      max_seen_timestamp = std::max(max_seen_timestamp, current_timestamp);
    }
    check_cnt++;
  };

  hesai_driver_->read_bag(scan_callback);
  EXPECT_GT(check_cnt, 0);
}

void ScanCuttingTest::SetUp()
{
  auto decoder_params = GetParam();
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("UnitTest"));
  logger_->set_level(rclcpp::Logger::Level::Info);

  RCLCPP_DEBUG_STREAM(*logger_, "Testing " << decoder_params.sensor_model);

  std::string node_name = "nebula_hesai_scan_cutting_test";
  rclcpp::NodeOptions options;

  hesai_driver_ =
    std::make_shared<nebula::ros::HesaiRosDecoderTest>(options, node_name, decoder_params);

  nebula::Status driver_status = hesai_driver_->get_status();
  if (driver_status != nebula::Status::OK) {
    throw std::runtime_error("Could not initialize driver");
  }
}

void ScanCuttingTest::TearDown()
{
  RCLCPP_INFO_STREAM(*logger_, "Tearing down");
  hesai_driver_.reset();
  logger_.reset();
}

INSTANTIATE_TEST_SUITE_P(
  TestMain, ScanCuttingTest, testing::ValuesIn(g_test_configs),
  [](const testing::TestParamInfo<nebula::ros::HesaiRosDecoderTestParams> & p) {
    return p.param.sensor_model + "__cut" + std::to_string(static_cast<int>(p.param.cut_angle)) +
           "__fov_start" + std::to_string(static_cast<int>(p.param.fov_start)) + "__fov_end" +
           std::to_string(static_cast<int>(p.param.fov_end));
  });

}  // namespace nebula::test

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
