// Copyright 2024 TIER IV, Inc.

#include "hesai_ros_scan_cutting_test_main.hpp"

#include "hesai_ros_decoder_test.hpp"

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/point_types.hpp>
#include <nebula_decoders/nebula_decoders_common/angles.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>

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

// Checks that there are never any points outside the defined FoV, and that there are points close
// to the FoV boundaries
TEST_P(ScanCuttingTest, FovAlignment)
{
  int check_cnt = 0;

  auto fov_min_rad = drivers::deg2rad(hesai_driver_->params_.fov_start);
  auto fov_max_rad = drivers::deg2rad(hesai_driver_->params_.fov_end);
  // The threshold near the FoV borders within which each channel has to have at least one point
  auto near_threshold_rad = drivers::deg2rad(.3);

  // Skip the first n clouds (because the recorded rosbags do not always contain full scans and the
  // tests would thus fail)
  int skip_first = 2;

  auto scan_callback = [&](uint64_t, uint64_t, nebula::drivers::NebulaPointCloudPtr pointcloud) {
    if (!pointcloud) return;

    if (skip_first > 0) {
      skip_first--;
      return;
    }

    bool none_outside_fov = true;
    std::map<uint16_t, bool> has_points_near_start;
    std::map<uint16_t, bool> has_points_near_end;

    for (const drivers::NebulaPoint & p : pointcloud->points) {
      none_outside_fov &= drivers::angle_is_between(fov_min_rad, fov_max_rad, p.azimuth);

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
    EXPECT_TRUE(none_outside_fov);
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
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
