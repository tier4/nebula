#include "hesai_scan_cutting_test.hpp"

#include "nebula_common/nebula_common.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <iomanip>
#include <ios>
#include <string>

namespace nebula::test
{

const HesaiScanCuttingTestParams TEST_CONFIGS[] = {
  {"Pandar128E4X.csv", 0, 360, 0},   {"Pandar128E4X.csv", 0, 360, 180},
  {"Pandar128E4X.csv", 90, 270, 0},  {"Pandar128E4X.csv", 90, 270, 180},
  {"Pandar128E4X.csv", 270, 90, 0},  {"Pandar128E4X.csv", 270, 90, 180},
  {"PandarAT128.dat", 30, 150, 30},  {"PandarAT128.dat", 30, 150, 90},
  {"PandarAT128.dat", 30, 150, 150}, {"PandarAT128.dat", 60, 120, 90},
};

TEST_P(ScanCuttingTest, TestAngles)
{
  auto params = GetParam();

  const int angle_resolutions[] = {5, 10, 20};

  for (auto resolution : angle_resolutions) {
    auto step = angle_unit_ / resolution;
    auto last_angle = 360 * angle_unit_ - step;
    auto n_cuts = 0;
    for (auto i = 0; i < 360 * angle_unit_; i += step) {
      auto angle = i % (360 * angle_unit_);
      bool cut = angle_corrector_->blockCompletesScan(angle, last_angle);

      if (cut) {
        n_cuts++;

        for (size_t channel_id = 0; channel_id < n_channels_; ++channel_id) {
          auto corrected_angle_rad = angle_corrector_->getCorrectedAzimuth(angle, channel_id);
          EXPECT_GT(corrected_angle_rad, cut_angle_rad_);
        }

        bool any_unfinished = false;
        for (size_t channel_id = 0; channel_id < n_channels_; ++channel_id) {
          auto corrected_angle_rad = angle_corrector_->getCorrectedAzimuth(last_angle, channel_id);
          if (corrected_angle_rad <= cut_angle_rad_) {
            any_unfinished = true;
            break;
          }
        }
        EXPECT_TRUE(any_unfinished);
      }

      last_angle = angle;
    }

    EXPECT_EQ(n_cuts, n_expected_cuts_);
  }
}

INSTANTIATE_TEST_SUITE_P(
  TestMain, ScanCuttingTest, testing::ValuesIn(TEST_CONFIGS),
  [](const testing::TestParamInfo<HesaiScanCuttingTestParams> & p) {
    auto name = p.param.calibration_file_path;
    const auto search_for = ".";
    name = name.substr(0, name.rfind(search_for));
    return name + "_" + std::to_string(p.param.cloud_min_angle) + "_" +
           std::to_string(p.param.cloud_max_angle) + "_" + std::to_string(p.param.scan_phase);
  });

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}

}  // namespace nebula::test
