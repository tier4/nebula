#include "hesai_ros_decoder_test_main.hpp"

#include "hesai_common.hpp"
#include "hesai_ros_decoder_test.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <functional>
#include <memory>

namespace nebula
{
namespace test
{

const nebula::ros::HesaiRosDecoderTestParams TEST_CONFIGS[6] = {
  {
    "Pandar40P",
    "Dual",
    "Pandar40P.csv",
    "40p/1673400149412331409",
  },
  {
    "Pandar64",
    "Dual",
    "Pandar64.csv",
    "64/1673403880599376836",
  },
  {
    "PandarAT128",
    "LastStrongest",
    "PandarAT128.csv",
    "at128/1679653308406038376",
    "PandarAT128.dat",
  },
  {
    "PandarQT64",
    "Dual",
    "PandarQT64.csv",
    "qt64/1673401195788312575",
  },
  {
    "PandarXT32",
    "Dual",
    "PandarXT32.csv",
    "xt32/1673400677802009732",
  },
  {
    "PandarXT32M",
    "LastStrongest",
    "PandarXT32M.csv",
    "xt32m/1660893203042895158",
  }};

// Compares geometrical output of decoder against pre-recorded reference pointcloud.
TEST_P(DecoderTest, TestPcd)
{
  pcl::PCDReader pcd_reader;
  rcpputils::fs::path bag_dir(hesai_driver_->params_.bag_path);
  rcpputils::fs::path pcd_dir = bag_dir.parent_path();

  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  int check_cnt = 0;

  auto scan_callback = [&](
                         uint64_t msg_timestamp, uint64_t /*scan_timestamp*/,
                         nebula::drivers::NebulaPointCloudPtr pointcloud) {
    if (!pointcloud) return;

    auto fn = std::to_string(msg_timestamp) + ".pcd";

    auto target_pcd_path = (pcd_dir / fn);
    RCLCPP_DEBUG_STREAM(*logger_, target_pcd_path);
    if (target_pcd_path.exists()) {
      RCLCPP_DEBUG_STREAM(*logger_, "exists: " << target_pcd_path);
      auto rt = pcd_reader.read(target_pcd_path.string(), *ref_pointcloud);
      RCLCPP_DEBUG_STREAM(*logger_, rt << " loaded: " << target_pcd_path);
      checkPCDs(pointcloud, ref_pointcloud);
      check_cnt++;
      // ref_pointcloud.reset(new nebula::drivers::NebulaPointCloud);
      ref_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
  };

  hesai_driver_->ReadBag(scan_callback);
  EXPECT_GT(check_cnt, 0);
}

// Tests if decoders handle timezone settings correctly, i.e. their output timestamps
// are not affected by timezones and are always output in UST.
TEST_P(DecoderTest, TestTimezone)
{
  // For each pointcloud decoded, this will contain the pointcloud timestamp returned by the driver
  std::vector<uint64_t> decoded_timestamps;

  auto scan_callback = [&](
                         uint64_t /*msg_timestamp*/, uint64_t scan_timestamp,
                         nebula::drivers::NebulaPointCloudPtr pointcloud) {
    if (!pointcloud) return;
    decoded_timestamps.push_back(scan_timestamp);
  };

  // First, set the timezone to e.g. GMT, check if setting TZ was successful,
  // then decode scans and record timestamps
  setenv("TZ", "/usr/share/zoneinfo/GMT", 1);
  tzset();
  ASSERT_STREQ(tzname[0], "GMT");
  auto gmt = timezone;
  hesai_driver_->ReadBag(scan_callback);

  // Then, reset driver and timestamps vector for the next decode run
  TearDown();
  SetUp();
  auto decoded_timestamps_cmp = std::vector<uint64_t>(decoded_timestamps);
  decoded_timestamps.clear();

  // Perform the next run with a different timezone, e.g. JST, check if set successfully,
  // then decode and record times again
  setenv("TZ", "/usr/share/zoneinfo/Japan", 1);
  tzset();
  ASSERT_STREQ(tzname[0], "JST");
  auto jst = timezone;
  hesai_driver_->ReadBag(scan_callback);

  // Wrong timezone settings do not throw an error, they just result in UST+0.
  // Thus, verify that timezone setting has effect on local timestamp
  ASSERT_NE(gmt, jst);

  // Assert that the same number (>0) of pointclouds have been decoded,
  // then compare e.g. the last timestamp to verify that it is not affected
  // by timezone settings
  ASSERT_EQ(decoded_timestamps.size(), decoded_timestamps_cmp.size());
  ASSERT_GT(decoded_timestamps.size(), 0U);
  EXPECT_EQ(decoded_timestamps.back(), decoded_timestamps_cmp.back());
}

void DecoderTest::SetUp()
{
  auto decoder_params = GetParam();
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("UnitTest"));
  logger_->set_level(rclcpp::Logger::Level::Info);

  RCLCPP_DEBUG_STREAM(*logger_, "Testing " << decoder_params.sensor_model);
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_hesai_decoder_test";

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  hesai_driver_ =
    std::make_shared<nebula::ros::HesaiRosDecoderTest>(options, node_name, decoder_params);
  exec.add_node(hesai_driver_->get_node_base_interface());

  nebula::Status driver_status = hesai_driver_->GetStatus();
  if (driver_status != nebula::Status::OK) {
    throw std::runtime_error("Could not initialize driver");
  }
}

void DecoderTest::TearDown()
{
  RCLCPP_INFO_STREAM(*logger_, "Tearing down");
  hesai_driver_.reset();
  logger_.reset();
}

INSTANTIATE_TEST_SUITE_P(
  TestMain, DecoderTest, testing::ValuesIn(TEST_CONFIGS),
  [](const testing::TestParamInfo<nebula::ros::HesaiRosDecoderTestParams> & p) {
    return p.param.sensor_model;
  });

}  // namespace test
}  // namespace nebula

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
