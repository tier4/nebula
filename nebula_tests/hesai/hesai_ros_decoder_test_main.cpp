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
  {.sensor_model = "Pandar40P",
   .return_mode = "Dual",
   .calibration_file = "Pandar40P.csv",
   .bag_path = "40p/1673400149412331409"},
  {
    .sensor_model = "Pandar64",
    .return_mode = "Dual",
    .calibration_file = "Pandar64.csv",
    .bag_path = "64/1673403880599376836",
  },
  {
    .sensor_model = "PandarAT128",
    .return_mode = "LastStrongest",
    .calibration_file = "PandarAT128.csv",
    .correction_file = "PandarAT128.dat",
    .bag_path = "at128/1679653308406038376",
  },
  {
    .sensor_model = "PandarQT64",
    .return_mode = "Dual",
    .calibration_file = "PandarQT64.csv",
    .bag_path = "qt64/1673401195788312575",
  },
  {
    .sensor_model = "PandarXT32",
    .return_mode = "Dual",
    .calibration_file = "PandarXT32.csv",
    .bag_path = "xt32/1673400677802009732",
  },
  {
    .sensor_model = "PandarXT32M",
    .return_mode = "LastStrongest",
    .calibration_file = "PandarXT32M.csv",
    .bag_path = "xt32m/1660893203042895158",
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
                         uint64_t msg_timestamp, uint64_t scan_timestamp,
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



int main(int argc, char * argv[])
{
  std::cout << "hesai_ros_decoder_test_main.cpp" << std::endl;

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
