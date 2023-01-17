#include <gtest/gtest.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "velodyne_ros_decoder_test_vlp16.hpp"

std::shared_ptr<nebula::ros::VelodyneRosDecoderTest> velodyne_driver;

TEST(TestDecoder, TestPcd)
{
  std::cout << "TEST(TestDecoder, TestPcd)" << std::endl;
  velodyne_driver->ReadBag();
}

int main(int argc, char * argv[])
{
  std::cout << "velodyne_ros_decoder_test_main_vlp16.cpp" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_velodyne_decoder_test";

  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  velodyne_driver = std::make_shared<nebula::ros::VelodyneRosDecoderTest>(options, node_name);
  exec.add_node(velodyne_driver->get_node_base_interface());

  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Get Status");
  nebula::Status driver_status = velodyne_driver->GetStatus();
  int result = 0;
  if (driver_status == nebula::Status::OK) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Reading Started");
    result = RUN_ALL_TESTS();
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(node_name), driver_status);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Ending");
  velodyne_driver.reset();
  rclcpp::shutdown();

  return result;
}
