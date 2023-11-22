#include "innoviz_ros_decoder_test_raven.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>

std::shared_ptr<nebula::ros::InnovizRosDecoderTest> innoviz_driver;

TEST(TestDecoder, TestPcd)
{
  std::cout << "TEST(TestDecoder, TestPcd)" << std::endl;
  innoviz_driver->ReadBag();
}

int main(int argc, char * argv[])
{
  std::cout << "innoviz_ros_decoder_test_raven.cpp" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_innoviz_decoder_test";

  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  innoviz_driver = std::make_shared<nebula::ros::InnovizRosDecoderTest>(options, node_name);
  exec.add_node(innoviz_driver->get_node_base_interface());

  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Get Status");
  nebula::Status driver_status = innoviz_driver->GetStatus();
  int result = 0;
  if (driver_status == nebula::Status::OK) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Reading Started");
    //    innoviz_driver->ReadBag();
    result = RUN_ALL_TESTS();
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(node_name), driver_status);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Ending");
  innoviz_driver.reset();
  rclcpp::shutdown();

  return result;
}
