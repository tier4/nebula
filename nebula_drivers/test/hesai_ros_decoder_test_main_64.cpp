#include "hesai_ros_decoder_test_64.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>

std::shared_ptr<nebula::ros::HesaiRosDecoderTest> hesai_driver;

TEST(TestDecoder, TestPcd)
{
  std::cout << "TEST(TestDecoder, TestPcd)" << std::endl;
  hesai_driver->ReadBag();
}

int main(int argc, char * argv[])
{
  std::cout << "hesai_ros_decoder_test_main_64.cpp" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_hesai_decoder_test";

  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  hesai_driver = std::make_shared<nebula::ros::HesaiRosDecoderTest>(options, node_name);
  exec.add_node(hesai_driver->get_node_base_interface());

  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Get Status");
  nebula::Status driver_status = hesai_driver->GetStatus();
  int result = 0;
  if (driver_status == nebula::Status::OK) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Reading Started");
    //    hesai_driver->ReadBag();
    result = RUN_ALL_TESTS();
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(node_name), driver_status);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Ending");
  hesai_driver.reset();
  rclcpp::shutdown();

  return result;
}
