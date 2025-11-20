// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "continental_ros_decoder_test_ars548.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

std::shared_ptr<nebula::ros::ContinentalRosDecoderTest> continental_driver;

TEST(TestDecoder, TestBag)
{
  std::cout << "TEST(TestDecoder, TestBag)" << std::endl;
  continental_driver->read_bag();
}

int main(int argc, char * argv[])
{
  std::cout << "continental_ros_decoder_test_main_ars548.cpp" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_continental_decoder_test";

  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  continental_driver = std::make_shared<nebula::ros::ContinentalRosDecoderTest>(options, node_name);
  exec.add_node(continental_driver->get_node_base_interface());

  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Get Status");
  nebula::Status driver_status = continental_driver->get_status();
  int result = 0;
  if (driver_status == nebula::Status::OK) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Reading Started");
    result = RUN_ALL_TESTS();
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(node_name), driver_status);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Ending");
  continental_driver.reset();
  rclcpp::shutdown();

  return result;
}
