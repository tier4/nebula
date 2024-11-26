// Copyright 2023-2024 Map IV, Inc.
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

#include "hesai/hesai_ros_offline_extract_bag_decode.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

/**
 * @file hesai_ros_offline_extract_bag_decode.cpp
 * @brief Main script to decode Hesai LiDAR packets from a ROS bag and write to a new bag
 *
 * This script reads a ROS bag containing Hesai LiDAR packets, decodes them into standard
 * sensor_msgs::pointcloud point cloud data, and writes the decoded data to a new ROS bag. It uses
 * the HesaiRosOfflineExtractBagDecode class to handle the packet decoding and bag read/write
 * operations.
 *
 * The script performs the following steps:
 * 1. Initializes ROS node and executor
 * 2. Creates an instance of HesaiRosOfflineExtractBagDecode
 * 3. Checks driver status
 * 4. If status is OK, reads and processes the input bag
 * 5. Shuts down ROS node after processing completes
 */

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_hesai_offline_extraction_with_bag";

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto hesai_driver =
    std::make_shared<nebula::ros::HesaiRosOfflineExtractBagDecode>(options, node_name);
  exec.add_node(hesai_driver->get_node_base_interface());

  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Get Status");
  nebula::Status driver_status = hesai_driver->get_status();
  if (driver_status == nebula::Status::OK) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Reading Started");
    driver_status = hesai_driver->read_bag();
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(node_name), driver_status);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Ending");
  rclcpp::shutdown();

  return 0;
}
