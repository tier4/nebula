#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "ros2_driver_wrapper.hpp"

int main(int argc, char * argv[])
{
  std::string node_name = "lidar_driver_node";

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto lidar_driver = std::make_shared<lidar_driver::RosDriverWrapper>(
      options,
      node_name
  );  // Internally call StartStream() and StopStream() during constructor/destructor.
  exec.add_node(lidar_driver);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}