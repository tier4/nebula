#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "hesai/hesai_hw_interface_ros_wrapper.hpp"

int main(int argc, char * argv[])
{
  std::string node_name = "nebula_hesai_hw_interface";

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto hesai_driver = std::make_shared<nebula::ros::HesaiHwInterfaceWrapper>(
    options,
    node_name
  );
  exec.add_node(hesai_driver);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
