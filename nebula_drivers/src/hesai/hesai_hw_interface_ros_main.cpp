#include "hesai/hesai_hw_interface_ros_wrapper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_hesai_hw_interface";

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto hesai_hw_interface =
    std::make_shared<nebula::ros::HesaiHwInterfaceRosWrapper>(options, node_name);
  exec.add_node(hesai_hw_interface->get_node_base_interface());
  nebula::Status driver_status = hesai_hw_interface->StreamStart();
  if (driver_status == nebula::Status::OK) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "UDP Driver Started");
    exec.spin();
  }

  rclcpp::shutdown();

  return 0;
}
