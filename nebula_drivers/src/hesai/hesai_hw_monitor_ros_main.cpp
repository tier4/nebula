#include "hesai/hesai_hw_monitor_ros_wrapper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_hesai_hw_monitor";

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto hesai_hw_monitor =
    std::make_shared<nebula::ros::HesaiHwMonitorRosWrapper>(options, node_name);
  exec.add_node(hesai_hw_monitor->get_node_base_interface());
  nebula::Status driver_status = hesai_hw_monitor->MonitorStart();
  if (driver_status == nebula::Status::OK) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Monitor Started");
    exec.spin();
  }

  rclcpp::shutdown();

  return 0;
}
