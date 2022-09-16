#include "velodyne/velodyne_hw_monitor_ros_wrapper.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_velodyne_hw_monitor";

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
//  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
//  std::cout << "exec.get_number_of_threads()=" << exec.get_number_of_threads() << std::endl;

  auto velodyne_hw_monitor =
    std::make_shared<nebula::ros::VelodyneHwMonitorRosWrapper>(options, node_name);
  //*
  exec.add_node(velodyne_hw_monitor->get_node_base_interface());
//  velodyne_hw_interface->declare_parameter("warning1",0.0);
  nebula::Status driver_status = velodyne_hw_monitor->MonitorStart();
  if (driver_status == nebula::Status::OK) {
//  if (true) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Monitor Started");
    exec.spin();
  }
  //*/
// rclcpp::spin(velodyne_hw_monitor->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
