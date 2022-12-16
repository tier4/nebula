#include "velodyne/velodyne_driver_ros_wrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_velodyne_driver";

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto velodyne_driver = std::make_shared<nebula::ros::VelodyneDriverRosWrapper>(options);
  exec.add_node(velodyne_driver->get_node_base_interface());

  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Get Status");
  nebula::Status driver_status = velodyne_driver->GetStatus();
  if (driver_status == nebula::Status::OK) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Velodyne Driver Started");
    exec.spin();
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(node_name), driver_status);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Ending");
  rclcpp::shutdown();

  return 0;
}