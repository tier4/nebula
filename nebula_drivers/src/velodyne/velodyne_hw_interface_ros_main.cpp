#include "velodyne/velodyne_hw_interface_ros_wrapper.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_velodyne_hw_interface";

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
//  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
//  std::cout << "exec.get_number_of_threads()=" << exec.get_number_of_threads() << std::endl;

  auto velodyne_hw_interface =
    std::make_shared<nebula::ros::VelodyneHwInterfaceRosWrapper>(options, node_name);
  exec.add_node(velodyne_hw_interface->get_node_base_interface());
//  velodyne_hw_interface->declare_parameter("warning1",0.0);
  nebula::Status driver_status = velodyne_hw_interface->StreamStart();
  //*
  if (driver_status == nebula::Status::OK) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "UDP Driver Started");
    exec.spin();
  }
  //*/
// rclcpp::spin(velodyne_hw_interface->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
