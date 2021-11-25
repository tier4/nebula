#include "hesai/hesai_driver_ros_wrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::string node_name = "nebula_hesai_driver";

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto hesai_driver = std::make_shared<nebula::ros::HesaiDriverRosWrapper>(options, node_name);
//  exec.add_node(hesai_driver->get_node_base_interface());
//  nebula::Status driver_status = hesai_driver->StreamStart();
//  if (driver_status == nebula::Status::OK) {
//    RCLCPP_INFO_STREAM(rclcpp::get_logger(node_name), "Hesai Driver Started");
//    exec.spin();
//  }

  rclcpp::shutdown();

  return 0;
}
