#include "nebula_ros/robosense/hw_interface_wrapper.hpp"

namespace nebula
{
namespace ros
{
RobosenseHwInterfaceWrapper::RobosenseHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & config)
: hw_interface_(new nebula::drivers::RobosenseHwInterface()),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(nebula::Status::NOT_INITIALIZED)
{
  hw_interface_->SetLogger(
    std::make_shared<rclcpp::Logger>(parent_node->get_logger().get_child("HwInterface")));
  status_ = hw_interface_->SetSensorConfiguration(config);

  if (Status::OK != status_) {
    throw std::runtime_error(
      (std::stringstream{} << "Sensor configuration invalid: " << status_).str());
  }
}

void RobosenseHwInterfaceWrapper::OnConfigChange(
  const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & new_config)
{
  hw_interface_->SetSensorConfiguration(new_config);
}

nebula::Status RobosenseHwInterfaceWrapper::Status()
{
  return status_;
}

std::shared_ptr<drivers::RobosenseHwInterface> RobosenseHwInterfaceWrapper::HwInterface() const
{
  return hw_interface_;
}

}  // namespace ros
}  // namespace nebula
