// Copyright 2024 TIER IV, Inc.

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
  hw_interface_->set_logger(
    std::make_shared<rclcpp::Logger>(parent_node->get_logger().get_child("HwInterface")));
  status_ = hw_interface_->set_sensor_configuration(config);

  if (Status::OK != status_) {
    throw std::runtime_error(
      (std::stringstream{} << "Sensor configuration invalid: " << status_).str());
  }
}

void RobosenseHwInterfaceWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & new_config)
{
  hw_interface_->set_sensor_configuration(new_config);
}

nebula::Status RobosenseHwInterfaceWrapper::status()
{
  return status_;
}

std::shared_ptr<drivers::RobosenseHwInterface> RobosenseHwInterfaceWrapper::hw_interface() const
{
  return hw_interface_;
}

}  // namespace ros
}  // namespace nebula
