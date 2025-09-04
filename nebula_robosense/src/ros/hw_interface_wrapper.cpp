// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/robosense/hw_interface_wrapper.hpp"

#include "nebula_ros/common/rclcpp_logger.hpp"

#include <nebula_common/util/string_conversions.hpp>

#include <memory>

namespace nebula::ros
{
RobosenseHwInterfaceWrapper::RobosenseHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & config)
: hw_interface_(
    std::make_shared<drivers::RobosenseHwInterface>(
      drivers::loggers::RclcppLogger(parent_node->get_logger()).child("HwInterface"))),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(nebula::Status::NOT_INITIALIZED)
{
  status_ = hw_interface_->set_sensor_configuration(config);

  if (Status::OK != status_) {
    throw std::runtime_error("Sensor configuration invalid: " + util::to_string(status_));
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

}  // namespace nebula::ros
