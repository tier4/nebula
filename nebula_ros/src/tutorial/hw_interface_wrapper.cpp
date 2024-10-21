// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/tutorial/hw_interface_wrapper.hpp"

#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/tutorial/loggers/ros_logger.hpp"
namespace nebula::ros
{

TutorialHwInterfaceWrapper::TutorialHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & config)
: hw_interface_(std::make_shared<nebula::drivers::TutorialHwInterface>(
    std::static_pointer_cast<nebula::drivers::loggers::Logger>(
      std::make_shared<loggers::RosLogger>(parent_node->get_logger().get_child("HwInterface"), "")),
    config)),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(Status::NOT_INITIALIZED)
{
  // This parameter allows for two common workflows:
  // 1. Setting up the sensor using an external tool (web interface etc.)
  //    and not changing settings during runtime
  // 2. Setting up and running the sensor using Nebula
  setup_sensor_ = parent_node->declare_parameter<bool>("setup_sensor", param_read_only());

  if (setup_sensor_) {
    status_ = hw_interface_->compare_and_send_config(*config);
  }

  if (status_ != Status::OK) {
    throw std::runtime_error((std::stringstream{} << "Could not set up sensor: " << status_).str());
  }
}

void TutorialHwInterfaceWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & new_config)
{
  if (setup_sensor_) {
    status_ = hw_interface_->compare_and_send_config(*new_config);
  }

  if (status_ != Status::OK) {
    throw std::runtime_error((std::stringstream{} << "Could not set up sensor: " << status_).str());
  }
}

Status TutorialHwInterfaceWrapper::status()
{
  return status_;
}

std::shared_ptr<nebula::drivers::TutorialHwInterface> TutorialHwInterfaceWrapper::hw_interface()
  const
{
  return hw_interface_;
}

}  // namespace nebula::ros
