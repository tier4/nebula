#include "nebula_ros/tutorial/hw_interface_wrapper.hpp"

namespace nebula
{
namespace ros
{

TutorialHwInterfaceWrapper::TutorialHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & config)
: hw_interface_(new nebula::drivers::TutorialHwInterface(
    std::static_pointer_cast<nebula::drivers::loggers::Logger>(
      std::make_shared<loggers::RosLogger>(parent_node->get_logger().get_child("HwInterface"), "")),
    config)),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(Status::NOT_INITIALIZED)
{
  setup_sensor_ = parent_node->declare_parameter<bool>("setup_sensor", param_read_only());

  if (setup_sensor_) {
    status_ = hw_interface_->compareAndSendConfig(*config);
  }

  if (status_ != Status::OK) {
    throw std::runtime_error((std::stringstream{} << "Could not set up sensor: " << status_).str());
  }
}

void TutorialHwInterfaceWrapper::OnConfigChange(
  const std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & new_config)
{
  if (setup_sensor_) {
    status_ = hw_interface_->compareAndSendConfig(*new_config);
  }

  if (status_ != Status::OK) {
    throw std::runtime_error((std::stringstream{} << "Could not set up sensor: " << status_).str());
  }
}

Status TutorialHwInterfaceWrapper::Status()
{
  return status_;
}

std::shared_ptr<nebula::drivers::TutorialHwInterface> TutorialHwInterfaceWrapper::HwInterface()
  const
{
  return hw_interface_;
}

}  // namespace ros
}  // namespace nebula