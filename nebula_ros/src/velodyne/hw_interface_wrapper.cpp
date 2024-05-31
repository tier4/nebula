#include "nebula_ros/velodyne/hw_interface_wrapper.hpp"

namespace nebula
{
namespace ros
{

VelodyneHwInterfaceWrapper::VelodyneHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config)
: hw_interface_(new nebula::drivers::VelodyneHwInterface()),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(Status::NOT_INITIALIZED)
{
  setup_sensor_ = parent_node->declare_parameter<bool>("setup_sensor", param_read_only());

  hw_interface_->SetLogger(
    std::make_shared<rclcpp::Logger>(parent_node->get_logger().get_child("HwInterface")));
  status_ = hw_interface_->InitializeSensorConfiguration(config);

  if (status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Could not initialize HW interface: " << status_).str());
  }

  status_ = hw_interface_->InitHttpClient();

  if (status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Could not initialize HTTP client: " << status_).str());
  }

  if (setup_sensor_) {
    RCLCPP_INFO_STREAM(logger_, "Setting sensor configuration");
    status_ = hw_interface_->SetSensorConfiguration(config);
  }

  if (status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Could not set sensor configuration: " << status_).str());
  }

  status_ = Status::OK;
}

void VelodyneHwInterfaceWrapper::OnConfigChange(
  const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & new_config)
{
  hw_interface_->InitializeSensorConfiguration(new_config);
  hw_interface_->InitHttpClient();
  if (setup_sensor_) {
    hw_interface_->SetSensorConfiguration(new_config);
  }
}

Status VelodyneHwInterfaceWrapper::Status()
{
  return status_;
}

std::shared_ptr<drivers::VelodyneHwInterface> VelodyneHwInterfaceWrapper::HwInterface() const
{
  return hw_interface_;
}

}  // namespace ros
}  // namespace nebula
