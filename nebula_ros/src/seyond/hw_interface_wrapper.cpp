#include "nebula_ros/seyond/hw_interface_wrapper.hpp"

namespace nebula
{
namespace ros
{

SeyondHwInterfaceWrapper::SeyondHwInterfaceWrapper(
  rclcpp::Node * const parent_node, std::shared_ptr<const SeyondSensorConfiguration> & config)
: hw_interface_(new SeyondHwInterface()),
  logger_(parent_node->get_logger().get_child("HwInterface")),
  status_(Status::NOT_INITIALIZED)
{
  setup_sensor_ = parent_node->declare_parameter<bool>("setup_sensor", true, param_read_only());
  bool retry_connect = parent_node->declare_parameter<bool>("retry_hw", true, param_read_only());

  status_ = hw_interface_->SetSensorConfiguration(config);

  if (status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Could not initialize HW interface: " << status_).str());
  }

  hw_interface_->SetLogger(std::make_shared<rclcpp::Logger>(parent_node->get_logger()));
  // hw_interface_->SetTargetModel(config->sensor_model);

  int retry_count = 0;

  // NOTE: for when TP interface is implemented
  // while (true)
  // {
  //   status_ = hw_interface_->InitializeTcpDriver();
  //   if (status_ == Status::OK || !retry_connect)
  //   {
  //     break;
  //   }

  //   retry_count++;
  //   std::this_thread::sleep_for(std::chrono::milliseconds(8000));  // >5000
  //   RCLCPP_WARN_STREAM(logger_, status_ << ". Retry #" << retry_count);
  // }

  status_ = Status::OK;
}

void SeyondHwInterfaceWrapper::OnConfigChange(
  const std::shared_ptr<const SeyondSensorConfiguration> & new_config)
{
  hw_interface_->SetSensorConfiguration(new_config);
  // if (setup_sensor_) {
  //   hw_interface_->CheckAndSetConfig();
  // }
}

Status SeyondHwInterfaceWrapper::Status()
{
  return status_;
}

std::shared_ptr<SeyondHwInterface> SeyondHwInterfaceWrapper::HwInterface() const
{
  return hw_interface_;
}

}  // namespace ros
}  // namespace nebula
