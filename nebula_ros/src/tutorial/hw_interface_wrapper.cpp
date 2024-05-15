#include "nebula_ros/tutorial/hw_interface_wrapper.hpp"

namespace nebula
{
namespace ros
{

TutorialHwInterfaceWrapper::TutorialHwInterfaceWrapper(
    rclcpp::Node* const parent_node, std::shared_ptr<const TutorialSensorConfiguration>& config)
  : hw_interface_(new TutorialHwInterface())
  , logger_(parent_node->get_logger().get_child("HwInterface"))
  , status_(Status::NOT_INITIALIZED)
{
  setup_sensor_ = parent_node->declare_parameter<bool>("setup_sensor", true, param_read_only());
  bool retry_connect = parent_node->declare_parameter<bool>("retry_hw", true, param_read_only());

  status_ = hw_interface_->SetSensorConfiguration(config);

  if (status_ != Status::OK) {
    throw std::runtime_error((std::stringstream{} << "Could not initialize HW interface: " << status_).str());
  }

  hw_interface_->SetLogger(std::make_shared<rclcpp::Logger>(parent_node->get_logger()));
  hw_interface_->SetTargetModel(config->sensor_model);

  int retry_count = 0;

  while (true)
  {
    status_ = hw_interface_->InitializeTcpDriver();
    if (status_ == Status::OK || !retry_connect)
    {
      break;
    }

    retry_count++;
    std::this_thread::sleep_for(std::chrono::milliseconds(8000));  // >5000
    RCLCPP_WARN_STREAM(logger_, status_ << ". Retry #" << retry_count);
  }

  if (status_ == Status::OK)
  {
    try
    {
      auto inventory = hw_interface_->GetInventory();
      RCLCPP_INFO_STREAM(logger_, inventory);
      hw_interface_->SetTargetModel(inventory.model);
    }
    catch (...)
    {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get model from sensor...");
    }
    if (setup_sensor_)
    {
      hw_interface_->CheckAndSetConfig();
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to get model from sensor... Set from config: " << config->sensor_model);
  }

  status_ = Status::OK;
}

void TutorialHwInterfaceWrapper::OnConfigChange(
    const std::shared_ptr<const TutorialSensorConfiguration>& new_config)
{
  hw_interface_->SetSensorConfiguration(new_config);
  if (setup_sensor_) {
    hw_interface_->CheckAndSetConfig();
  }
}

Status TutorialHwInterfaceWrapper::Status()
{
  return status_;
}

std::shared_ptr<TutorialHwInterface> TutorialHwInterfaceWrapper::HwInterface() const
{
  return hw_interface_;
}

}  // namespace ros
}  // namespace nebula