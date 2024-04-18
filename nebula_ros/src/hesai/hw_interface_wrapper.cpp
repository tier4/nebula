#include "nebula_ros/hesai/hw_interface_wrapper.hpp"

namespace nebula
{
namespace ros
{

HesaiHwInterfaceWrapper::HesaiHwInterfaceWrapper(rclcpp::Node* const parent_node,
                                                 std::shared_ptr<nebula::drivers::HesaiSensorConfiguration>& config)
  : hw_interface_(new nebula::drivers::HesaiHwInterface())
  , logger_(parent_node->get_logger().get_child("HwInterface"))
  , status_(Status::NOT_INITIALIZED)
{
  bool setup_sensor, retry_connect;

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    parent_node->declare_parameter<bool>("setup_sensor", true, descriptor);
    setup_sensor = parent_node->get_parameter("setup_sensor").as_bool();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    parent_node->declare_parameter<bool>("retry_hw", true, descriptor);
    retry_connect = parent_node->get_parameter("retry_hw").as_bool();
  }

  hw_interface_->SetSensorConfiguration(std::static_pointer_cast<drivers::SensorConfigurationBase>(config));

  status_ = Status::OK;

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
    RCLCPP_WARN_STREAM(logger_, "Retry: " << retry_count);
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
    if (setup_sensor)
    {
      hw_interface_->CheckAndSetConfig();
      // updateParameters(); TODO
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to get model from sensor... Set from config: " << config->sensor_model);
  }

  status_ = Status::OK;
}

Status HesaiHwInterfaceWrapper::Status()
{
  return status_;
}

std::shared_ptr<drivers::HesaiHwInterface> HesaiHwInterfaceWrapper::HwInterface() const
{
  return hw_interface_;
}

}  // namespace ros
}  // namespace nebula