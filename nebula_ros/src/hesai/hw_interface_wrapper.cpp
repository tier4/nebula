// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/hesai/hw_interface_wrapper.hpp"

#include "nebula_ros/common/parameter_descriptors.hpp"

namespace nebula
{
namespace ros
{

HesaiHwInterfaceWrapper::HesaiHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config)
: hw_interface_(new nebula::drivers::HesaiHwInterface()),
  logger_(parent_node->get_logger().get_child("HwInterface")),
  status_(Status::NOT_INITIALIZED)
{
  setup_sensor_ = parent_node->declare_parameter<bool>("setup_sensor", param_read_only());
  bool retry_connect = parent_node->declare_parameter<bool>("retry_hw", param_read_only());

  status_ = hw_interface_->SetSensorConfiguration(
    std::static_pointer_cast<const drivers::SensorConfigurationBase>(config));

  if (status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Could not initialize HW interface: " << status_).str());
  }

  hw_interface_->SetLogger(std::make_shared<rclcpp::Logger>(parent_node->get_logger()));
  hw_interface_->SetTargetModel(config->sensor_model);

  int retry_count = 0;

  while (true) {
    status_ = hw_interface_->InitializeTcpDriver();
    if (status_ == Status::OK || !retry_connect) {
      break;
    }

    retry_count++;
    std::this_thread::sleep_for(std::chrono::milliseconds(8000));  // >5000
    RCLCPP_WARN_STREAM(logger_, status_ << ". Retry #" << retry_count);
  }

  if (status_ == Status::OK) {
    try {
      auto inventory = hw_interface_->GetInventory();
      RCLCPP_INFO_STREAM(logger_, inventory);
      hw_interface_->SetTargetModel(inventory.model);
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get model from sensor...");
    }
    if (setup_sensor_) {
      hw_interface_->CheckAndSetConfig();
    }
  } else {
    RCLCPP_ERROR_STREAM(
      logger_, "Failed to get model from sensor... Set from config: " << config->sensor_model);
  }

  status_ = Status::OK;
}

void HesaiHwInterfaceWrapper::OnConfigChange(
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & new_config)
{
  hw_interface_->SetSensorConfiguration(
    std::static_pointer_cast<const nebula::drivers::SensorConfigurationBase>(new_config));
  if (setup_sensor_) {
    hw_interface_->CheckAndSetConfig();
  }
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
