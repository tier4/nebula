// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/velodyne/hw_interface_wrapper.hpp"

#include <nebula_common/nebula_common.hpp>

#include <cassert>

namespace nebula::ros
{

VelodyneHwInterfaceWrapper::VelodyneHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config,
  drivers::ConnectionMode connection_mode)
: hw_interface_(new nebula::drivers::VelodyneHwInterface()),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(Status::NOT_INITIALIZED),
  connection_mode_(connection_mode)
{
  assert(connection_mode_ != drivers::ConnectionMode::OFFLINE);

  hw_interface_->set_logger(
    std::make_shared<rclcpp::Logger>(parent_node->get_logger().get_child("HwInterface")));
  status_ = hw_interface_->initialize_sensor_configuration(config);

  if (status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Could not initialize HW interface: " << status_).str());
  }

  if (connection_mode_ == drivers::ConnectionMode::UDP_ONLY) {
    // Do not initialize http client
    return;
  }

  status_ = hw_interface_->init_http_client();

  if (status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Could not initialize HTTP client: " << status_).str());
  }

  if (connection_mode_ == drivers::ConnectionMode::FULL) {
    RCLCPP_INFO_STREAM(logger_, "Setting sensor configuration");
    status_ = hw_interface_->set_sensor_configuration(config);
  }

  if (status_ != Status::OK) {
    throw std::runtime_error(
      (std::stringstream{} << "Could not set sensor configuration: " << status_).str());
  }

  status_ = Status::OK;
}

void VelodyneHwInterfaceWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & new_config)
{
  hw_interface_->initialize_sensor_configuration(new_config);
  if (connection_mode_ == drivers::ConnectionMode::UDP_ONLY) {
    return;
  }
  hw_interface_->init_http_client();
  if (connection_mode_ == drivers::ConnectionMode::FULL) {
    hw_interface_->set_sensor_configuration(new_config);
  }
}

Status VelodyneHwInterfaceWrapper::status()
{
  return status_;
}

std::shared_ptr<drivers::VelodyneHwInterface> VelodyneHwInterfaceWrapper::hw_interface() const
{
  return hw_interface_;
}

}  // namespace nebula::ros
