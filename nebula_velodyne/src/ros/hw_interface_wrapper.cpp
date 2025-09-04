// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/velodyne/hw_interface_wrapper.hpp"

#include "nebula_ros/common/rclcpp_logger.hpp"

#include <nebula_common/util/string_conversions.hpp>

#include <memory>

namespace nebula::ros
{

VelodyneHwInterfaceWrapper::VelodyneHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config, bool use_udp_only)
: hw_interface_(
    std::make_shared<drivers::VelodyneHwInterface>(
      drivers::loggers::RclcppLogger(parent_node->get_logger()).child("HwInterface"))),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(Status::NOT_INITIALIZED),
  use_udp_only_(use_udp_only)
{
  setup_sensor_ = parent_node->declare_parameter<bool>("setup_sensor", param_read_only());

  status_ = hw_interface_->initialize_sensor_configuration(config);

  if (status_ != Status::OK) {
    throw std::runtime_error("Could not initialize HW interface: " + util::to_string(status_));
  }

  if (use_udp_only_) {
    // Do not initialize http client
    return;
  }

  status_ = hw_interface_->init_http_client();

  if (status_ != Status::OK) {
    throw std::runtime_error("Could not initialize HTTP client: " + util::to_string(status_));
  }

  if (setup_sensor_) {
    RCLCPP_INFO_STREAM(logger_, "Setting sensor configuration");
    status_ = hw_interface_->set_sensor_configuration(config);
  }

  if (status_ != Status::OK) {
    throw std::runtime_error("Could not set sensor configuration: " + util::to_string(status_));
  }

  status_ = Status::OK;
}

void VelodyneHwInterfaceWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & new_config)
{
  hw_interface_->initialize_sensor_configuration(new_config);
  if (use_udp_only_) {
    return;
  }
  hw_interface_->init_http_client();
  if (setup_sensor_) {
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
