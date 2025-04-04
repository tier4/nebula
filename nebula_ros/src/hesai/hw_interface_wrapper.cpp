// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/hesai/hw_interface_wrapper.hpp"

#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/common/rclcpp_logger.hpp"

#include <nebula_common/util/string_conversions.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp>

#include <memory>

namespace nebula::ros
{

HesaiHwInterfaceWrapper::HesaiHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config, bool use_udp_only)
: hw_interface_(
    std::make_shared<drivers::HesaiHwInterface>(
      drivers::loggers::RclcppLogger(parent_node->get_logger()).child("HwInterface"))),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(Status::NOT_INITIALIZED),
  use_udp_only_(use_udp_only)
{
  setup_sensor_ = parent_node->declare_parameter<bool>("setup_sensor", param_read_only());
  bool retry_connect = parent_node->declare_parameter<bool>("retry_hw", param_read_only());

  status_ = hw_interface_->set_sensor_configuration(
    std::static_pointer_cast<const drivers::SensorConfigurationBase>(config));

  if (status_ != Status::OK) {
    throw std::runtime_error("Could not initialize HW interface: " + util::to_string(status_));
  }

  hw_interface_->set_target_model(config->sensor_model);

  if (use_udp_only) {
    // Do not initialize TCP
    return;
  }

  int retry_count = 0;

  while (true) {
    status_ = hw_interface_->initialize_tcp_driver();
    if (status_ == Status::OK || !retry_connect) {
      break;
    }

    retry_count++;
    std::this_thread::sleep_for(std::chrono::milliseconds(8000));  // >5000
    RCLCPP_WARN_STREAM(logger_, status_ << ". Retry #" << retry_count);
  }

  if (status_ == Status::OK) {
    try {
      auto inventory = hw_interface_->get_inventory();
      hw_interface_->set_target_model(inventory->model_number());
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get model from sensor...");
    }
    if (setup_sensor_) {
      hw_interface_->check_and_set_config();
    }
  } else {
    RCLCPP_ERROR_STREAM(
      logger_, "Failed to get model from sensor... Set from config: " << config->sensor_model);
  }

  status_ = Status::OK;
}

void HesaiHwInterfaceWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & new_config)
{
  hw_interface_->set_sensor_configuration(
    std::static_pointer_cast<const nebula::drivers::SensorConfigurationBase>(new_config));
  if (!use_udp_only_ && setup_sensor_) {
    hw_interface_->check_and_set_config();
  }
}

Status HesaiHwInterfaceWrapper::status()
{
  return status_;
}

std::shared_ptr<drivers::HesaiHwInterface> HesaiHwInterfaceWrapper::hw_interface() const
{
  return hw_interface_;
}

}  // namespace nebula::ros
