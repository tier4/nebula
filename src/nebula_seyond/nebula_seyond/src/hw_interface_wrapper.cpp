// Copyright 2026 TIER IV, Inc.

#include "nebula_seyond/hw_interface_wrapper.hpp"

#include <memory>
#include <stdexcept>

namespace nebula::ros
{

SeyondHwInterfaceWrapper::SeyondHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<const nebula::drivers::SeyondSensorConfiguration> & config)
: config_(config),
  calibration_(std::make_shared<nebula::drivers::SeyondCalibrationData>()),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(Status::NOT_INITIALIZED)
{
  if (!config_) {
    throw std::runtime_error("SeyondHwInterfaceWrapper requires a valid sensor configuration");
  }

  hw_interface_ = std::make_shared<nebula::drivers::SeyondHwInterface>(*config_);

  if (config_->setup_sensor) {
    status_ = hw_interface_->setup_sensor(*config_);
    if (status_ != Status::OK) {
      throw std::runtime_error("Failed to configure Seyond sensor");
    }
  }

  auto calibration_or_error = hw_interface_->get_calibration();
  if (calibration_or_error.has_value()) {
    *calibration_ = calibration_or_error.value();
  } else {
    RCLCPP_WARN(logger_, "Failed to fetch calibration from sensor. Using defaults.");
  }

  status_ = Status::OK;
}

void SeyondHwInterfaceWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::SeyondSensorConfiguration> & new_config)
{
  config_ = new_config;
  if (!config_) {
    status_ = Status::SENSOR_CONFIG_ERROR;
    return;
  }

  if (config_->setup_sensor) {
    status_ = hw_interface_->setup_sensor(*config_);
    if (status_ != Status::OK) {
      return;
    }
  }

  auto calibration_or_error = hw_interface_->get_calibration();
  if (calibration_or_error.has_value()) {
    *calibration_ = calibration_or_error.value();
  }

  status_ = Status::OK;
}

nebula::Status SeyondHwInterfaceWrapper::status() const
{
  return status_;
}

std::shared_ptr<nebula::drivers::SeyondHwInterface> SeyondHwInterfaceWrapper::hw_interface() const
{
  return hw_interface_;
}

std::shared_ptr<const nebula::drivers::SeyondCalibrationData>
SeyondHwInterfaceWrapper::calibration() const
{
  return calibration_;
}

}  // namespace nebula::ros
