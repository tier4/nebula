// Copyright 2024 TIER IV, Inc.

#include "nebula_hesai/hw_interface_wrapper.hpp"

#include "nebula_core_ros/cie_thread_factory.hpp"
#include "nebula_core_ros/parameter_descriptors.hpp"
#include "nebula_core_ros/rclcpp_logger.hpp"
#include "nebula_hesai_hw_interfaces/hesai_cmd_response.hpp"

#include <nebula_core_common/util/string_conversions.hpp>
#include <nebula_hesai_hw_interfaces/hesai_hw_interface.hpp>

#include <chrono>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace nebula::ros
{

namespace
{
/// @brief Back-off between hardware bring-up attempts (TCP connect and sensor configuration).
/// The sensor can take more than 5s to become responsive after power-on, so this is generous.
constexpr auto g_hw_retry_delay = std::chrono::milliseconds(8000);
/// @brief Maximum number of sensor-configuration attempts when retry_hw is enabled.
constexpr int g_hw_config_max_attempts = 5;
}  // namespace

HesaiHwInterfaceWrapper::HesaiHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config, bool use_udp_only)
: hw_interface_(
    std::make_shared<drivers::HesaiHwInterface>(
      drivers::loggers::RclcppLogger(parent_node->get_logger()).child("HwInterface"),
      make_cie_thread_factory("nebula_hesai_udp_receiver@" + config->frame_id))),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(Status::NOT_INITIALIZED),
  use_udp_only_(use_udp_only)
{
  setup_sensor_ = parent_node->declare_parameter<bool>("setup_sensor", param_read_only());
  retry_hw_ = parent_node->declare_parameter<bool>("retry_hw", param_read_only());

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
    status_ = hw_interface_->initialize_tcp_socket();
    if (status_ == Status::OK || !retry_hw_) {
      break;
    }

    retry_count++;
    std::this_thread::sleep_for(g_hw_retry_delay);
    RCLCPP_WARN_STREAM(logger_, status_ << ". Retry #" << retry_count);
  }

  if (status_ == Status::OK) {
    try {
      inventory_ = hw_interface_->get_inventory();
      hw_interface_->set_target_model(inventory_->model_number());
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get model from sensor...");
    }
    if (setup_sensor_) {
      configure_sensor();
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
    // Unlike startup, a runtime reconfiguration failure must not bring down a running node (this
    // runs inside a set-parameters callback), so the node is kept alive. Note that
    // check_and_set_config applies settings incrementally, so a mid-sequence failure can leave the
    // sensor partially reconfigured rather than at either the old or the new configuration.
    try {
      configure_sensor();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Could not apply new configuration; the sensor may be left in a partially reconfigured "
        "state: "
          << e.what());
    }
  }
}

void HesaiHwInterfaceWrapper::configure_sensor()
{
  // A transient comms fault (e.g. a slow sensor timing out a single PTC command) should not fail
  // setup outright, so attempts are retried. When retry_hw is set, a bounded number of retries
  // gives a slow-to-respond sensor time to become ready. If configuration still cannot be applied,
  // we throw rather than run a misconfigured sensor; the caller decides whether that is fatal.
  const int attempts = retry_hw_ ? g_hw_config_max_attempts : 1;
  for (int attempt = 1; attempt <= attempts; ++attempt) {
    try {
      hw_interface_->check_and_set_config();
      return;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Could not configure sensor (attempt " << attempt << "/" << attempts << "): " << e.what());
    }
    if (attempt < attempts) {
      std::this_thread::sleep_for(g_hw_retry_delay);
    }
  }

  throw std::runtime_error(
    "Could not configure sensor after " + std::to_string(attempts) + " attempt(s)");
}

Status HesaiHwInterfaceWrapper::status()
{
  return status_;
}

std::shared_ptr<drivers::HesaiHwInterface> HesaiHwInterfaceWrapper::hw_interface() const
{
  return hw_interface_;
}

std::shared_ptr<const HesaiInventoryBase> HesaiHwInterfaceWrapper::inventory() const
{
  return inventory_;
}

}  // namespace nebula::ros
