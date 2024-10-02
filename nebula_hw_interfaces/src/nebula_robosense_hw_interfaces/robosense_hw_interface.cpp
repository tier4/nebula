// Copyright 2024 TIER IV, Inc.

#include "nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp"
namespace nebula
{
namespace drivers
{
RobosenseHwInterface::RobosenseHwInterface()
: cloud_io_context_(new ::drivers::common::IoContext(1)),
  info_io_context_(new ::drivers::common::IoContext(1)),
  cloud_udp_driver_(new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)),
  info_udp_driver_(new ::drivers::udp_driver::UdpDriver(*info_io_context_))
{
}

void RobosenseHwInterface::receive_sensor_packet_callback(std::vector<uint8_t> & buffer)
{
  if (!scan_reception_callback_) {
    return;
  }

  scan_reception_callback_(buffer);
}

void RobosenseHwInterface::receive_info_packet_callback(std::vector<uint8_t> & buffer)
{
  if (!info_reception_callback_) {
    return;
  }

  info_reception_callback_(buffer);
}

Status RobosenseHwInterface::sensor_interface_start()
{
  try {
    std::cout << "Starting UDP server for data packets on: " << *sensor_configuration_ << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();

    cloud_udp_driver_->receiver()->asyncReceive(std::bind(
      &RobosenseHwInterface::receive_sensor_packet_callback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status RobosenseHwInterface::info_interface_start()
{
  try {
    std::cout << "Starting UDP server for info packets on: " << *sensor_configuration_ << std::endl;
    print_info(
      "Starting UDP server for info packets on: " + sensor_configuration_->sensor_ip + ": " +
      std::to_string(sensor_configuration_->gnss_port));
    info_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->gnss_port);
    info_udp_driver_->receiver()->open();
    info_udp_driver_->receiver()->bind();

    info_udp_driver_->receiver()->asyncReceive(
      std::bind(&RobosenseHwInterface::receive_info_packet_callback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->gnss_port << std::endl;
    return status;
  }

  return Status::OK;
}

Status RobosenseHwInterface::set_sensor_configuration(
  std::shared_ptr<const RobosenseSensorConfiguration> sensor_configuration)
{
  if (!(sensor_configuration->sensor_model == SensorModel::ROBOSENSE_BPEARL_V3 ||
        sensor_configuration->sensor_model == SensorModel::ROBOSENSE_BPEARL_V4 ||
        sensor_configuration->sensor_model == SensorModel::ROBOSENSE_HELIOS)) {
    return Status::INVALID_SENSOR_MODEL;
  }

  sensor_configuration_ = sensor_configuration;

  return Status::OK;
}

Status RobosenseHwInterface::register_scan_callback(
  std::function<void(std::vector<uint8_t> &)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
  return Status::OK;
}

Status RobosenseHwInterface::register_info_callback(
  std::function<void(std::vector<uint8_t> &)> info_callback)
{
  info_reception_callback_ = std::move(info_callback);
  return Status::OK;
}

void RobosenseHwInterface::print_debug(std::string debug)
{
  if (parent_node_logger_) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger_), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

void RobosenseHwInterface::print_info(std::string info)
{
  if (parent_node_logger_) {
    RCLCPP_INFO_STREAM((*parent_node_logger_), info);
  } else {
    std::cout << info << std::endl;
  }
}

void RobosenseHwInterface::set_logger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger_ = logger;
}

}  // namespace drivers
}  // namespace nebula
