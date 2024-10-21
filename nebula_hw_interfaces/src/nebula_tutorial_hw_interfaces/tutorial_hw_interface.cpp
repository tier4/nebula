// Copyright 2024 TIER IV, Inc.

#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/tutorial_hw_interface.hpp"

#include <boost/asio.hpp>

namespace nebula::drivers
{
TutorialHwInterface::TutorialHwInterface(
  const std::shared_ptr<loggers::Logger> & logger,
  const std::shared_ptr<const TutorialSensorConfiguration> & sensor_configuration)
: logger_(logger),
  my_protocol_connection_(
    logger->child("MyProtocol"), sensor_configuration->host_ip, g_tcp_port,
    sensor_configuration->sensor_ip, g_tcp_port),
  udp_receiver_(
    logger->child("UDP"), sensor_configuration->host_ip, sensor_configuration->data_port,
    std::bind(&TutorialHwInterface::on_sensor_packet, this, std::placeholders::_1))
{
}

TutorialConfig TutorialHwInterface::get_config()
{
  return my_protocol_connection_.get<TutorialConfig>(g_ptc_command_get_config_info);
}

Status TutorialHwInterface::set_spin_rate(uint16_t rpm)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back((rpm >> 8u) & 0xffu);
  request_payload.emplace_back(rpm & 0xffu);

  my_protocol_connection_.set(g_ptc_command_set_spin_rate, request_payload);
  return Status::OK;
}

Status TutorialHwInterface::compare_and_send_config(const TutorialSensorConfiguration & updated)
{
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)

  // ////////////////////////////////////////
  // Retrieve current config from sensor
  // ////////////////////////////////////////

  auto current = get_config();

  // ////////////////////////////////////////
  // Compare and send updated motor RPM
  // ////////////////////////////////////////

  auto current_spin_rate = current.spin_rate;
  if (updated.rotation_speed != current_spin_rate.value()) {
    logger_->debug(
      "current spin rate (sensor): " + std::to_string(static_cast<int>(current_spin_rate.value())));
    logger_->debug("current spin rate (config): " + std::to_string(updated.rotation_speed));
    logger_->info("Updating sensor's spin rate to " + std::to_string(updated.rotation_speed));
    set_spin_rate(updated.rotation_speed);
  }

  return Status::OK;
}

}  // namespace nebula::drivers
