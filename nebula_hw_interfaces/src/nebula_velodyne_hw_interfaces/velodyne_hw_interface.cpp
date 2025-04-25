// Copyright 2024 TIER IV, Inc.

#include "nebula_hw_interfaces/nebula_hw_interfaces_velodyne/velodyne_hw_interface.hpp"

#include "nebula_common/util/string_conversions.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{
using std::string_literals::operator""s;

VelodyneHwInterface::VelodyneHwInterface(const std::shared_ptr<loggers::Logger> & logger)
: boost_ctx_{new boost::asio::io_context()},
  http_client_driver_{new ::drivers::tcp_driver::HttpClientDriver(boost_ctx_)},
  logger_{logger}
{
}

nebula::util::expected<std::string, VelodyneStatus> VelodyneHwInterface::http_get_request(
  const std::string & endpoint)
{
  std::lock_guard lock(mtx_inflight_request_);
  auto do_request = [this, &endpoint]() { return http_client_driver_->get(endpoint); };
  return do_http_request_with_retries(do_request, http_client_driver_);
}

nebula::util::expected<std::string, VelodyneStatus> VelodyneHwInterface::http_post_request(
  const std::string & endpoint, const std::string & body)
{
  std::lock_guard lock(mtx_inflight_request_);
  auto do_request = [this, &endpoint, &body]() {
    return http_client_driver_->post(endpoint, body);
  };
  return do_http_request_with_retries(do_request, http_client_driver_);
}

Status VelodyneHwInterface::initialize_sensor_configuration(
  std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration)
{
  sensor_configuration_ = sensor_configuration;
  return Status::OK;
}

Status VelodyneHwInterface::set_sensor_configuration(
  std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration)
{
  auto snapshot = get_snapshot();
  if (!snapshot.has_value()) {
    return snapshot.error();
  }
  auto tree = parse_json(snapshot.value());
  VelodyneStatus status = check_and_set_config(sensor_configuration, tree);

  return status;
}

Status VelodyneHwInterface::sensor_interface_start()
{
  auto builder = connections::UdpSocket::Builder(
    sensor_configuration_->host_ip, sensor_configuration_->data_port);

  udp_socket_.emplace(std::move(builder).bind());
  udp_socket_->subscribe([&](
                           const std::vector<uint8_t> & packet,
                           const connections::UdpSocket::RxMetadata & /* metadata */) {
    receive_sensor_packet_callback(packet);
  });

  return Status::OK;
}

Status VelodyneHwInterface::register_scan_callback(
  std::function<void(const std::vector<uint8_t> & packet)> scan_callback)
{
  cloud_packet_callback_ = std::move(scan_callback);
  return Status::OK;
}

void VelodyneHwInterface::receive_sensor_packet_callback(const std::vector<uint8_t> & buffer)
{
  if (!cloud_packet_callback_) {
    return;
  }

  cloud_packet_callback_(buffer);
}
Status VelodyneHwInterface::sensor_interface_stop()
{
  if (udp_socket_) {
    udp_socket_->unsubscribe();
  }
  return Status::OK;
}

Status VelodyneHwInterface::get_sensor_configuration(SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  logger_->debug(ss.str());
  return Status::ERROR_1;
}

VelodyneStatus VelodyneHwInterface::init_http_client()
{
  try {
    http_client_driver_->init_client(sensor_configuration_->sensor_ip, 80);
    if (!http_client_driver_->client()->isOpen()) {
      http_client_driver_->client()->open();
    }
  } catch (const std::exception & ex) {
    VelodyneStatus status = Status::HTTP_CONNECTION_ERROR;
    return status;
  }
  return Status::OK;
}

void VelodyneHwInterface::string_callback(const std::string & str)
{
  logger_->debug("VelodyneHwInterface::string_callback: " + str);
}

boost::property_tree::ptree VelodyneHwInterface::parse_json(const std::string & str)
{
  boost::property_tree::ptree tree;
  try {
    std::stringstream ss;
    ss << str;
    boost::property_tree::read_json(ss, tree);
  } catch (boost::property_tree::json_parser_error & e) {
    logger_->error("Error on ParseJson: "s + e.what());
  }
  return tree;
}

VelodyneStatus VelodyneHwInterface::check_and_set_config(
  std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration,
  boost::property_tree::ptree tree)
{
  VelodyneStatus status;
  const auto & ok = VelodyneStatus::OK;

  std::string target_key = "config.returns";
  auto current_return_mode_str = tree.get<std::string>(target_key);
  auto current_return_mode =
    nebula::drivers::return_mode_from_string_velodyne(tree.get<std::string>(target_key));
  if (sensor_configuration->return_mode != current_return_mode) {
    status = set_return_type(sensor_configuration->return_mode);
    if (status != ok) return status;

    logger_->debug(
      "VelodyneHwInterface::parse_json(" + target_key + "): " + current_return_mode_str);
    logger_->debug("current_return_mode: " + util::to_string(current_return_mode));
    logger_->debug(
      "sensor_configuration->return_mode: " + util::to_string(sensor_configuration->return_mode));
  }

  target_key = "config.rpm";
  auto current_rotation_speed = tree.get<u_int16_t>(target_key);
  if (sensor_configuration->rotation_speed != current_rotation_speed) {
    status = set_rpm(sensor_configuration->rotation_speed);
    if (status != ok) return status;

    logger_->debug(
      "VelodyneHwInterface::parse_json(" + target_key +
      "): " + std::to_string(current_rotation_speed));
    logger_->debug(
      "sensor_configuration->rotation_speed: " +
      std::to_string(sensor_configuration->rotation_speed));
  }

  target_key = "config.fov.start";
  auto current_cloud_min_angle = tree.get<std::uint16_t>(target_key);
  int setting_cloud_min_angle = sensor_configuration->cloud_min_angle;
  // Velodyne only allows a maximum of 359 in the setting
  if (setting_cloud_min_angle == 360) {
    setting_cloud_min_angle = 359;
  }
  if (setting_cloud_min_angle != current_cloud_min_angle) {
    status = set_fov_start(setting_cloud_min_angle);
    if (status != ok) return status;

    logger_->debug(
      "VelodyneHwInterface::parse_json(" + target_key +
      "): " + std::to_string(current_cloud_min_angle));
    logger_->debug(
      "sensor_configuration->cloud_min_angle: " + std::to_string(setting_cloud_min_angle));
  }

  target_key = "config.fov.end";
  auto current_cloud_max_angle = tree.get<std::uint16_t>(target_key);
  int setting_cloud_max_angle = sensor_configuration->cloud_max_angle;
  // Velodyne only allows a maximum of 359 in the setting
  if (setting_cloud_max_angle == 360) {
    setting_cloud_max_angle = 359;
  }
  if (setting_cloud_max_angle != current_cloud_max_angle) {
    status = set_fov_end(setting_cloud_max_angle);
    if (status != ok) return status;

    logger_->debug(
      "VelodyneHwInterface::parse_json(" + target_key +
      "): " + std::to_string(current_cloud_max_angle));
    logger_->debug(
      "sensor_configuration->cloud_max_angle: " + std::to_string(setting_cloud_max_angle));
  }

  target_key = "config.host.addr";
  auto current_host_addr = tree.get<std::string>(target_key);
  if (sensor_configuration->host_ip != current_host_addr) {
    status = set_host_addr(sensor_configuration->host_ip);
    if (status != ok) return status;

    logger_->debug("VelodyneHwInterface::parse_json(" + target_key + "): " + current_host_addr);
    logger_->debug("sensor_configuration->host_ip: " + sensor_configuration->host_ip);
  }

  target_key = "config.host.dport";
  auto current_host_dport = tree.get<std::uint16_t>(target_key);
  if (sensor_configuration->data_port != current_host_dport) {
    status = set_host_dport(sensor_configuration->data_port);
    if (status != ok) return status;

    logger_->debug(
      "VelodyneHwInterface::parse_json(" + target_key + "): " + std::to_string(current_host_dport));
    logger_->debug(
      "sensor_configuration->data_port: " + std::to_string(sensor_configuration->data_port));
  }

  target_key = "config.host.tport";
  auto current_host_tport = tree.get<std::uint16_t>(target_key);
  if (sensor_configuration->gnss_port != current_host_tport) {
    status = set_host_tport(sensor_configuration->gnss_port);
    if (status != ok) return status;

    logger_->debug(
      "VelodyneHwInterface::parse_json(" + target_key + "): " + std::to_string(current_host_tport));
    logger_->debug(
      "sensor_configuration->gnss_port: " + std::to_string(sensor_configuration->gnss_port));
  }

  return ok;
}

// sync

nebula::util::expected<std::string, VelodyneStatus> VelodyneHwInterface::get_status()
{
  return http_get_request(target_status_);
}

nebula::util::expected<std::string, VelodyneStatus> VelodyneHwInterface::get_diag()
{
  auto response = http_get_request(target_diag_);
  if (response.has_value()) {
    logger_->debug("read_response: " + response.value());
  }
  return response;
}

nebula::util::expected<std::string, VelodyneStatus> VelodyneHwInterface::get_snapshot()
{
  return http_get_request(target_snapshot_);
}

VelodyneStatus VelodyneHwInterface::set_rpm(uint16_t rpm)
{
  if (rpm < 300 || 1200 < rpm || rpm % 60 != 0) {
    return VelodyneStatus::INVALID_RPM_ERROR;
  }
  auto rt = http_post_request(target_setting_, (boost::format("rpm=%d") % rpm).str());
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_fov_start(uint16_t fov_start)
{
  if (359 < fov_start) {
    return VelodyneStatus::INVALID_FOV_ERROR;
  }
  auto rt = http_post_request(target_fov_, (boost::format("start=%d") % fov_start).str());
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_fov_end(uint16_t fov_end)
{
  if (359 < fov_end) {
    return VelodyneStatus::INVALID_FOV_ERROR;
  }
  auto rt = http_post_request(target_fov_, (boost::format("end=%d") % fov_end).str());
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_return_type(nebula::drivers::ReturnMode return_mode)
{
  std::string body_str = "";
  switch (return_mode) {
    case nebula::drivers::ReturnMode::SINGLE_STRONGEST:
      body_str = "returns=Strongest";
      break;
    case nebula::drivers::ReturnMode::SINGLE_LAST:
      body_str = "returns=Last";
      break;
    case nebula::drivers::ReturnMode::DUAL_ONLY:
      body_str = "returns=Dual";
      break;
    default:
      return VelodyneStatus::INVALID_RETURN_MODE_ERROR;
  }
  auto rt = http_post_request(target_setting_, body_str);
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::save_config()
{
  std::string body_str = "submit";
  auto rt = http_post_request(target_save_, body_str);
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::reset_system()
{
  std::string body_str = "reset_system";
  auto rt = http_post_request(target_reset_, body_str);
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::laser_on()
{
  std::string body_str = "laser=on";
  auto rt = http_post_request(target_setting_, body_str);
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::laser_off()
{
  std::string body_str = "laser=off";
  auto rt = http_post_request(target_setting_, body_str);
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::laser_on_off(bool on)
{
  std::string body_str = (boost::format("laser=%s") % (on ? "on" : "off")).str();
  auto rt = http_post_request(target_setting_, body_str);
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_host_addr(std::string addr)
{
  auto rt = http_post_request(target_host_, (boost::format("addr=%s") % addr).str());
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_host_dport(uint16_t dport)
{
  auto rt = http_post_request(target_host_, (boost::format("dport=%d") % dport).str());
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_host_tport(uint16_t tport)
{
  auto rt = http_post_request(target_host_, (boost::format("tport=%d") % tport).str());
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_net_addr(std::string addr)
{
  auto rt = http_post_request(target_net_, (boost::format("addr=%s") % addr).str());
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_net_mask(std::string mask)
{
  auto rt = http_post_request(target_net_, (boost::format("mask=%s") % mask).str());
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_net_gateway(std::string gateway)
{
  auto rt = http_post_request(target_net_, (boost::format("gateway=%s") % gateway).str());
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_net_dhcp(bool use_dhcp)
{
  auto rt =
    http_post_request(target_net_, (boost::format("dhcp=%s") % (use_dhcp ? "on" : "off")).str());
  if (!rt.has_value()) {
    return rt.error();
  }
  string_callback(rt.value());
  return Status::OK;
}

}  // namespace nebula::drivers
