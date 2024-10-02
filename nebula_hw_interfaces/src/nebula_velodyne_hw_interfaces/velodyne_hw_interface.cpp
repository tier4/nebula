// Copyright 2024 TIER IV, Inc.

#include "nebula_hw_interfaces/nebula_hw_interfaces_velodyne/velodyne_hw_interface.hpp"

namespace nebula
{
namespace drivers
{
VelodyneHwInterface::VelodyneHwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  boost_ctx_{new boost::asio::io_context()},
  http_client_driver_{new ::drivers::tcp_driver::HttpClientDriver(boost_ctx_)}
{
}

std::string VelodyneHwInterface::http_get_request(const std::string & endpoint)
{
  std::lock_guard lock(mtx_inflight_request_);
  if (!http_client_driver_->client()->isOpen()) {
    http_client_driver_->client()->open();
  }

  std::string response = http_client_driver_->get(endpoint);
  http_client_driver_->client()->close();
  return response;
}

std::string VelodyneHwInterface::http_post_request(
  const std::string & endpoint, const std::string & body)
{
  std::lock_guard lock(mtx_inflight_request_);
  if (!http_client_driver_->client()->isOpen()) {
    http_client_driver_->client()->open();
  }

  std::string response = http_client_driver_->post(endpoint, body);
  http_client_driver_->client()->close();
  return response;
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
  auto tree = parse_json(snapshot);
  VelodyneStatus status = check_and_set_config(sensor_configuration, tree);

  return status;
}

Status VelodyneHwInterface::sensor_interface_start()
{
  try {
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();
    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&VelodyneHwInterface::receive_sensor_packet_callback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status VelodyneHwInterface::register_scan_callback(
  std::function<void(std::vector<uint8_t> & packet)> scan_callback)
{
  cloud_packet_callback_ = std::move(scan_callback);
  return Status::OK;
}

void VelodyneHwInterface::receive_sensor_packet_callback(std::vector<uint8_t> & buffer)
{
  if (!cloud_packet_callback_) {
    return;
  }

  cloud_packet_callback_(buffer);
}
Status VelodyneHwInterface::sensor_interface_stop()
{
  return Status::ERROR_1;
}

Status VelodyneHwInterface::get_sensor_configuration(SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  print_debug(ss.str());
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
  std::cout << "VelodyneHwInterface::string_callback: " << str << std::endl;
}

boost::property_tree::ptree VelodyneHwInterface::parse_json(const std::string & str)
{
  boost::property_tree::ptree tree;
  try {
    std::stringstream ss;
    ss << str;
    boost::property_tree::read_json(ss, tree);
  } catch (boost::property_tree::json_parser_error & e) {
    std::cerr << "Error on ParseJson: " << e.what() << std::endl;
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

    std::cout << "VelodyneHwInterface::parse_json(" << target_key
              << "): " << current_return_mode_str << std::endl;
    std::cout << "current_return_mode: " << current_return_mode << std::endl;
    std::cout << "sensor_configuration->return_mode: " << sensor_configuration->return_mode
              << std::endl;
  }

  target_key = "config.rpm";
  auto current_rotation_speed = tree.get<u_int16_t>(target_key);
  if (sensor_configuration->rotation_speed != current_rotation_speed) {
    status = set_rpm(sensor_configuration->rotation_speed);
    if (status != ok) return status;

    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_rotation_speed
              << std::endl;
    std::cout << "sensor_configuration->rotation_speed: " << sensor_configuration->rotation_speed
              << std::endl;
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

    std::cout << "VelodyneHwInterface::parse_json(" << target_key
              << "): " << current_cloud_min_angle << std::endl;
    std::cout << "sensor_configuration->cloud_min_angle: " << setting_cloud_min_angle << std::endl;
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

    std::cout << "VelodyneHwInterface::parse_json(" << target_key
              << "): " << current_cloud_max_angle << std::endl;
    std::cout << "sensor_configuration->cloud_max_angle: " << setting_cloud_max_angle << std::endl;
  }

  target_key = "config.host.addr";
  auto current_host_addr = tree.get<std::string>(target_key);
  if (sensor_configuration->host_ip != current_host_addr) {
    status = set_host_addr(sensor_configuration->host_ip);
    if (status != ok) return status;

    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_host_addr
              << std::endl;
    std::cout << "sensor_configuration->host_ip: " << sensor_configuration->host_ip << std::endl;
  }

  target_key = "config.host.dport";
  auto current_host_dport = tree.get<std::uint16_t>(target_key);
  if (sensor_configuration->data_port != current_host_dport) {
    status = set_host_dport(sensor_configuration->data_port);
    if (status != ok) return status;

    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_host_dport
              << std::endl;
    std::cout << "sensor_configuration->data_port: " << sensor_configuration->data_port
              << std::endl;
  }

  target_key = "config.host.tport";
  auto current_host_tport = tree.get<std::uint16_t>(target_key);
  if (sensor_configuration->gnss_port != current_host_tport) {
    status = set_host_tport(sensor_configuration->gnss_port);
    if (status != ok) return status;

    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_host_tport
              << std::endl;
    std::cout << "sensor_configuration->gnss_port: " << sensor_configuration->gnss_port
              << std::endl;
  }

  return ok;
}

// sync

std::string VelodyneHwInterface::get_status()
{
  return http_get_request(target_status_);
}

std::string VelodyneHwInterface::get_diag()
{
  auto rt = http_get_request(target_diag_);
  std::cout << "read_response: " << rt << std::endl;
  return rt;
}

std::string VelodyneHwInterface::get_snapshot()
{
  return http_get_request(target_snapshot_);
}

VelodyneStatus VelodyneHwInterface::set_rpm(uint16_t rpm)
{
  if (rpm < 300 || 1200 < rpm || rpm % 60 != 0) {
    return VelodyneStatus::INVALID_RPM_ERROR;
  }
  auto rt = http_post_request(target_setting_, (boost::format("rpm=%d") % rpm).str());
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_fov_start(uint16_t fov_start)
{
  if (359 < fov_start) {
    return VelodyneStatus::INVALID_FOV_ERROR;
  }
  auto rt = http_post_request(target_fov_, (boost::format("start=%d") % fov_start).str());
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_fov_end(uint16_t fov_end)
{
  if (359 < fov_end) {
    return VelodyneStatus::INVALID_FOV_ERROR;
  }
  auto rt = http_post_request(target_fov_, (boost::format("end=%d") % fov_end).str());
  string_callback(rt);
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
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::save_config()
{
  std::string body_str = "submit";
  auto rt = http_post_request(target_save_, body_str);
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::reset_system()
{
  std::string body_str = "reset_system";
  auto rt = http_post_request(target_reset_, body_str);
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::laser_on()
{
  std::string body_str = "laser=on";
  auto rt = http_post_request(target_setting_, body_str);
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::laser_off()
{
  std::string body_str = "laser=off";
  auto rt = http_post_request(target_setting_, body_str);
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::laser_on_off(bool on)
{
  std::string body_str = (boost::format("laser=%s") % (on ? "on" : "off")).str();
  auto rt = http_post_request(target_setting_, body_str);
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_host_addr(std::string addr)
{
  auto rt = http_post_request(target_host_, (boost::format("addr=%s") % addr).str());
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_host_dport(uint16_t dport)
{
  auto rt = http_post_request(target_host_, (boost::format("dport=%d") % dport).str());
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_host_tport(uint16_t tport)
{
  auto rt = http_post_request(target_host_, (boost::format("tport=%d") % tport).str());
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_net_addr(std::string addr)
{
  auto rt = http_post_request(target_net_, (boost::format("addr=%s") % addr).str());
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_net_mask(std::string mask)
{
  auto rt = http_post_request(target_net_, (boost::format("mask=%s") % mask).str());
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_net_gateway(std::string gateway)
{
  auto rt = http_post_request(target_net_, (boost::format("gateway=%s") % gateway).str());
  string_callback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::set_net_dhcp(bool use_dhcp)
{
  auto rt =
    http_post_request(target_net_, (boost::format("dhcp=%s") % (use_dhcp ? "on" : "off")).str());
  string_callback(rt);
  return Status::OK;
}

void VelodyneHwInterface::set_logger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger_ = logger;
}

void VelodyneHwInterface::print_info(std::string info)
{
  if (parent_node_logger_) {
    RCLCPP_INFO_STREAM((*parent_node_logger_), info);
  } else {
    std::cout << info << std::endl;
  }
}

void VelodyneHwInterface::print_error(std::string error)
{
  if (parent_node_logger_) {
    RCLCPP_ERROR_STREAM((*parent_node_logger_), error);
  } else {
    std::cerr << error << std::endl;
  }
}

void VelodyneHwInterface::print_debug(std::string debug)
{
  if (parent_node_logger_) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger_), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

}  // namespace drivers
}  // namespace nebula
