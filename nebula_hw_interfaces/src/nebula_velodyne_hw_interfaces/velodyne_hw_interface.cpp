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

std::string VelodyneHwInterface::HttpGetRequest(const std::string & endpoint)
{
  std::lock_guard lock(mtx_inflight_request_);
  if (!http_client_driver_->client()->isOpen()) {
    http_client_driver_->client()->open();
  }

  std::string response = http_client_driver_->get(endpoint);
  http_client_driver_->client()->close();
  return response;
}

std::string VelodyneHwInterface::HttpPostRequest(
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

Status VelodyneHwInterface::InitializeSensorConfiguration(
  std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration)
{
  sensor_configuration_ = sensor_configuration;
  return Status::OK;
}

Status VelodyneHwInterface::SetSensorConfiguration(
  std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration)
{
  auto snapshot = GetSnapshot();
  auto tree = ParseJson(snapshot);
  VelodyneStatus status = CheckAndSetConfig(sensor_configuration, tree);

  return status;
}

Status VelodyneHwInterface::SensorInterfaceStart()
{
  try {
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();
    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&VelodyneHwInterface::ReceiveSensorPacketCallback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status VelodyneHwInterface::RegisterScanCallback(
  std::function<void(std::vector<uint8_t> & packet)> scan_callback)
{
  cloud_packet_callback_ = std::move(scan_callback);
  return Status::OK;
}

void VelodyneHwInterface::ReceiveSensorPacketCallback(std::vector<uint8_t> & buffer)
{
  if (!cloud_packet_callback_) {
    return;
  }

  cloud_packet_callback_(buffer);
}
Status VelodyneHwInterface::SensorInterfaceStop()
{
  return Status::ERROR_1;
}

Status VelodyneHwInterface::GetSensorConfiguration(SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  PrintDebug(ss.str());
  return Status::ERROR_1;
}

VelodyneStatus VelodyneHwInterface::InitHttpClient()
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

void VelodyneHwInterface::StringCallback(const std::string & str)
{
  std::cout << "VelodyneHwInterface::StringCallback: " << str << std::endl;
}

boost::property_tree::ptree VelodyneHwInterface::ParseJson(const std::string & str)
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

VelodyneStatus VelodyneHwInterface::CheckAndSetConfig(
  std::shared_ptr<const VelodyneSensorConfiguration> sensor_configuration,
  boost::property_tree::ptree tree)
{
  VelodyneStatus status;
  const auto & OK = VelodyneStatus::OK;

  std::string target_key = "config.returns";
  auto current_return_mode_str = tree.get<std::string>(target_key);
  auto current_return_mode =
    nebula::drivers::ReturnModeFromStringVelodyne(tree.get<std::string>(target_key));
  if (sensor_configuration->return_mode != current_return_mode) {
    status = SetReturnType(sensor_configuration->return_mode);
    if (status != OK) return status;

    std::cout << "VelodyneHwInterface::parse_json(" << target_key
              << "): " << current_return_mode_str << std::endl;
    std::cout << "current_return_mode: " << current_return_mode << std::endl;
    std::cout << "sensor_configuration->return_mode: " << sensor_configuration->return_mode
              << std::endl;
  }

  target_key = "config.rpm";
  auto current_rotation_speed = tree.get<u_int16_t>(target_key);
  if (sensor_configuration->rotation_speed != current_rotation_speed) {
    status = SetRpm(sensor_configuration->rotation_speed);
    if (status != OK) return status;

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
    status = SetFovStart(setting_cloud_min_angle);
    if (status != OK) return status;

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
    status = SetFovEnd(setting_cloud_max_angle);
    if (status != OK) return status;

    std::cout << "VelodyneHwInterface::parse_json(" << target_key
              << "): " << current_cloud_max_angle << std::endl;
    std::cout << "sensor_configuration->cloud_max_angle: " << setting_cloud_max_angle << std::endl;
  }

  target_key = "config.host.addr";
  auto current_host_addr = tree.get<std::string>(target_key);
  if (sensor_configuration->host_ip != current_host_addr) {
    status = SetHostAddr(sensor_configuration->host_ip);
    if (status != OK) return status;

    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_host_addr
              << std::endl;
    std::cout << "sensor_configuration->host_ip: " << sensor_configuration->host_ip << std::endl;
  }

  target_key = "config.host.dport";
  auto current_host_dport = tree.get<std::uint16_t>(target_key);
  if (sensor_configuration->data_port != current_host_dport) {
    status = SetHostDport(sensor_configuration->data_port);
    if (status != OK) return status;

    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_host_dport
              << std::endl;
    std::cout << "sensor_configuration->data_port: " << sensor_configuration->data_port
              << std::endl;
  }

  target_key = "config.host.tport";
  auto current_host_tport = tree.get<std::uint16_t>(target_key);
  if (sensor_configuration->gnss_port != current_host_tport) {
    status = SetHostTport(sensor_configuration->gnss_port);
    if (status != OK) return status;

    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_host_tport
              << std::endl;
    std::cout << "sensor_configuration->gnss_port: " << sensor_configuration->gnss_port
              << std::endl;
  }

  return OK;
}

// sync

std::string VelodyneHwInterface::GetStatus()
{
  return HttpGetRequest(TARGET_STATUS);
}

std::string VelodyneHwInterface::GetDiag()
{
  auto rt = HttpGetRequest(TARGET_DIAG);
  std::cout << "read_response: " << rt << std::endl;
  return rt;
}

std::string VelodyneHwInterface::GetSnapshot()
{
  return HttpGetRequest(TARGET_SNAPSHOT);
}

VelodyneStatus VelodyneHwInterface::SetRpm(uint16_t rpm)
{
  if (rpm < 300 || 1200 < rpm || rpm % 60 != 0) {
    return VelodyneStatus::INVALID_RPM_ERROR;
  }
  auto rt = HttpPostRequest(TARGET_SETTING, (boost::format("rpm=%d") % rpm).str());
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetFovStart(uint16_t fov_start)
{
  if (359 < fov_start) {
    return VelodyneStatus::INVALID_FOV_ERROR;
  }
  auto rt = HttpPostRequest(TARGET_FOV, (boost::format("start=%d") % fov_start).str());
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetFovEnd(uint16_t fov_end)
{
  if (359 < fov_end) {
    return VelodyneStatus::INVALID_FOV_ERROR;
  }
  auto rt = HttpPostRequest(TARGET_FOV, (boost::format("end=%d") % fov_end).str());
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetReturnType(nebula::drivers::ReturnMode return_mode)
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
  auto rt = HttpPostRequest(TARGET_SETTING, body_str);
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SaveConfig()
{
  std::string body_str = "submit";
  auto rt = HttpPostRequest(TARGET_SAVE, body_str);
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::ResetSystem()
{
  std::string body_str = "reset_system";
  auto rt = HttpPostRequest(TARGET_RESET, body_str);
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::LaserOn()
{
  std::string body_str = "laser=on";
  auto rt = HttpPostRequest(TARGET_SETTING, body_str);
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::LaserOff()
{
  std::string body_str = "laser=off";
  auto rt = HttpPostRequest(TARGET_SETTING, body_str);
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::LaserOnOff(bool on)
{
  std::string body_str = (boost::format("laser=%s") % (on ? "on" : "off")).str();
  auto rt = HttpPostRequest(TARGET_SETTING, body_str);
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetHostAddr(std::string addr)
{
  auto rt = HttpPostRequest(TARGET_HOST, (boost::format("addr=%s") % addr).str());
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetHostDport(uint16_t dport)
{
  auto rt = HttpPostRequest(TARGET_HOST, (boost::format("dport=%d") % dport).str());
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetHostTport(uint16_t tport)
{
  auto rt = HttpPostRequest(TARGET_HOST, (boost::format("tport=%d") % tport).str());
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetNetAddr(std::string addr)
{
  auto rt = HttpPostRequest(TARGET_NET, (boost::format("addr=%s") % addr).str());
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetNetMask(std::string mask)
{
  auto rt = HttpPostRequest(TARGET_NET, (boost::format("mask=%s") % mask).str());
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetNetGateway(std::string gateway)
{
  auto rt = HttpPostRequest(TARGET_NET, (boost::format("gateway=%s") % gateway).str());
  StringCallback(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetNetDhcp(bool use_dhcp)
{
  auto rt =
    HttpPostRequest(TARGET_NET, (boost::format("dhcp=%s") % (use_dhcp ? "on" : "off")).str());
  StringCallback(rt);
  return Status::OK;
}

void VelodyneHwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger = logger;
}

void VelodyneHwInterface::PrintInfo(std::string info)
{
  if (parent_node_logger) {
    RCLCPP_INFO_STREAM((*parent_node_logger), info);
  } else {
    std::cout << info << std::endl;
  }
}

void VelodyneHwInterface::PrintError(std::string error)
{
  if (parent_node_logger) {
    RCLCPP_ERROR_STREAM((*parent_node_logger), error);
  } else {
    std::cerr << error << std::endl;
  }
}

void VelodyneHwInterface::PrintDebug(std::string debug)
{
  if (parent_node_logger) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

}  // namespace drivers
}  // namespace nebula
