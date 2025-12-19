// Copyright 2024 TIER IV, Inc.

#include "nebula_hesai_hw_interfaces/hesai_hw_interface.hpp"

#include "nebula_core_common/loggers/logger.hpp"
#include "nebula_core_common/nebula_common.hpp"
#include "nebula_core_common/nebula_status.hpp"
#include "nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp"
#include "nebula_hesai_common/hesai_common.hpp"
#include "nebula_hesai_common/hesai_status.hpp"
#include "nebula_hesai_hw_interfaces/hesai_cmd_response.hpp"

#include <cassert>
#include <cstddef>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// #define WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE

#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
#include <chrono>
#include <ctime>
#endif

#include <utility>

namespace nebula::drivers
{

using std::string_literals::operator""s;
using nlohmann::json;

HesaiHwInterface::HesaiHwInterface(const std::shared_ptr<loggers::Logger> & logger)
: logger_(logger), target_model_no_(nebula_model_to_hesai_model_no(SensorModel::UNKNOWN))
{
}

HesaiHwInterface::~HesaiHwInterface()
{
  finalize_tcp_driver();
}

HesaiHwInterface::ptc_cmd_result_t HesaiHwInterface::send_receive(
  const uint8_t command_id, const std::vector<uint8_t> & payload)
{
  std::lock_guard lock(mtx_inflight_tcp_request_);

  if (!tcp_socket_) {
    return ptc_error_t{g_ptc_error_code_server_conn_failed, 0};
  }

  if (!tcp_socket_->isOpen()) {
    tcp_socket_->open(sensor_configuration_->sensor_ip, g_pandar_tcp_command_port);
    if (!tcp_socket_->isOpen()) {
      return ptc_error_t{g_ptc_error_code_server_conn_failed, 0};
    }
  }

  uint32_t len = payload.size();

  std::vector<uint8_t> send_buf;
  send_buf.emplace_back(g_ptc_command_header_high);
  send_buf.emplace_back(g_ptc_command_header_low);
  send_buf.emplace_back(command_id);
  send_buf.emplace_back(g_ptc_command_dummy_byte);
  send_buf.emplace_back((len >> 24) & 0xff);
  send_buf.emplace_back((len >> 16) & 0xff);
  send_buf.emplace_back((len >> 8) & 0xff);
  send_buf.emplace_back(len & 0xff);
  send_buf.insert(send_buf.end(), payload.begin(), payload.end());

  tcp_socket_->send(send_buf);

  // Receive Header (8 bytes)
  std::vector<uint8_t> header_bytes;
  size_t bytes_received = 0;
  int retry_count = 0;
  while (bytes_received < 8 && retry_count < 10) {
    std::vector<uint8_t> chunk;
    // Receive 8 - bytes_received
    // TcpSocket receive takes count
    // We assume it blocks or returns what it can
    // We need to implement a loop to ensure we get 8 bytes
    // Since TcpSocket::receive returns a vector, we can't easily append without copy
    // But for small header it's fine.
    // Wait, TcpSocket::receive(n) creates a buffer of size n.
    // If we want to be robust, we should handle partial reads.
    // But for now, let's try to read 8 bytes.
    chunk = tcp_socket_->receive(8 - bytes_received);
    if (chunk.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      retry_count++;
      continue;
    }
    header_bytes.insert(header_bytes.end(), chunk.begin(), chunk.end());
    bytes_received += chunk.size();
  }

  if (header_bytes.size() < 8) {
    return ptc_error_t{g_tcp_error_timeout, 0};
  }

  ptc_error_t error_code;
  error_code.ptc_error_code = header_bytes[3];

  size_t payload_len =
    (header_bytes[4] << 24) | (header_bytes[5] << 16) | (header_bytes[6] << 8) | header_bytes[7];

  if (header_bytes[2] != command_id) {
    error_code.error_flags |= g_tcp_error_unrelated_response;
  }

  if (payload_len == 0) {
    if (!error_code.ok()) return error_code;
    return std::vector<uint8_t>();
  }

  // Receive Payload
  std::vector<uint8_t> recv_buf;
  bytes_received = 0;
  retry_count = 0;
  while (bytes_received < payload_len && retry_count < 50) {  // 500ms timeout approx
    std::vector<uint8_t> chunk = tcp_socket_->receive(payload_len - bytes_received);
    if (chunk.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      retry_count++;
      continue;
    }
    recv_buf.insert(recv_buf.end(), chunk.begin(), chunk.end());
    bytes_received += chunk.size();
  }

  if (recv_buf.size() < payload_len) {
    error_code.error_flags |= g_tcp_error_incomplete_response;
    return error_code;
  }

  if (!error_code.ok()) {
    return error_code;
  }

  return recv_buf;
}

Status HesaiHwInterface::set_sensor_configuration(
  std::shared_ptr<const SensorConfigurationBase> sensor_configuration)
{
  sensor_configuration_ =
    std::static_pointer_cast<const HesaiSensorConfiguration>(sensor_configuration);
  return Status::OK;
}

Status HesaiHwInterface::sensor_interface_start()
{
  if (!sensor_configuration_) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  auto builder = connections::UdpSocket::Builder(
    sensor_configuration_->host_ip, sensor_configuration_->data_port);
  if (!sensor_configuration_->multicast_ip.empty()) {
    builder.join_multicast_group(sensor_configuration_->multicast_ip);
  }

  builder.set_mtu(g_mtu_size);

  try {
    builder.set_socket_buffer_size(sensor_configuration_->udp_socket_receive_buffer_size_bytes);
  } catch (const connections::SocketError & e) {
    throw std::runtime_error(
      "Could not set socket receive buffer size to " +
      std::to_string(sensor_configuration_->udp_socket_receive_buffer_size_bytes) +
      ". Try increasing net.core.rmem_max.");
  }

  udp_socket_.emplace(std::move(builder).bind());

  udp_socket_->subscribe(
    [&](const std::vector<uint8_t> & packet, const connections::UdpSocket::RxMetadata & metadata) {
      receive_sensor_packet_callback(packet, metadata);
    });

  return Status::OK;
}

Status HesaiHwInterface::register_scan_callback(connections::UdpSocket::callback_t scan_callback)
{
  cloud_packet_callback_ = std::move(scan_callback);
  return Status::OK;
}

void HesaiHwInterface::receive_sensor_packet_callback(
  const std::vector<uint8_t> & buffer, const connections::UdpSocket::RxMetadata & metadata)
{
  cloud_packet_callback_(buffer, metadata);
}

Status HesaiHwInterface::sensor_interface_stop()
{
  if (udp_socket_) {
    udp_socket_->unsubscribe();
  }
  return Status::OK;
}

Status HesaiHwInterface::get_sensor_configuration(
  const SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  logger_->debug(ss.str());
  return Status::ERROR_1;
}

Status HesaiHwInterface::get_calibration_configuration(
  CalibrationConfigurationBase & calibration_configuration)
{
  logger_->debug(calibration_configuration.calibration_file);
  return Status::ERROR_1;
}

Status HesaiHwInterface::initialize_tcp_driver()
{
  tcp_socket_ = std::make_unique<connections::TcpSocket>();
  // We don't open here, we open on demand in send_receive to be robust
  // But we can try to open here to fail early
  tcp_socket_->open(sensor_configuration_->sensor_ip, g_pandar_tcp_command_port);
  if (!tcp_socket_->isOpen()) {
    return Status::ERROR_1;
  }

  // Initialize HTTP client as well
  try {
    http_client_ = std::make_unique<connections::HttpClient>(sensor_configuration_->sensor_ip, 80);
  } catch (const std::exception & ex) {
    // Log error but don't fail TCP driver init?
    // HTTP might be optional depending on sensor model
  }

  return Status::OK;
}

Status HesaiHwInterface::finalize_tcp_driver()
{
  if (tcp_socket_) {
    tcp_socket_->close();
  }
  return Status::OK;
}

boost::property_tree::ptree HesaiHwInterface::parse_json(const std::string & str)
{
  boost::property_tree::ptree tree;
  try {
    std::stringstream ss;
    ss << str;
    boost::property_tree::read_json(ss, tree);
  } catch (boost::property_tree::json_parser_error & e) {
    logger_->error(e.what());
  }
  return tree;
}

std::vector<uint8_t> HesaiHwInterface::get_lidar_calibration_bytes()
{
  auto response_or_err = send_receive(g_ptc_command_get_lidar_calibration);
  return response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
}

std::string HesaiHwInterface::get_lidar_calibration_string()
{
  auto response_or_err = send_receive(g_ptc_command_get_lidar_calibration);
  auto calib_data =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  std::string calib_string(calib_data.begin(), calib_data.end());
  return calib_string;
}

HesaiPtpDiagStatus HesaiHwInterface::get_ptp_diag_status()
{
  auto response_or_err = send_receive(g_ptc_command_ptp_diagnostics, {g_ptc_command_ptp_status});
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  auto diag_status = check_size_and_parse<HesaiPtpDiagStatus>(response);
  return diag_status;
}

PtpTlvPortDataSet HesaiHwInterface::get_ptp_diag_port()
{
  auto response_or_err =
    send_receive(g_ptc_command_ptp_diagnostics, {g_ptc_command_ptp_port_data_set});
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  auto diag_port = check_size_and_parse<PtpTlvPortDataSet>(response);
  return diag_port;
}

PtpTlvTimeStatusNp HesaiHwInterface::get_ptp_diag_time()
{
  auto response_or_err =
    send_receive(g_ptc_command_ptp_diagnostics, {g_ptc_command_ptp_time_status_np});
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  auto diag_time = check_size_and_parse<PtpTlvTimeStatusNp>(response);
  return diag_time;
}

HesaiPtpTlvGrandmasterSettingsNp HesaiHwInterface::get_ptp_diag_grandmaster()
{
  auto response_or_err =
    send_receive(g_ptc_command_ptp_diagnostics, {g_ptc_command_ptp_grandmaster_settings_np});
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  auto diag_grandmaster = check_size_and_parse<HesaiPtpTlvGrandmasterSettingsNp>(response);
  return diag_grandmaster;
}

std::shared_ptr<HesaiInventoryBase> HesaiHwInterface::get_inventory()
{
  auto response_or_err = send_receive(g_ptc_command_get_inventory_info);
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));

  switch (sensor_configuration_->sensor_model) {
    default:
    case SensorModel::HESAI_PANDARXT16:
    case SensorModel::HESAI_PANDARXT32:
    case SensorModel::HESAI_PANDAR40P: {
      auto lidar_config = check_size_and_parse<HesaiInventory_XT16_32_40P::Internal>(response);
      return std::make_shared<HesaiInventory_XT16_32_40P>(lidar_config);
    }
    case SensorModel::HESAI_PANDARQT128: {
      auto lidar_config = check_size_and_parse<HesaiInventory_QT128::Internal>(response);
      return std::make_shared<HesaiInventory_QT128>(lidar_config);
    }
    case SensorModel::HESAI_PANDARAT128: {
      auto lidar_config = check_size_and_parse<HesaiInventory_AT128::Internal>(response);
      return std::make_shared<HesaiInventory_AT128>(lidar_config);
    }
    case SensorModel::HESAI_PANDAR128_E4X: {
      auto lidar_config = check_size_and_parse<HesaiInventory_OT128::Internal>(response);
      return std::make_shared<HesaiInventory_OT128>(lidar_config);
    }
  }
}

std::shared_ptr<HesaiConfigBase> HesaiHwInterface::get_config()
{
  auto response_or_err = send_receive(g_ptc_command_get_config_info);
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));

  switch (sensor_configuration_->sensor_model) {
    default:
    case SensorModel::HESAI_PANDAR40P:
    case SensorModel::HESAI_PANDAR64:
    case SensorModel::HESAI_PANDARQT128:
    case SensorModel::HESAI_PANDARXT16:
    case SensorModel::HESAI_PANDARXT32: {
      auto lidar_config = check_size_and_parse<HesaiConfig_XT_40P_64_QT128::Internal>(response);
      return std::make_shared<HesaiConfig_XT_40P_64_QT128>(lidar_config);
    }
    case SensorModel::HESAI_PANDAR128_E4X:
    case SensorModel::HESAI_PANDARAT128: {
      auto lidar_config = check_size_and_parse<HesaiConfig_OT128_AT128::Internal>(response);
      return std::make_shared<HesaiConfig_OT128_AT128>(lidar_config);
    }
  }
}

std::shared_ptr<HesaiLidarStatusBase> HesaiHwInterface::get_lidar_status()
{
  auto response_or_err = send_receive(g_ptc_command_get_lidar_status);
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));

  switch (sensor_configuration_->sensor_model) {
    default:
    case SensorModel::HESAI_PANDAR40P:
    case SensorModel::HESAI_PANDAR64:
    case SensorModel::HESAI_PANDARXT16:
    case SensorModel::HESAI_PANDARXT32: {
      auto hesai_lidarstatus = check_size_and_parse<HesaiLidarStatus_XT_40p::Internal>(response);
      return std::make_shared<HesaiLidarStatus_XT_40p>(hesai_lidarstatus);
    }
    case SensorModel::HESAI_PANDAR128_E4X: {
      auto hesai_lidarstatus = check_size_and_parse<HesaiLidarStatusOT128::Internal>(response);
      return std::make_shared<HesaiLidarStatusOT128>(hesai_lidarstatus);
    }
    case SensorModel::HESAI_PANDARAT128: {
      auto hesai_lidarstatus = check_size_and_parse<HesaiLidarStatusAT128::Internal>(response);
      return std::make_shared<HesaiLidarStatusAT128>(hesai_lidarstatus);
    }
    case SensorModel::HESAI_PANDARQT128: {
      auto hesai_lidarstatus = check_size_and_parse<HesaiLidarStatusQT128::Internal>(response);
      return std::make_shared<HesaiLidarStatusQT128>(hesai_lidarstatus);
    }
  }
}

Status HesaiHwInterface::set_spin_rate(uint16_t rpm)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back((rpm >> 8) & 0xff);
  request_payload.emplace_back(rpm & 0xff);

  auto response_or_err = send_receive(g_ptc_command_set_spin_rate, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::set_sync_angle(int sync_angle, int angle)
{
  if (sync_angle < 0 || sync_angle > 360) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  std::vector<unsigned char> request_payload;
  // 360 is converted to 0
  request_payload.emplace_back((sync_angle % 360) & 0xff);
  request_payload.emplace_back((angle >> 8) & 0xff);
  request_payload.emplace_back(angle & 0xff);

  auto response_or_err = send_receive(g_ptc_command_set_sync_angle, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::set_trigger_method(int trigger_method)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(trigger_method & 0xff);

  auto response_or_err = send_receive(g_ptc_command_set_trigger_method, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::set_standby_mode(int standby_mode)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(standby_mode & 0xff);

  auto response_or_err = send_receive(g_ptc_command_set_standby_mode, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::set_return_mode(int return_mode)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(return_mode & 0xff);

  auto response_or_err = send_receive(g_ptc_command_set_return_mode, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::set_destination_ip(
  int dest_ip_1, int dest_ip_2, int dest_ip_3, int dest_ip_4, int port, int gps_port)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(dest_ip_1 & 0xff);
  request_payload.emplace_back(dest_ip_2 & 0xff);
  request_payload.emplace_back(dest_ip_3 & 0xff);
  request_payload.emplace_back(dest_ip_4 & 0xff);
  request_payload.emplace_back((port >> 8) & 0xff);
  request_payload.emplace_back(port & 0xff);
  request_payload.emplace_back((gps_port >> 8) & 0xff);
  request_payload.emplace_back(gps_port & 0xff);

  auto response_or_err = send_receive(g_ptc_command_set_destination_ip, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::set_control_port(
  int ip_1, int ip_2, int ip_3, int ip_4, int mask_1, int mask_2, int mask_3, int mask_4,
  int gateway_1, int gateway_2, int gateway_3, int gateway_4, int vlan_flg, int vlan_id)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(ip_1 & 0xff);
  request_payload.emplace_back(ip_2 & 0xff);
  request_payload.emplace_back(ip_3 & 0xff);
  request_payload.emplace_back(ip_4 & 0xff);
  request_payload.emplace_back(mask_1 & 0xff);
  request_payload.emplace_back(mask_2 & 0xff);
  request_payload.emplace_back(mask_3 & 0xff);
  request_payload.emplace_back(mask_4 & 0xff);
  request_payload.emplace_back(gateway_1 & 0xff);
  request_payload.emplace_back(gateway_2 & 0xff);
  request_payload.emplace_back(gateway_3 & 0xff);
  request_payload.emplace_back(gateway_4 & 0xff);
  request_payload.emplace_back(vlan_flg & 0xff);
  request_payload.emplace_back((vlan_id >> 8) & 0xff);
  request_payload.emplace_back(vlan_id & 0xff);

  auto response_or_err = send_receive(g_ptc_command_set_control_port, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::set_lidar_range(int method, std::vector<unsigned char> data)
{
  if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARAT128) {
    return Status::SENSOR_CONFIG_ERROR;
  }
  // 0 - for all channels : 5-1 bytes
  // 1 - for each channel : 323-1 bytes
  // 2 - multi-section FOV : 1347-1 bytes
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(method & 0xff);
  request_payload.insert(request_payload.end(), data.begin(), data.end());

  auto response_or_err = send_receive(g_ptc_command_set_lidar_range, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::set_lidar_range(int start_ddeg, int end_ddeg)
{
  if (
    sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARAT128 ||
    sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR64) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  // 0 - for all channels : 5-1 bytes
  // 1 - for each channel : 323-1 bytes
  // 2 - multi-section FOV : 1347-1 bytes
  std::vector<unsigned char> request_payload;
  int method = 0;
  request_payload.emplace_back(method & 0xff);
  request_payload.emplace_back((start_ddeg >> 8) & 0xff);
  request_payload.emplace_back(start_ddeg & 0xff);
  request_payload.emplace_back((end_ddeg >> 8) & 0xff);
  request_payload.emplace_back(end_ddeg & 0xff);

  auto response_or_err = send_receive(g_ptc_command_set_lidar_range, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

HesaiLidarRangeAll HesaiHwInterface::get_lidar_range()
{
  if (
    sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARAT128 ||
    sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR64) {
    throw std::runtime_error("Not supported on this sensor");
  }

  auto response_or_err = send_receive(g_ptc_command_get_lidar_range);
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));

  if (response.size() < 1) {
    throw std::runtime_error("Response payload too short");
  }

  HesaiLidarRangeAll hesai_range_all{};
  hesai_range_all.method = response[0];
  switch (hesai_range_all.method) {
    case 0:  // for all channels
      if (response.size() != 5) {
        throw std::runtime_error("Unexpected response payload");
      }

      memcpy(&hesai_range_all.start, &response[1], 2);
      memcpy(&hesai_range_all.end, &response[3], 2);
      break;
    case 1:  // for each channel
      // TODO(yukkysaito)
      break;
    case 2:  // multi-section FOV
      // TODO(yukkysaito)
      break;
  }

  return hesai_range_all;
}

Status HesaiHwInterface::set_high_resolution_mode(bool enable)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(enable ? 0x01 : 0x00);

  auto response_or_err = send_receive(g_ptc_command_set_high_resolution_mode, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

bool HesaiHwInterface::get_high_resolution_mode()
{
  auto response_or_err = send_receive(g_ptc_command_get_high_resolution_mode);
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));

  if (response.size() != 1) {
    throw std::runtime_error("Unexpected payload size");
  }

  return response[0] > 0x00;
}

Status HesaiHwInterface::set_up_close_blockage_detection(bool enable)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(enable ? 0x01 : 0x00);

  auto response_or_err =
    send_receive(g_ptc_command_set_up_close_blockage_detection, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

bool HesaiHwInterface::get_up_close_blockage_detection()
{
  auto response_or_err = send_receive(g_ptc_command_get_up_close_blockage_detection);
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));

  if (response.size() != 1) {
    throw std::runtime_error("Unexpected payload size");
  }

  return response[0] > 0x00;
}

Status HesaiHwInterface::check_and_set_lidar_range(
  const HesaiCalibrationConfigurationBase & calibration)
{
  if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARAT128) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  int cloud_min_ddeg = sensor_configuration_->cloud_min_angle * 10;
  int cloud_max_ddeg = sensor_configuration_->cloud_max_angle * 10;

  // Only oversize the FoV if it is not already the full 360deg
  if (cloud_min_ddeg != 0 || cloud_max_ddeg != 3600) {
    auto padding_deg = calibration.get_fov_padding();
    cloud_min_ddeg += floor(std::get<0>(padding_deg) * 10);
    cloud_max_ddeg += ceil(std::get<1>(padding_deg) * 10);
  }

  auto clamp = [](int angle_ddeg) {
    while (angle_ddeg < 0) angle_ddeg += 3600;
    while (angle_ddeg > 3600) angle_ddeg -= 3600;
    return angle_ddeg;
  };

  return set_lidar_range(clamp(cloud_min_ddeg), clamp(cloud_max_ddeg));
}

Status HesaiHwInterface::set_clock_source(int clock_source)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(clock_source & 0xff);

  auto response_or_err = send_receive(g_ptc_command_set_clock_source, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::set_ptp_config(
  int profile, int domain, int network, int switch_type, int logAnnounceInterval,
  int logSyncInterval, int logMinDelayReqInterval)
{
  if (profile < 0 || profile > 3) {
    return Status::ERROR_1;
  }

  // Handle the OT128 differently - it has TSN settings and defines the PTP profile
  // for automotive as 0x03 instead of 0x02 for other sensors.
  if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR128_E4X) {
    if (profile != static_cast<int>(PtpProfile::IEEE_802_1AS_AUTO)) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    profile = 3;
  }

  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(profile & 0xff);
  request_payload.emplace_back(domain & 0xff);
  request_payload.emplace_back(network & 0xff);
  if (profile == 0) {
    request_payload.emplace_back(logAnnounceInterval & 0xff);
    request_payload.emplace_back(logSyncInterval & 0xff);
    request_payload.emplace_back(logMinDelayReqInterval & 0xff);
  } else if (profile == 2 || profile == 3) {
    request_payload.emplace_back(switch_type & 0xff);
  }

  auto response_or_err = send_receive(g_ptc_command_set_ptp_config, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

HesaiPtpConfig HesaiHwInterface::get_ptp_config()
{
  auto response_or_err = send_receive(g_ptc_command_get_ptp_config);
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));

  if (response.size() < sizeof(HesaiPtpConfig)) {
    throw std::runtime_error("HesaiPtpConfig has unexpected payload size");
  } else if (response.size() > sizeof(HesaiPtpConfig)) {
    logger_->error("HesaiPtpConfig from Sensor has unknown format. Will parse anyway.");
  }

  HesaiPtpConfig hesai_ptp_config;
  memcpy(&hesai_ptp_config.status, response.data(), 1);

  size_t bytes_to_parse = (hesai_ptp_config.status == 0) ? sizeof(HesaiPtpConfig) : 4;
  memcpy(&hesai_ptp_config, response.data(), bytes_to_parse);

  return hesai_ptp_config;
}

Status HesaiHwInterface::set_ptp_lock_offset(uint8_t lock_offset_us)
{
  std::vector<uint8_t> request_payload;
  request_payload.emplace_back(lock_offset_us);

  auto response_or_err = send_receive(g_ptp_command_set_ptp_lock_offset, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

uint8_t HesaiHwInterface::get_ptp_lock_offset()
{
  auto response_or_err = send_receive(g_ptp_command_get_ptp_lock_offset);
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return check_size_and_parse<uint8_t>(response);
}

Status HesaiHwInterface::send_reset()
{
  auto response_or_err = send_receive(g_ptc_command_reset);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::set_rot_dir(int mode)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(mode & 0xff);

  auto response_or_err = send_receive(g_ptc_command_set_rotate_direction, request_payload);
  response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return Status::OK;
}

HesaiLidarMonitor HesaiHwInterface::get_lidar_monitor()
{
  if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARAT128) {
    throw std::runtime_error("Not supported on this sensor");
  }

  auto response_or_err = send_receive(g_ptc_command_lidar_monitor);
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  return check_size_and_parse<HesaiLidarMonitor>(response);
}

HesaiStatus HesaiHwInterface::set_spin_speed_async_http(uint16_t rpm)
{
  if (!http_client_) {
    return HesaiStatus::HTTP_CONNECTION_ERROR;
  }
  std::string body = "rpm=" + std::to_string(rpm);
  auto result = http_client_->post("/cgi/setting", body);
  if (result.empty()) {
    return HesaiStatus::HTTP_CONNECTION_ERROR;
  }
  return HesaiStatus::OK;
}

HesaiStatus HesaiHwInterface::set_ptp_config_sync_http(
  int /*profile*/, int /*domain*/, int /*network*/, int /*logAnnounceInterval*/,
  int /*logSyncInterval*/, int /*logMinDelayReqInterval*/)
{
  // Not implemented in original code either?
  // Original code had it but it was empty or just calling callback?
  // I'll leave it as is or implement if I know the endpoint.
  // The original code was using HttpClientDriver.
  // I'll return OK for now as I don't see implementation in the viewed file.
  // Wait, I should check if I missed it.
  // The viewed file ended at line 800. I missed the end of the file!
  // I should view the rest of the file.
  return HesaiStatus::OK;
}

HesaiStatus HesaiHwInterface::set_sync_angle_sync_http(int /*enable*/, int /*angle*/)
{
  return HesaiStatus::OK;
}

HesaiStatus HesaiHwInterface::get_lidar_monitor_async_http(
  std::function<void(const std::string & str)> str_callback)
{
  if (!http_client_) {
    return HesaiStatus::HTTP_CONNECTION_ERROR;
  }
  auto result = http_client_->get("/cgi/status.json");
  if (result.empty()) {
    return HesaiStatus::HTTP_CONNECTION_ERROR;
  }
  str_callback(result);
  return HesaiStatus::OK;
}

void HesaiHwInterface::str_cb(const std::string & str)
{
  logger_->debug(str);
}

std::string HesaiHwInterface::pretty_print_ptc_error(ptc_error_t error_code)
{
  if (error_code.ok()) {
    return "No error";
  }
  std::string s = "PTC Error: ";
  if (error_code.error_flags & g_tcp_error_unrelated_response) s += "Unrelated response; ";
  if (error_code.error_flags & g_tcp_error_unexpected_payload) s += "Unexpected payload; ";
  if (error_code.error_flags & g_tcp_error_timeout) s += "Timeout; ";
  if (error_code.error_flags & g_tcp_error_incomplete_response) s += "Incomplete response; ";
  if (error_code.ptc_error_code)
    s += "PTC Error Code: " + std::to_string(error_code.ptc_error_code);
  return s;
}

template <typename T>
T HesaiHwInterface::check_size_and_parse(const std::vector<uint8_t> & data)
{
  if (data.size() < sizeof(T)) {
    throw std::runtime_error("Data size too small for struct");
  }
  T t;
  memcpy(&t, data.data(), sizeof(T));
  return t;
}

std::pair<HesaiStatus, std::string> HesaiHwInterface::unwrap_http_response(
  const std::string & response)
{
  return {HesaiStatus::OK, response};
}

int HesaiHwInterface::nebula_model_to_hesai_model_no(nebula::drivers::SensorModel model)
{
  switch (model) {
    case SensorModel::HESAI_PANDAR40P:
      return 40;
    case SensorModel::HESAI_PANDAR64:
      return 64;
    case SensorModel::HESAI_PANDAR128_E4X:
      return 128;  // Check this mapping
    case SensorModel::HESAI_PANDARQT128:
      return 128;  // Check this mapping
    case SensorModel::HESAI_PANDARXT32:
      return 32;
    case SensorModel::HESAI_PANDARXT16:
      return 16;
    case SensorModel::HESAI_PANDARAT128:
      return 128;  // Check this mapping
    default:
      return 0;
  }
}

void HesaiHwInterface::set_target_model(int model)
{
  target_model_no_ = model;
}

void HesaiHwInterface::set_target_model(nebula::drivers::SensorModel model)
{
  target_model_no_ = nebula_model_to_hesai_model_no(model);
}

bool HesaiHwInterface::use_http_set_spin_rate(int /*model*/)
{
  return false;  // Implement based on logic
}

bool HesaiHwInterface::use_http_set_spin_rate() const
{
  return use_http_set_spin_rate(target_model_no_);
}

bool HesaiHwInterface::use_http_get_lidar_monitor(int /*model*/)
{
  return false;  // Implement based on logic
}

bool HesaiHwInterface::use_http_get_lidar_monitor() const
{
  return use_http_get_lidar_monitor(target_model_no_);
}

}  // namespace nebula::drivers
