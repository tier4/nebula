// Copyright 2024 TIER IV, Inc.

#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp"

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/hesai/hesai_status.hpp"
#include "nebula_common/loggers/logger.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>

#include <boost/asio/socket_base.hpp>

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

#include <boost/asio.hpp>

#include <utility>

namespace nebula::drivers
{

using std::string_literals::operator""s;
using nlohmann::json;

HesaiHwInterface::HesaiHwInterface(const std::shared_ptr<loggers::Logger> & logger)
: logger_(logger),
  m_owned_ctx_{new boost::asio::io_context(1)},
  tcp_driver_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx_)},
  target_model_no_(nebula_model_to_hesai_model_no(SensorModel::UNKNOWN))
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

  // These are shared_ptrs so that in case of request timeout, the callback (if ever called) can
  // access valid memory
  auto recv_buf = std::make_shared<std::vector<uint8_t>>();
  auto response_complete = std::make_shared<bool>(false);

  auto error_code = std::make_shared<ptc_error_t>();

  std::stringstream ss;
  ss << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(command_id)
     << " (" << len << ") ";
  std::string log_tag = ss.str();

  logger_->debug(log_tag + "Entering lock");

  std::timed_mutex tm;
  tm.lock();

  if (tcp_driver_->GetIOContext()->stopped()) {
    logger_->debug(log_tag + "IOContext was stopped");
    tcp_driver_->GetIOContext()->restart();
  }

  logger_->debug(log_tag + "Sending payload");
  tcp_driver_->asyncSendReceiveHeaderPayload(
    send_buf,
    [this, log_tag, command_id, response_complete,
     error_code](const std::vector<uint8_t> & header_bytes) {
      error_code->ptc_error_code = header_bytes[3];

      size_t payload_len = (header_bytes[4] << 24) | (header_bytes[5] << 16) |
                           (header_bytes[6] << 8) | header_bytes[7];
      logger_->debug(
        log_tag + "Received header (expecting " + std::to_string(payload_len) + "B payload)");
      // If command_id in the response does not match, we got a response for another command (or
      // rubbish), probably as a result of too many simultaneous TCP connections to the sensor (e.g.
      // from GUI, Web UI, another nebula instance, etc.)
      if (header_bytes[2] != command_id) {
        error_code->error_flags |= g_tcp_error_unrelated_response;
      }
      if (payload_len == 0) {
        *response_complete = true;
      }
    },
    [this, log_tag, recv_buf, response_complete,
     error_code](const std::vector<uint8_t> & payload_bytes) {
      logger_->debug(log_tag + "Received payload");

      // Header had payload length 0 (thus, header callback processed request successfully already),
      // but we still received a payload: invalid state
      if (*response_complete) {
        error_code->error_flags |= g_tcp_error_unexpected_payload;
        return;
      }

      // Skip 8 header bytes
      recv_buf->insert(recv_buf->end(), std::next(payload_bytes.begin(), 8), payload_bytes.end());
      *response_complete = true;
    },
    [this, log_tag, &tm]() {
      logger_->debug(log_tag + "Unlocking mutex");
      tm.unlock();
      logger_->debug(log_tag + "Unlocked mutex");
    });
  this->io_context_run();
  if (!tm.try_lock_for(std::chrono::seconds(1))) {
    logger_->error(log_tag + "Request did not finish within 1s");
    error_code->error_flags |= g_tcp_error_timeout;
    return *error_code;
  }

  if (!response_complete) {
    logger_->error(log_tag + "Did not receive response");
    error_code->error_flags |= g_tcp_error_incomplete_response;
    return *error_code;
  }

  if (!error_code->ok()) {
    return *error_code;
  }

  logger_->debug(log_tag + "Received response");

  return *recv_buf;
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
  auto builder = connections::UdpSocket::Builder(
    sensor_configuration_->host_ip, sensor_configuration_->data_port);
  if (!sensor_configuration_->multicast_ip.empty()) {
    builder.join_multicast_group(sensor_configuration_->multicast_ip);
  }

  builder.set_mtu(g_mtu_size);

  try {
    builder.set_socket_buffer_size(g_udp_socket_buffer_size);
  } catch (const connections::SocketError & e) {
    throw std::runtime_error(
      "Could not set socket receive buffer size to " + std::to_string(g_udp_socket_buffer_size) +
      ". Try increasing net.core.rmem_max.");
  }

  udp_socket_.emplace(std::move(builder).bind());

  udp_socket_->subscribe([&](
                           const std::vector<uint8_t> & packet,
                           const connections::UdpSocket::RxMetadata & /* metadata */) {
    receive_sensor_packet_callback(packet);
  });

  return Status::OK;
}

Status HesaiHwInterface::register_scan_callback(
  std::function<void(const std::vector<uint8_t> &)> scan_callback)
{
  cloud_packet_callback_ = std::move(scan_callback);
  return Status::OK;
}

void HesaiHwInterface::receive_sensor_packet_callback(const std::vector<uint8_t> & buffer)
{
  cloud_packet_callback_(buffer);
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
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  logger_->debug("HesaiHwInterface::InitializeTcpDriver");
  logger_->debug("st: tcp_driver_->init_socket");
  logger_->debug("sensor_configuration_->sensor_ip=" + sensor_configuration_->sensor_ip);
  logger_->debug("sensor_configuration_->host_ip=" + sensor_configuration_->host_ip);
  logger_->debug("PandarTcpCommandPort=" + std::to_string(g_pandar_tcp_command_port));
#endif
  tcp_driver_->init_socket(
    sensor_configuration_->sensor_ip, g_pandar_tcp_command_port, sensor_configuration_->host_ip,
    g_pandar_tcp_command_port);
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  logger_->debug("ed: tcp_driver_->init_socket");
#endif
  if (!tcp_driver_->open()) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    logger_->debug("!tcp_driver_->open()");
#endif
    //    tcp_driver_->close();
    tcp_driver_->closeSync();
    return Status::ERROR_1;
  }
  return Status::OK;
}

Status HesaiHwInterface::finalize_tcp_driver()
{
  try {
    if (tcp_driver_) {
      tcp_driver_->close();
    }
  } catch (std::exception & e) {
    logger_->error("Error while finalizing the TcpDriver");
    return Status::UDP_CONNECTION_ERROR;
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

HesaiPtpDiagPort HesaiHwInterface::get_ptp_diag_port()
{
  auto response_or_err =
    send_receive(g_ptc_command_ptp_diagnostics, {g_ptc_command_ptp_port_data_set});
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  auto diag_port = check_size_and_parse<HesaiPtpDiagPort>(response);
  return diag_port;
}

HesaiPtpDiagTime HesaiHwInterface::get_ptp_diag_time()
{
  auto response_or_err =
    send_receive(g_ptc_command_ptp_diagnostics, {g_ptc_command_ptp_time_status_np});
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  auto diag_time = check_size_and_parse<HesaiPtpDiagTime>(response);
  return diag_time;
}

HesaiPtpDiagGrandmaster HesaiHwInterface::get_ptp_diag_grandmaster()
{
  auto response_or_err =
    send_receive(g_ptc_command_ptp_diagnostics, {g_ptc_command_ptp_grandmaster_settings_np});
  auto response =
    response_or_err.value_or_throw(pretty_print_ptc_error(response_or_err.error_or({})));
  auto diag_grandmaster = check_size_and_parse<HesaiPtpDiagGrandmaster>(response);
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

void HesaiHwInterface::io_context_run()
{
  m_owned_ctx_->run();
}

std::shared_ptr<boost::asio::io_context> HesaiHwInterface::get_io_context()
{
  return m_owned_ctx_;
}

HesaiStatus HesaiHwInterface::get_http_client_driver_once(
  std::shared_ptr<boost::asio::io_context> ctx,
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd)
{
  hcd = std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>(
    new ::drivers::tcp_driver::HttpClientDriver(ctx));
  try {
    hcd->init_client(sensor_configuration_->sensor_ip, 80);
  } catch (const std::exception & ex) {
    Status status = Status::HTTP_CONNECTION_ERROR;
    std::stringstream ss;
    ss << "HesaiHwInterface::GetHttpClientDriverOnce: " << status
       << sensor_configuration_->sensor_ip << "," << 80 << std::endl;
    logger_->error(ss.str());
    return Status::HTTP_CONNECTION_ERROR;
  }
  return Status::OK;
}

HesaiStatus HesaiHwInterface::get_http_client_driver_once(
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd_tmp;
  auto st = get_http_client_driver_once(std::make_shared<boost::asio::io_context>(), hcd_tmp);
  hcd = std::move(hcd_tmp);
  return st;
}

void HesaiHwInterface::str_cb(const std::string & str)
{
  logger_->info(str);
}

std::pair<HesaiStatus, std::string> HesaiHwInterface::unwrap_http_response(
  const std::string & response)
{
  json j;
  try {
    j = json::parse(response);
  } catch (const json::parse_error & e) {
    return {Status::ERROR_1, "JSON response malformed: "s + e.what()};
  }

  if (!j.contains("Head") || !j["Head"].contains("ErrorCode") || !j["Head"].contains("Message")) {
    return {Status::ERROR_1, "Unexpected JSON structure"};
  }

  json error_code = j["Head"]["ErrorCode"];
  json message = j["Head"]["Message"];
  if (error_code == "0") {
    return {Status::OK, message};
  }

  return {Status::ERROR_1, message};
}

HesaiStatus HesaiHwInterface::set_spin_speed_async_http(
  std::shared_ptr<boost::asio::io_context> ctx, uint16_t rpm)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = get_http_client_driver_once(ctx, hcd);
  if (st != Status::OK) {
    return st;
  }

  int rpm_key = 2;
  switch (rpm) {
    case 300:
      rpm_key = 1;
      break;
    case 600:
      rpm_key = 2;
      break;
    case 1200:
      rpm_key = 3;
      break;
    default:
      return HesaiStatus::INVALID_RPM_ERROR;
      break;
  }
  hcd->asyncGet(
    [this](const std::string & str) { str_cb(str); },
    (boost::format("/pandar.cgi?action=set&object=lidar&key=spin_speed&value=%d") % rpm_key).str());
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

HesaiStatus HesaiHwInterface::set_spin_speed_async_http(uint16_t rpm)
{
  return set_spin_speed_async_http(std::make_shared<boost::asio::io_context>(), rpm);
}

HesaiStatus HesaiHwInterface::set_ptp_config_sync_http(
  std::shared_ptr<boost::asio::io_context> ctx, int profile, int domain, int network,
  int logAnnounceInterval, int logSyncInterval, int logMinDelayReqInterval)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = get_http_client_driver_once(ctx, hcd);
  if (st != Status::OK) {
    return st;
  }

  auto response = hcd->get((boost::format(
                              "/pandar.cgi?action=set&object=lidar&key=ptp_configuration&value={"
                              "\"Profile\": %d,"
                              "\"Domain\": %d,"
                              "\"Network\": %d,"
                              "\"LogAnnounceInterval\": %d,"
                              "\"LogSyncInterval\": %d,"
                              "\"LogMinDelayReqInterval\": %d,"
                              "\"tsn_switch\": %d"
                              "}") %
                            profile % domain % network % logAnnounceInterval % logSyncInterval %
                            logMinDelayReqInterval % 0)
                             .str());
  ctx->run();
  return unwrap_http_response(response).first;
}

HesaiStatus HesaiHwInterface::set_ptp_config_sync_http(
  int profile, int domain, int network, int logAnnounceInterval, int logSyncInterval,
  int logMinDelayReqInterval)
{
  return set_ptp_config_sync_http(
    std::make_shared<boost::asio::io_context>(), profile, domain, network, logAnnounceInterval,
    logSyncInterval, logMinDelayReqInterval);
}

HesaiStatus HesaiHwInterface::set_sync_angle_sync_http(
  std::shared_ptr<boost::asio::io_context> ctx, int enable, int angle)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = get_http_client_driver_once(ctx, hcd);
  if (st != Status::OK) {
    return st;
  }
  auto tmp_str = (boost::format(
                    "/pandar.cgi?action=set&object=lidar_sync&key=sync_angle&value={"
                    "\"sync\": %d,"
                    "\"syncAngle\": %d"
                    "}") %
                  enable % angle)
                   .str();
  auto response = hcd->get(tmp_str);
  ctx->run();
  return unwrap_http_response(response).first;
}

HesaiStatus HesaiHwInterface::set_sync_angle_sync_http(int enable, int angle)
{
  return set_sync_angle_sync_http(std::make_shared<boost::asio::io_context>(), enable, angle);
}

HesaiStatus HesaiHwInterface::get_lidar_monitor_async_http(
  std::shared_ptr<boost::asio::io_context> ctx,
  std::function<void(const std::string & str)> str_callback)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = get_http_client_driver_once(ctx, hcd);
  if (st != Status::OK) {
    logger_->error("HesaiHwInterface::GetLidarMonitorAsyncHttp: cannot GetHttpClientDriverOnce");
    return st;
  }

  hcd->asyncGet(
    [str_callback](const std::string & str) { str_callback(str); },
    "/pandar.cgi?action=get&object=lidar_monitor");
  boost::system::error_code ec;
  ctx->run(ec);
  if (ec) {
    logger_->error("HesaiHwInterface::GetLidarMonitorAsyncHttp: " + ec.message());
  }
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

HesaiStatus HesaiHwInterface::get_lidar_monitor_async_http(
  std::function<void(const std::string & str)> str_callback)
{
  return get_lidar_monitor_async_http(std::make_shared<boost::asio::io_context>(), str_callback);
}

HesaiStatus HesaiHwInterface::check_and_set_config(
  std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration,
  std::shared_ptr<HesaiConfigBase> hesai_config_ptr)
{
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  logger_->debug("Start CheckAndSetConfig(HesaiConfig)!");
#endif
  const auto hesai_config = hesai_config_ptr->get();
  auto current_return_mode = nebula::drivers::return_mode_from_int_hesai(
    hesai_config.return_mode, sensor_configuration->sensor_model);
  // Avoids spamming the sensor, which leads to failure when configuring it.
  auto wait_time = 100ms;
  if (sensor_configuration->return_mode != current_return_mode) {
    std::stringstream ss;
    ss << current_return_mode;
    logger_->info("Current LiDAR return_mode: " + ss.str());
    std::stringstream ss2;
    ss2 << sensor_configuration->return_mode;
    logger_->info("Current Configuration return_mode: " + ss2.str());
    std::thread t([this, sensor_configuration] {
      auto return_mode_int = nebula::drivers::int_from_return_mode_hesai(
        sensor_configuration->return_mode, sensor_configuration->sensor_model);
      if (return_mode_int < 0) {
        logger_->error(
          "Invalid Return Mode for this sensor. Please check your settings. Falling back to Dual "
          "mode.");
        return_mode_int = 2;
      }
      set_return_mode(return_mode_int);
    });
    t.join();
    std::this_thread::sleep_for(wait_time);
  }

  auto current_rotation_speed = hesai_config.spin_rate;
  if (sensor_configuration->rotation_speed != current_rotation_speed.value()) {
    logger_->info(
      "current lidar rotation_speed: " +
      std::to_string(static_cast<int>(current_rotation_speed.value())));
    logger_->info(
      "current configuration rotation_speed: " +
      std::to_string(sensor_configuration->rotation_speed));
    if (use_http_set_spin_rate()) {
      set_spin_speed_async_http(sensor_configuration->rotation_speed);
    } else {
      logger_->info(
        "Setting up spin rate via TCP." + std::to_string(sensor_configuration->rotation_speed));
      std::thread t(
        [this, sensor_configuration] { set_spin_rate(sensor_configuration->rotation_speed); });
      t.join();
    }
    std::this_thread::sleep_for(wait_time);
  }

  bool set_flg = false;
  std::stringstream ss;
  ss << static_cast<int>(hesai_config.dest_ipaddr[0]) << "."
     << static_cast<int>(hesai_config.dest_ipaddr[1]) << "."
     << static_cast<int>(hesai_config.dest_ipaddr[2]) << "."
     << static_cast<int>(hesai_config.dest_ipaddr[3]);
  auto current_host_addr = ss.str();
  auto desired_host_addr = sensor_configuration->multicast_ip.empty()
                             ? sensor_configuration->host_ip
                             : sensor_configuration->multicast_ip;
  if (desired_host_addr != current_host_addr) {
    set_flg = true;
    logger_->info("current lidar dest_ipaddr: " + current_host_addr);
    logger_->info("current configuration host_ip: " + desired_host_addr);
  }

  auto current_host_dport = hesai_config.dest_LiDAR_udp_port;
  if (sensor_configuration->data_port != current_host_dport.value()) {
    set_flg = true;
    logger_->info(
      "current lidar dest_LiDAR_udp_port: " +
      std::to_string(static_cast<int>(current_host_dport.value())));
    logger_->info(
      "current configuration data_port: " + std::to_string(sensor_configuration->data_port));
  }

  auto current_host_tport = hesai_config.dest_gps_udp_port;
  if (sensor_configuration->gnss_port != current_host_tport.value()) {
    set_flg = true;
    logger_->info(
      "current lidar dest_gps_udp_port: " +
      std::to_string(static_cast<int>(current_host_tport.value())));
    logger_->info(
      "current configuration gnss_port: " + std::to_string(sensor_configuration->gnss_port));
  }

  if (set_flg) {
    std::vector<std::string> list_string;
    boost::split(list_string, desired_host_addr, boost::is_any_of("."));
    std::thread t([this, sensor_configuration, list_string] {
      set_destination_ip(
        std::stoi(list_string[0]), std::stoi(list_string[1]), std::stoi(list_string[2]),
        std::stoi(list_string[3]), sensor_configuration->data_port,
        sensor_configuration->gnss_port);
    });
    t.join();
    std::this_thread::sleep_for(wait_time);
  }

  if (sensor_configuration->sensor_model != SensorModel::HESAI_PANDARAT128) {
    set_flg = true;
    auto sensor_sync_angle = static_cast<int>(hesai_config.sync_angle.value() / 100);
    auto config_sync_angle = sensor_configuration->sync_angle;
    int sync_flg = 1;
    if (config_sync_angle != sensor_sync_angle) {
      set_flg = true;
    }
    if (sync_flg && set_flg) {
      logger_->info("current lidar sync: " + std::to_string(hesai_config.sync));
      logger_->info("current lidar sync_angle: " + std::to_string(sensor_sync_angle));
      logger_->info("current configuration sync_angle: " + std::to_string(config_sync_angle));
      std::thread t(
        [this, sync_flg, config_sync_angle] { set_sync_angle(sync_flg, config_sync_angle); });
      t.join();
      std::this_thread::sleep_for(wait_time);
    }

    std::thread t([this, sensor_configuration] {
      if (
        sensor_configuration->sensor_model == SensorModel::HESAI_PANDAR40P ||
        sensor_configuration->sensor_model == SensorModel::HESAI_PANDAR64 ||
        sensor_configuration->sensor_model == SensorModel::HESAI_PANDARQT64 ||
        sensor_configuration->sensor_model == SensorModel::HESAI_PANDARXT16 ||
        sensor_configuration->sensor_model == SensorModel::HESAI_PANDARXT32 ||
        sensor_configuration->sensor_model == SensorModel::HESAI_PANDARXT32M) {
        logger_->info("Trying to set Clock source to PTP");
        set_clock_source(g_hesai_lidar_ptp_clock_source);
      }
      std::ostringstream tmp_ostringstream;
      tmp_ostringstream << "Trying to set PTP Config: " << sensor_configuration->ptp_profile
                        << ", Domain: " << std::to_string(sensor_configuration->ptp_domain)
                        << ", Transport: " << sensor_configuration->ptp_transport_type
                        << ", Switch Type: " << sensor_configuration->ptp_switch_type << " via TCP";
      logger_->info(tmp_ostringstream.str());
      set_ptp_config(
        static_cast<int>(sensor_configuration->ptp_profile), sensor_configuration->ptp_domain,
        static_cast<int>(sensor_configuration->ptp_transport_type),
        static_cast<int>(sensor_configuration->ptp_switch_type), g_ptp_log_announce_interval,
        g_ptp_sync_interval, g_ptp_log_min_delay_interval);
      logger_->debug("Setting properties done");
    });
    logger_->debug("Waiting for thread to finish");

    t.join();
    logger_->debug("Thread finished");

    switch (sensor_configuration_->sensor_model) {
      case SensorModel::HESAI_PANDAR128_E4X:
      case SensorModel::HESAI_PANDARQT128:
      case SensorModel::HESAI_PANDARXT16:
      case SensorModel::HESAI_PANDARXT32:
      case SensorModel::HESAI_PANDARXT32M: {
        uint8_t sensor_ptp_lock_threshold = get_ptp_lock_offset();
        if (sensor_ptp_lock_threshold != sensor_configuration_->ptp_lock_threshold) {
          NEBULA_LOG_STREAM(
            logger_->info, "changing sensor PTP lock offset from "
                             << static_cast<int>(sensor_ptp_lock_threshold) << " to "
                             << static_cast<int>(sensor_configuration_->ptp_lock_threshold));
          set_ptp_lock_offset(sensor_configuration_->ptp_lock_threshold);
        }
        break;
      }
      default:
        break;
    }

    std::this_thread::sleep_for(wait_time);
  } else {  // AT128 only supports PTP setup via HTTP
    logger_->info("Trying to set SyncAngle via HTTP");
    set_sync_angle_sync_http(1, sensor_configuration->sync_angle);
    std::ostringstream tmp_ostringstream;
    tmp_ostringstream << "Trying to set PTP Config: " << sensor_configuration->ptp_profile
                      << ", Domain: " << sensor_configuration->ptp_domain
                      << ", Transport: " << sensor_configuration->ptp_transport_type << " via HTTP";
    logger_->info(tmp_ostringstream.str());
    set_ptp_config_sync_http(
      static_cast<int>(sensor_configuration->ptp_profile), sensor_configuration->ptp_domain,
      static_cast<int>(sensor_configuration->ptp_transport_type), g_ptp_log_announce_interval,
      g_ptp_sync_interval, g_ptp_log_min_delay_interval);
  }

  if (
    sensor_configuration->sensor_model == SensorModel::HESAI_PANDAR128_E3X ||
    sensor_configuration->sensor_model == SensorModel::HESAI_PANDAR128_E4X) {
    auto hires_currently_enabled = get_high_resolution_mode();

    if (hires_currently_enabled != sensor_configuration->hires_mode) {
      logger_->info("current lidar hires_mode: " + std::to_string(hires_currently_enabled));
      logger_->info(
        "current configuration hires_mode: " + std::to_string(sensor_configuration->hires_mode));

      logger_->info("Setting hires_mode via TCP.");
      set_high_resolution_mode(sensor_configuration->hires_mode);
    }
  }

#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  logger_->debug("End CheckAndSetConfig(HesaiConfig)!");
#endif
  logger_->debug("GetAndCheckConfig(HesaiConfig) finished");

  return Status::OK;
}

HesaiStatus HesaiHwInterface::check_and_set_config(
  std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration,
  HesaiLidarRangeAll hesai_lidar_range_all)
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  logger_->debug("Start CheckAndSetConfig(HesaiLidarRangeAll)!");
#endif
  //*
  // g_ptc_command_set_lidar_range
  bool set_flg = false;
  if (hesai_lidar_range_all.method != 0) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    logger_->debug(
      "current hesai_lidar_range_all.method: " + std::to_string(hesai_lidar_range_all.method));
#endif
    set_flg = true;
  } else {
    auto current_cloud_min_angle_ddeg = hesai_lidar_range_all.start;
    if (
      static_cast<int>(sensor_configuration->cloud_min_angle * 10) !=
      current_cloud_min_angle_ddeg.value()) {
      set_flg = true;
      logger_->info(
        "current lidar range.start: " +
        std::to_string(static_cast<int>(current_cloud_min_angle_ddeg.value())));
      logger_->info(
        "current configuration cloud_min_angle: " +
        std::to_string(sensor_configuration->cloud_min_angle));
    }

    auto current_cloud_max_angle_ddeg = hesai_lidar_range_all.end;
    if (
      static_cast<int>(sensor_configuration->cloud_max_angle * 10) !=
      current_cloud_max_angle_ddeg.value()) {
      set_flg = true;
      logger_->info(
        "current lidar range.end: " +
        std::to_string(static_cast<int>(current_cloud_max_angle_ddeg.value())));
      logger_->info(
        "current configuration cloud_max_angle: " +
        std::to_string(sensor_configuration->cloud_max_angle));
    }
  }

  if (set_flg) {
    std::thread t([this, sensor_configuration] {
      set_lidar_range(
        static_cast<int>(sensor_configuration->cloud_min_angle * 10),
        static_cast<int>(sensor_configuration->cloud_max_angle * 10));
    });
    t.join();
  }

#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  logger_->debug("End CheckAndSetConfig(HesaiLidarRangeAll)!");
#endif
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

HesaiStatus HesaiHwInterface::check_and_set_config()
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  logger_->debug("Start CheckAndSetConfig!");
#endif
  std::thread t([this] {
    auto result = get_config();
    check_and_set_config(
      std::static_pointer_cast<const HesaiSensorConfiguration>(sensor_configuration_), result);
  });
  t.join();

  if (
    sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARAT128 ||
    sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR64) {
    return Status::OK;
  }

  std::thread t2([this] {
    auto result = get_lidar_range();
    check_and_set_config(
      std::static_pointer_cast<const HesaiSensorConfiguration>(sensor_configuration_), result);
  });
  t2.join();
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  logger_->debug("End CheckAndSetConfig!");
#endif
  return Status::OK;
}

/*
0: Pandar40P
2: Pandar64
3: Pandar 128
15: PandarQT
17: Pandar40M
20: PandarMind64 (Pandar64)
25: Pandar XT32
26: Pandar XT16
32: QT128C2X
38: ?
40: AT128?
42: OT128
48: ?
*/
int HesaiHwInterface::nebula_model_to_hesai_model_no(nebula::drivers::SensorModel model)
{
  switch (model) {
    case SensorModel::HESAI_PANDAR40P:
      return 0;
    case SensorModel::HESAI_PANDAR64:
      return 2;
    case SensorModel::HESAI_PANDARQT64:  // check required
      return 15;
    case SensorModel::HESAI_PANDAR40M:
      return 17;
    case SensorModel::HESAI_PANDARXT32:
      return 25;
    case SensorModel::HESAI_PANDARXT16:
      return 26;
    case SensorModel::HESAI_PANDARQT128:
      return 32;
    case SensorModel::HESAI_PANDARXT32M:
      return 38;
    case SensorModel::HESAI_PANDAR128_E3X:  // check required
      return 40;
    case SensorModel::HESAI_PANDAR128_E4X:  // OT128
      return 42;
    case SensorModel::HESAI_PANDARAT128:
      return 48;
    // All other vendors and unknown sensors
    default:
      return -1;
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

bool HesaiHwInterface::use_http_set_spin_rate(int model)
{
  switch (model) {
    case 0:
      return true;
      break;
    case 2:
      return true;
      break;
    case 3:
      return false;
      break;
    case 15:
      return true;
      break;
    case 17:
      return true;
      break;
    case 25:
      return false;
      break;
    case 26:
      return false;
      break;
    case 32:
      return false;
      break;
    case 38:
      return false;
      break;
    case 42:
      return false;
      break;
    case 48:
      return false;
      break;
    default:
      return true;
      break;
  }
}
bool HesaiHwInterface::use_http_set_spin_rate()
{
  return use_http_set_spin_rate(target_model_no_);
}
bool HesaiHwInterface::use_http_get_lidar_monitor(int model)
{
  switch (model) {
    case 0:
      return true;
      break;
    case 2:
      return true;
      break;
    case 3:
      return false;
      break;
    case 15:
      return true;
      break;
    case 17:
      return true;
      break;
    case 25:
      return false;
      break;
    case 26:
      return false;
      break;
    case 32:
      return true;
      break;
    case 38:
      return false;
      break;
    case 42:
      return false;
      break;
    case 48:
      return false;
      break;
    default:
      return true;
      break;
  }
}
bool HesaiHwInterface::use_http_get_lidar_monitor()
{
  return use_http_get_lidar_monitor(target_model_no_);
}

std::string HesaiHwInterface::pretty_print_ptc_error(ptc_error_t error_code)
{
  if (error_code.ok()) {
    return "No error";
  }

  auto ptc_error = error_code.ptc_error_code;
  auto error_flags = error_code.error_flags;
  std::stringstream ss;

  if (ptc_error) {
    ss << "Sensor error: 0x" << std::setfill('0') << std::setw(2) << std::hex
       << static_cast<int>(ptc_error) << ' ';
  }

  switch (ptc_error) {
    case g_ptc_error_code_no_error:
      break;
    case g_ptc_error_code_invalid_input_param:
      ss << "Invalid input parameter";
      break;
    case g_ptc_error_code_server_conn_failed:
      ss << "Failure to connect to server";
      break;
    case g_ptc_error_code_invalid_data:
      ss << "No valid data returned";
      break;
    case g_ptc_error_code_out_of_memory:
      ss << "Server does not have enough memory";
      break;
    case g_ptc_error_code_unsupported_cmd:
      ss << "Server does not support this command yet";
      break;
    case g_ptc_error_code_fpga_comm_failed:
      ss << "Server failed to communicate with FPGA";
      break;
    case g_ptc_error_code_other:
      ss << "Unspecified internal error";
      break;
    default:
      ss << "Unknown error";
      break;
  }

  if (!error_flags) {
    return ss.str();
  }

  if (ptc_error) {
    ss << ", ";
  }

  ss << "Communication error: ";
  std::vector<std::string> nebula_errors;

  if (error_flags & g_tcp_error_incomplete_response) {
    nebula_errors.emplace_back("Incomplete response payload");
  }
  if (error_flags & g_tcp_error_timeout) {
    nebula_errors.emplace_back("Request timeout");
  }
  if (error_flags & g_tcp_error_unexpected_payload) {
    nebula_errors.emplace_back("Received payload but expected payload length 0");
  }
  if (error_flags & g_tcp_error_unrelated_response) {
    nebula_errors.emplace_back("Received unrelated response");
  }

  ss << boost::algorithm::join(nebula_errors, ", ");

  return ss.str();
}

template <typename T>
T HesaiHwInterface::check_size_and_parse(const std::vector<uint8_t> & data)
{
  if (data.size() < sizeof(T)) {
    throw std::runtime_error("Attempted to parse too-small payload");
  }

  if (data.size() > sizeof(T)) {
    // TODO(mojomex): having  a static variable for this is not optimal, but the loggers::Logger
    // class does not support things like _ONCE macros yet
    static bool already_warned_for_this_type = false;
    if (!already_warned_for_this_type) {
      logger_->warn("Sensor returned longer payload than expected. Truncating and parsing anyway.");
      already_warned_for_this_type = true;
    }
  }

  T parsed;
  memcpy(&parsed, data.data(), sizeof(T));
  return parsed;
}

}  // namespace nebula::drivers
