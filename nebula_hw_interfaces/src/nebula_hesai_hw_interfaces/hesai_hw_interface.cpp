// Copyright 2024 TIER IV, Inc.

#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp"

// #define WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE

#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
#include <chrono>
#include <ctime>
#endif

#include <boost/asio.hpp>

namespace nebula::drivers
{
HesaiHwInterface::HesaiHwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  m_owned_ctx{new boost::asio::io_context(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  tcp_driver_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx)}
{
}

HesaiHwInterface::~HesaiHwInterface()
{
  FinalizeTcpDriver();
}

HesaiHwInterface::ptc_cmd_result_t HesaiHwInterface::SendReceive(
  const uint8_t command_id, const std::vector<uint8_t> & payload)
{
  std::lock_guard lock(mtx_inflight_tcp_request_);

  uint32_t len = payload.size();

  std::vector<uint8_t> send_buf;
  send_buf.emplace_back(PTC_COMMAND_HEADER_HIGH);
  send_buf.emplace_back(PTC_COMMAND_HEADER_LOW);
  send_buf.emplace_back(command_id);
  send_buf.emplace_back(PTC_COMMAND_DUMMY_BYTE);
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

  PrintDebug(log_tag + "Entering lock");

  std::timed_mutex tm;
  tm.lock();

  if (tcp_driver_->GetIOContext()->stopped()) {
    PrintDebug(log_tag + "IOContext was stopped");
    tcp_driver_->GetIOContext()->restart();
  }

  PrintDebug(log_tag + "Sending payload");
  tcp_driver_->asyncSendReceiveHeaderPayload(
    send_buf,
    [this, log_tag, command_id, response_complete,
     error_code](const std::vector<uint8_t> & header_bytes) {
      error_code->ptc_error_code = header_bytes[3];

      size_t payload_len = (header_bytes[4] << 24) | (header_bytes[5] << 16) |
                           (header_bytes[6] << 8) | header_bytes[7];
      PrintDebug(
        log_tag + "Received header (expecting " + std::to_string(payload_len) + "B payload)");
      // If command_id in the response does not match, we got a response for another command (or
      // rubbish), probably as a result of too many simultaneous TCP connections to the sensor (e.g.
      // from GUI, Web UI, another nebula instance, etc.)
      if (header_bytes[2] != command_id) {
        error_code->error_flags |= TCP_ERROR_UNRELATED_RESPONSE;
      }
      if (payload_len == 0) {
        *response_complete = true;
      }
    },
    [this, log_tag, recv_buf, response_complete,
     error_code](const std::vector<uint8_t> & payload_bytes) {
      PrintDebug(log_tag + "Received payload");

      // Header had payload length 0 (thus, header callback processed request successfully already),
      // but we still received a payload: invalid state
      if (*response_complete == true) {
        error_code->error_flags |= TCP_ERROR_UNEXPECTED_PAYLOAD;
        return;
      }

      // Skip 8 header bytes
      recv_buf->insert(recv_buf->end(), std::next(payload_bytes.begin(), 8), payload_bytes.end());
      *response_complete = true;
    },
    [this, log_tag, &tm]() {
      PrintDebug(log_tag + "Unlocking mutex");
      tm.unlock();
      PrintDebug(log_tag + "Unlocked mutex");
    });
  this->IOContextRun();
  if (!tm.try_lock_for(std::chrono::seconds(1))) {
    PrintError(log_tag + "Request did not finish within 1s");
    error_code->error_flags |= TCP_ERROR_TIMEOUT;
    return *error_code;
  }

  if (!response_complete) {
    PrintError(log_tag + "Did not receive response");
    error_code->error_flags |= TCP_ERROR_INCOMPLETE_RESPONSE;
    return *error_code;
  }

  if (!error_code->ok()) {
    return *error_code;
  }

  PrintDebug(log_tag + "Received response");

  return *recv_buf;
}

Status HesaiHwInterface::SetSensorConfiguration(
  std::shared_ptr<const SensorConfigurationBase> sensor_configuration)
{
  sensor_configuration_ =
    std::static_pointer_cast<const HesaiSensorConfiguration>(sensor_configuration);
  return Status::OK;
}

Status HesaiHwInterface::SensorInterfaceStart()
{
  try {
    std::cout << "Starting UDP server on: " << *sensor_configuration_ << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    PrintError("init ok");
#endif
    cloud_udp_driver_->receiver()->open();
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    PrintError("open ok");
#endif
    cloud_udp_driver_->receiver()->bind();
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    PrintError("bind ok");
#endif

    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&HesaiHwInterface::ReceiveSensorPacketCallback, this, std::placeholders::_1));
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    PrintError("async receive set");
#endif
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status HesaiHwInterface::RegisterScanCallback(
  std::function<void(std::vector<uint8_t> &)> scan_callback)
{
  cloud_packet_callback_ = std::move(scan_callback);
  return Status::OK;
}

void HesaiHwInterface::ReceiveSensorPacketCallback(std::vector<uint8_t> & buffer)
{
  cloud_packet_callback_(buffer);
}
Status HesaiHwInterface::SensorInterfaceStop()
{
  return Status::ERROR_1;
}

Status HesaiHwInterface::GetSensorConfiguration(
  const SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  PrintDebug(ss.str());
  return Status::ERROR_1;
}

Status HesaiHwInterface::GetCalibrationConfiguration(
  CalibrationConfigurationBase & calibration_configuration)
{
  PrintDebug(calibration_configuration.calibration_file);
  return Status::ERROR_1;
}

Status HesaiHwInterface::InitializeTcpDriver()
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "HesaiHwInterface::InitializeTcpDriver" << std::endl;
  std::cout << "st: tcp_driver_->init_socket" << std::endl;
  std::cout << "sensor_configuration_->sensor_ip=" << sensor_configuration_->sensor_ip << std::endl;
  std::cout << "sensor_configuration_->host_ip=" << sensor_configuration_->host_ip << std::endl;
  std::cout << "PandarTcpCommandPort=" << PandarTcpCommandPort << std::endl;
#endif
  tcp_driver_->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "ed: tcp_driver_->init_socket" << std::endl;
#endif
  if (!tcp_driver_->open()) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "!tcp_driver_->open()" << std::endl;
#endif
    //    tcp_driver_->close();
    tcp_driver_->closeSync();
    return Status::ERROR_1;
  }
  return Status::OK;
}

Status HesaiHwInterface::FinalizeTcpDriver()
{
  try {
    if (tcp_driver_) {
      tcp_driver_->close();
    }
  } catch (std::exception & e) {
    PrintError("Error while finalizing the TcpDriver");
    return Status::UDP_CONNECTION_ERROR;
  }
  return Status::OK;
}

boost::property_tree::ptree HesaiHwInterface::ParseJson(const std::string & str)
{
  boost::property_tree::ptree tree;
  try {
    boost::property_tree::read_json(str, tree);
  } catch (boost::property_tree::json_parser_error & e) {
    std::cerr << e.what() << std::endl;
  }
  return tree;
}

std::vector<uint8_t> HesaiHwInterface::GetLidarCalibrationBytes()
{
  auto response_or_err = SendReceive(PTC_COMMAND_GET_LIDAR_CALIBRATION);
  return response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
}

std::string HesaiHwInterface::GetLidarCalibrationString()
{
  auto response_or_err = SendReceive(PTC_COMMAND_GET_LIDAR_CALIBRATION);
  auto calib_data =
    response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  std::string calib_string(calib_data.begin(), calib_data.end());
  return calib_string;
}

HesaiPtpDiagStatus HesaiHwInterface::GetPtpDiagStatus()
{
  auto response_or_err = SendReceive(PTC_COMMAND_PTP_DIAGNOSTICS, {PTC_COMMAND_PTP_STATUS});
  auto response = response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  auto diag_status = CheckSizeAndParse<HesaiPtpDiagStatus>(response);

  std::stringstream ss;
  ss << "HesaiHwInterface::GetPtpDiagStatus: " << diag_status;
  PrintInfo(ss.str());

  return diag_status;
}

HesaiPtpDiagPort HesaiHwInterface::GetPtpDiagPort()
{
  auto response_or_err = SendReceive(PTC_COMMAND_PTP_DIAGNOSTICS, {PTC_COMMAND_PTP_PORT_DATA_SET});
  auto response = response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  auto diag_port = CheckSizeAndParse<HesaiPtpDiagPort>(response);

  std::stringstream ss;
  ss << "HesaiHwInterface::GetPtpDiagPort: " << diag_port;
  PrintInfo(ss.str());

  return diag_port;
}

HesaiPtpDiagTime HesaiHwInterface::GetPtpDiagTime()
{
  auto response_or_err = SendReceive(PTC_COMMAND_PTP_DIAGNOSTICS, {PTC_COMMAND_PTP_TIME_STATUS_NP});
  auto response = response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  auto diag_time = CheckSizeAndParse<HesaiPtpDiagTime>(response);

  std::stringstream ss;
  ss << "HesaiHwInterface::GetPtpDiagTime: " << diag_time;
  PrintInfo(ss.str());

  return diag_time;
}

HesaiPtpDiagGrandmaster HesaiHwInterface::GetPtpDiagGrandmaster()
{
  auto response_or_err =
    SendReceive(PTC_COMMAND_PTP_DIAGNOSTICS, {PTC_COMMAND_PTP_GRANDMASTER_SETTINGS_NP});
  auto response = response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  auto diag_grandmaster = CheckSizeAndParse<HesaiPtpDiagGrandmaster>(response);

  std::stringstream ss;
  ss << "HesaiHwInterface::GetPtpDiagGrandmaster: " << diag_grandmaster;
  PrintInfo(ss.str());

  return diag_grandmaster;
}

HesaiInventory HesaiHwInterface::GetInventory()
{
  auto response_or_err = SendReceive(PTC_COMMAND_GET_INVENTORY_INFO);
  auto response = response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return CheckSizeAndParse<HesaiInventory>(response);
}

HesaiConfig HesaiHwInterface::GetConfig()
{
  auto response_or_err = SendReceive(PTC_COMMAND_GET_CONFIG_INFO);
  auto response = response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  auto hesai_config = CheckSizeAndParse<HesaiConfig>(response);
  std::cout << "Config: " << hesai_config << std::endl;
  return hesai_config;
}

HesaiLidarStatus HesaiHwInterface::GetLidarStatus()
{
  auto response_or_err = SendReceive(PTC_COMMAND_GET_LIDAR_STATUS);
  auto response = response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return CheckSizeAndParse<HesaiLidarStatus>(response);
}

Status HesaiHwInterface::SetSpinRate(uint16_t rpm)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back((rpm >> 8) & 0xff);
  request_payload.emplace_back(rpm & 0xff);

  auto response_or_err = SendReceive(PTC_COMMAND_SET_SPIN_RATE, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::SetSyncAngle(int sync_angle, int angle)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(sync_angle & 0xff);
  request_payload.emplace_back((angle >> 8) & 0xff);
  request_payload.emplace_back(angle & 0xff);

  auto response_or_err = SendReceive(PTC_COMMAND_SET_SYNC_ANGLE, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::SetTriggerMethod(int trigger_method)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(trigger_method & 0xff);

  auto response_or_err = SendReceive(PTC_COMMAND_SET_TRIGGER_METHOD, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::SetStandbyMode(int standby_mode)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(standby_mode & 0xff);

  auto response_or_err = SendReceive(PTC_COMMAND_SET_STANDBY_MODE, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::SetReturnMode(int return_mode)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(return_mode & 0xff);

  auto response_or_err = SendReceive(PTC_COMMAND_SET_RETURN_MODE, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::SetDestinationIp(
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

  auto response_or_err = SendReceive(PTC_COMMAND_SET_DESTINATION_IP, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::SetControlPort(
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

  auto response_or_err = SendReceive(PTC_COMMAND_SET_CONTROL_PORT, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::SetLidarRange(int method, std::vector<unsigned char> data)
{
  // 0 - for all channels : 5-1 bytes
  // 1 - for each channel : 323-1 bytes
  // 2 - multi-section FOV : 1347-1 bytes
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(method & 0xff);
  request_payload.insert(request_payload.end(), data.begin(), data.end());

  auto response_or_err = SendReceive(PTC_COMMAND_SET_LIDAR_RANGE, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::SetLidarRange(int start, int end)
{
  // 0 - for all channels : 5-1 bytes
  // 1 - for each channel : 323-1 bytes
  // 2 - multi-section FOV : 1347-1 bytes
  std::vector<unsigned char> request_payload;
  int method = 0;
  request_payload.emplace_back(method & 0xff);
  request_payload.emplace_back((start >> 8) & 0xff);
  request_payload.emplace_back(start & 0xff);
  request_payload.emplace_back((end >> 8) & 0xff);
  request_payload.emplace_back(end & 0xff);

  auto response_or_err = SendReceive(PTC_COMMAND_SET_LIDAR_RANGE, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

HesaiLidarRangeAll HesaiHwInterface::GetLidarRange()
{
  auto response_or_err = SendReceive(PTC_COMMAND_GET_LIDAR_RANGE);
  auto response = response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));

  if (response.size() < 1) {
    throw std::runtime_error("Response payload too short");
  }

  HesaiLidarRangeAll hesai_range_all;
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

Status HesaiHwInterface::checkAndSetLidarRange(
  const HesaiCalibrationConfigurationBase & calibration)
{
  int cloud_min = sensor_configuration_->cloud_min_angle * 10;
  int cloud_max = sensor_configuration_->cloud_max_angle * 10;

  std::cout << "Starting with HW FoV of " << cloud_min << "~" << cloud_max << std::endl;

  // Only oversize the FoV if it is not already the full 360deg
  if (cloud_min != 0 || cloud_max != 3600) {
    auto padding = calibration.getFovPadding();
    cloud_min += floor(std::get<0>(padding) * 10);
    cloud_max += ceil(std::get<1>(padding) * 10);
  }

  auto clamp = [](int x) {
    while (x < 0) x += 3600;
    while (x > 3600) x -= 3600;
    return x;
  };

  std::cout << "Setting HW FoV to " << cloud_min << "~" << cloud_max << std::endl;

  return SetLidarRange(clamp(cloud_min), clamp(cloud_max));
}

Status HesaiHwInterface::SetClockSource(int clock_source)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(clock_source & 0xff);

  auto response_or_err = SendReceive(PTC_COMMAND_SET_CLOCK_SOURCE, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::SetPtpConfig(
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

  auto response_or_err = SendReceive(PTC_COMMAND_SET_PTP_CONFIG, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

HesaiPtpConfig HesaiHwInterface::GetPtpConfig()
{
  auto response_or_err = SendReceive(PTC_COMMAND_GET_PTP_CONFIG);
  auto response = response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));

  if (response.size() < sizeof(HesaiPtpConfig)) {
    throw std::runtime_error("HesaiPtpConfig has unexpected payload size");
  } else if (response.size() > sizeof(HesaiPtpConfig)) {
    PrintError("HesaiPtpConfig from Sensor has unknown format. Will parse anyway.");
  }

  HesaiPtpConfig hesai_ptp_config;
  memcpy(&hesai_ptp_config.status, response.data(), 1);

  size_t bytes_to_parse = (hesai_ptp_config.status == 0) ? sizeof(HesaiPtpConfig) : 4;
  memcpy(&hesai_ptp_config, response.data(), bytes_to_parse);

  std::stringstream ss;
  ss << "HesaiHwInterface::GetPtpConfig: " << hesai_ptp_config;
  PrintInfo(ss.str());

  return hesai_ptp_config;
}

Status HesaiHwInterface::SendReset()
{
  auto response_or_err = SendReceive(PTC_COMMAND_RESET);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

Status HesaiHwInterface::SetRotDir(int mode)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(mode & 0xff);

  auto response_or_err = SendReceive(PTC_COMMAND_SET_ROTATE_DIRECTION, request_payload);
  response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return Status::OK;
}

HesaiLidarMonitor HesaiHwInterface::GetLidarMonitor()
{
  auto response_or_err = SendReceive(PTC_COMMAND_LIDAR_MONITOR);
  auto response = response_or_err.value_or_throw(PrettyPrintPTCError(response_or_err.error_or({})));
  return CheckSizeAndParse<HesaiLidarMonitor>(response);
}

void HesaiHwInterface::IOContextRun()
{
  m_owned_ctx->run();
}

std::shared_ptr<boost::asio::io_context> HesaiHwInterface::GetIOContext()
{
  return m_owned_ctx;
}

HesaiStatus HesaiHwInterface::GetHttpClientDriverOnce(
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
    PrintError(ss.str());
    return Status::HTTP_CONNECTION_ERROR;
  }
  return Status::OK;
}

HesaiStatus HesaiHwInterface::GetHttpClientDriverOnce(
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd_tmp;
  auto st = GetHttpClientDriverOnce(std::make_shared<boost::asio::io_context>(), hcd_tmp);
  hcd = std::move(hcd_tmp);
  return st;
}

void HesaiHwInterface::str_cb(const std::string & str)
{
  PrintInfo(str);
}

HesaiStatus HesaiHwInterface::SetSpinSpeedAsyncHttp(
  std::shared_ptr<boost::asio::io_context> ctx, uint16_t rpm)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
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

HesaiStatus HesaiHwInterface::SetSpinSpeedAsyncHttp(uint16_t rpm)
{
  return SetSpinSpeedAsyncHttp(std::make_shared<boost::asio::io_context>(), rpm);
}

HesaiStatus HesaiHwInterface::SetPtpConfigSyncHttp(
  std::shared_ptr<boost::asio::io_context> ctx, int profile, int domain, int network,
  int logAnnounceInterval, int logSyncInterval, int logMinDelayReqInterval)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if (st != Status::OK) {
    return st;
  }

  auto response =
    hcd->get((boost::format("/pandar.cgi?action=set&object=lidar&key=ptp_configuration&value={"
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
  PrintInfo(response);
  return Status::OK;
}

HesaiStatus HesaiHwInterface::SetPtpConfigSyncHttp(
  int profile, int domain, int network, int logAnnounceInterval, int logSyncInterval,
  int logMinDelayReqInterval)
{
  return SetPtpConfigSyncHttp(
    std::make_shared<boost::asio::io_context>(), profile, domain, network, logAnnounceInterval,
    logSyncInterval, logMinDelayReqInterval);
}

HesaiStatus HesaiHwInterface::SetSyncAngleSyncHttp(
  std::shared_ptr<boost::asio::io_context> ctx, int enable, int angle)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if (st != Status::OK) {
    return st;
  }
  auto tmp_str = (boost::format("/pandar.cgi?action=set&object=lidar_sync&key=sync_angle&value={"
                                "\"sync\": %d,"
                                "\"syncAngle\": %d"
                                "}") %
                  enable % angle)
                   .str();
  PrintInfo(tmp_str);
  auto response = hcd->get(tmp_str);
  ctx->run();
  PrintInfo(response);
  return Status::OK;
}

HesaiStatus HesaiHwInterface::SetSyncAngleSyncHttp(int enable, int angle)
{
  return SetSyncAngleSyncHttp(std::make_shared<boost::asio::io_context>(), enable, angle);
}

HesaiStatus HesaiHwInterface::GetLidarMonitorAsyncHttp(
  std::shared_ptr<boost::asio::io_context> ctx,
  std::function<void(const std::string & str)> str_callback)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if (st != Status::OK) {
    PrintError("HesaiHwInterface::GetLidarMonitorAsyncHttp: cannot GetHttpClientDriverOnce");
    return st;
  }

  hcd->asyncGet(
    [str_callback](const std::string & str) { str_callback(str); },
    "/pandar.cgi?action=get&object=lidar_monitor");
  boost::system::error_code ec;
  ctx->run(ec);
  if (ec) {
    PrintError("HesaiHwInterface::GetLidarMonitorAsyncHttp: " + ec.message());
  }
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

HesaiStatus HesaiHwInterface::GetLidarMonitorAsyncHttp(
  std::function<void(const std::string & str)> str_callback)
{
  return GetLidarMonitorAsyncHttp(std::make_shared<boost::asio::io_context>(), str_callback);
}

HesaiStatus HesaiHwInterface::CheckAndSetConfig(
  std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration, HesaiConfig hesai_config)
{
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "Start CheckAndSetConfig(HesaiConfig)!!" << std::endl;
#endif
  auto current_return_mode = nebula::drivers::ReturnModeFromIntHesai(
    hesai_config.return_mode, sensor_configuration->sensor_model);
  // Avoids spamming the sensor, which leads to failure when configuring it.
  auto wait_time = 100ms;
  if (sensor_configuration->return_mode != current_return_mode) {
    std::stringstream ss;
    ss << current_return_mode;
    PrintInfo("Current LiDAR return_mode: " + ss.str());
    std::stringstream ss2;
    ss2 << sensor_configuration->return_mode;
    PrintInfo("Current Configuration return_mode: " + ss2.str());
    std::thread t([this, sensor_configuration] {
      auto return_mode_int = nebula::drivers::IntFromReturnModeHesai(
        sensor_configuration->return_mode, sensor_configuration->sensor_model);
      if (return_mode_int < 0) {
        PrintError(
          "Invalid Return Mode for this sensor. Please check your settings. Falling back to Dual "
          "mode.");
        return_mode_int = 2;
      }
      SetReturnMode(return_mode_int);
    });
    t.join();
    std::this_thread::sleep_for(wait_time);
  }

  auto current_rotation_speed = hesai_config.spin_rate;
  if (sensor_configuration->rotation_speed != current_rotation_speed.value()) {
    PrintInfo(
      "current lidar rotation_speed: " +
      std::to_string(static_cast<int>(current_rotation_speed.value())));
    PrintInfo(
      "current configuration rotation_speed: " +
      std::to_string(sensor_configuration->rotation_speed));
    if (UseHttpSetSpinRate()) {
      SetSpinSpeedAsyncHttp(sensor_configuration->rotation_speed);
    } else {
      PrintInfo(
        "Setting up spin rate via TCP." + std::to_string(sensor_configuration->rotation_speed));
      std::thread t(
        [this, sensor_configuration] { SetSpinRate(sensor_configuration->rotation_speed); });
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
  if (sensor_configuration->host_ip != current_host_addr) {
    set_flg = true;
    PrintInfo("current lidar dest_ipaddr: " + current_host_addr);
    PrintInfo("current configuration host_ip: " + sensor_configuration->host_ip);
  }

  auto current_host_dport = hesai_config.dest_LiDAR_udp_port;
  if (sensor_configuration->data_port != current_host_dport.value()) {
    set_flg = true;
    PrintInfo(
      "current lidar dest_LiDAR_udp_port: " +
      std::to_string(static_cast<int>(current_host_dport.value())));
    PrintInfo(
      "current configuration data_port: " + std::to_string(sensor_configuration->data_port));
  }

  auto current_host_tport = hesai_config.dest_gps_udp_port;
  if (sensor_configuration->gnss_port != current_host_tport.value()) {
    set_flg = true;
    PrintInfo(
      "current lidar dest_gps_udp_port: " +
      std::to_string(static_cast<int>(current_host_tport.value())));
    PrintInfo(
      "current configuration gnss_port: " + std::to_string(sensor_configuration->gnss_port));
  }

  if (set_flg) {
    std::vector<std::string> list_string;
    boost::split(list_string, sensor_configuration->host_ip, boost::is_any_of("."));
    std::thread t([this, sensor_configuration, list_string] {
      SetDestinationIp(
        std::stoi(list_string[0]), std::stoi(list_string[1]), std::stoi(list_string[2]),
        std::stoi(list_string[3]), sensor_configuration->data_port,
        sensor_configuration->gnss_port);
    });
    t.join();
    std::this_thread::sleep_for(wait_time);
  }

  if (sensor_configuration->sensor_model != SensorModel::HESAI_PANDARAT128) {
    set_flg = true;
    auto sync_angle = static_cast<int>(hesai_config.sync_angle.value() / 100);
    auto scan_phase = static_cast<int>(sensor_configuration->scan_phase);
    int sync_flg = 1;
    if (scan_phase != sync_angle) {
      set_flg = true;
    }
    if (sync_flg && set_flg) {
      PrintInfo("current lidar sync: " + std::to_string(hesai_config.sync));
      PrintInfo("current lidar sync_angle: " + std::to_string(sync_angle));
      PrintInfo("current configuration scan_phase: " + std::to_string(scan_phase));
      std::thread t([this, sync_flg, scan_phase] { SetSyncAngle(sync_flg, scan_phase); });
      t.join();
      std::this_thread::sleep_for(wait_time);
    }

    std::thread t([this, sensor_configuration] {
      if (
        sensor_configuration->sensor_model == SensorModel::HESAI_PANDAR40P ||
        sensor_configuration->sensor_model == SensorModel::HESAI_PANDAR64 ||
        sensor_configuration->sensor_model == SensorModel::HESAI_PANDARQT64 ||
        sensor_configuration->sensor_model == SensorModel::HESAI_PANDARXT32 ||
        sensor_configuration->sensor_model == SensorModel::HESAI_PANDARXT32M) {
        PrintInfo("Trying to set Clock source to PTP");
        SetClockSource(HESAI_LIDAR_PTP_CLOCK_SOURCE);
      }
      std::ostringstream tmp_ostringstream;
      tmp_ostringstream << "Trying to set PTP Config: " << sensor_configuration->ptp_profile
                        << ", Domain: " << std::to_string(sensor_configuration->ptp_domain)
                        << ", Transport: " << sensor_configuration->ptp_transport_type
                        << ", Switch Type: " << sensor_configuration->ptp_switch_type << " via TCP";
      PrintInfo(tmp_ostringstream.str());
      SetPtpConfig(
        static_cast<int>(sensor_configuration->ptp_profile), sensor_configuration->ptp_domain,
        static_cast<int>(sensor_configuration->ptp_transport_type),
        static_cast<int>(sensor_configuration->ptp_switch_type), PTP_LOG_ANNOUNCE_INTERVAL,
        PTP_SYNC_INTERVAL, PTP_LOG_MIN_DELAY_INTERVAL);
      PrintDebug("Setting properties done");
    });
    PrintDebug("Waiting for thread to finish");

    t.join();
    PrintDebug("Thread finished");

    std::this_thread::sleep_for(wait_time);
  } else {  // AT128 only supports PTP setup via HTTP
    PrintInfo("Trying to set SyncAngle via HTTP");
    SetSyncAngleSyncHttp(1, static_cast<int>(sensor_configuration->scan_phase));
    std::ostringstream tmp_ostringstream;
    tmp_ostringstream << "Trying to set PTP Config: " << sensor_configuration->ptp_profile
                      << ", Domain: " << sensor_configuration->ptp_domain
                      << ", Transport: " << sensor_configuration->ptp_transport_type << " via HTTP";
    PrintInfo(tmp_ostringstream.str());
    SetPtpConfigSyncHttp(
      static_cast<int>(sensor_configuration->ptp_profile), sensor_configuration->ptp_domain,
      static_cast<int>(sensor_configuration->ptp_transport_type), PTP_LOG_ANNOUNCE_INTERVAL,
      PTP_SYNC_INTERVAL, PTP_LOG_MIN_DELAY_INTERVAL);
  }

#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "End CheckAndSetConfig(HesaiConfig)!!" << std::endl;
#endif
  PrintDebug("GetAndCheckConfig(HesaiConfig) finished");

  return Status::OK;
}

HesaiStatus HesaiHwInterface::CheckAndSetConfig(
  std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration,
  HesaiLidarRangeAll hesai_lidar_range_all)
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "Start CheckAndSetConfig(HesaiLidarRangeAll)!!" << std::endl;
#endif
  //*
  // PTC_COMMAND_SET_LIDAR_RANGE
  bool set_flg = false;
  if (hesai_lidar_range_all.method != 0) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "current hesai_lidar_range_all.method: " << hesai_lidar_range_all.method
              << std::endl;
#endif
    set_flg = true;
  } else {
    auto current_cloud_min_angle = hesai_lidar_range_all.start;
    if (
      static_cast<int>(sensor_configuration->cloud_min_angle * 10) !=
      current_cloud_min_angle.value()) {
      set_flg = true;
      PrintInfo(
        "current lidar range.start: " +
        std::to_string(static_cast<int>(current_cloud_min_angle.value())));
      PrintInfo(
        "current configuration cloud_min_angle: " +
        std::to_string(sensor_configuration->cloud_min_angle));
    }

    auto current_cloud_max_angle = hesai_lidar_range_all.end;
    if (
      static_cast<int>(sensor_configuration->cloud_max_angle * 10) !=
      current_cloud_max_angle.value()) {
      set_flg = true;
      PrintInfo(
        "current lidar range.end: " +
        std::to_string(static_cast<int>(current_cloud_max_angle.value())));
      PrintInfo(
        "current configuration cloud_max_angle: " +
        std::to_string(sensor_configuration->cloud_max_angle));
    }
  }

  if (set_flg) {
    std::thread t([this, sensor_configuration] {
      SetLidarRange(
        static_cast<int>(sensor_configuration->cloud_min_angle * 10),
        static_cast<int>(sensor_configuration->cloud_max_angle * 10)  //,
                                                                      //      false
      );
    });
    t.join();
  }

#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "End CheckAndSetConfig(HesaiLidarRangeAll)!!" << std::endl;
#endif
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

HesaiStatus HesaiHwInterface::CheckAndSetConfig()
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "Start CheckAndSetConfig!!" << std::endl;
#endif
  std::thread t([this] {
    auto result = GetConfig();

    std::stringstream ss;
    ss << result;
    PrintInfo(ss.str());
    CheckAndSetConfig(
      std::static_pointer_cast<const HesaiSensorConfiguration>(sensor_configuration_), result);
  });
  t.join();

  std::thread t2([this] {
    auto result = GetLidarRange();
    std::stringstream ss;
    ss << result;
    PrintInfo(ss.str());
    CheckAndSetConfig(
      std::static_pointer_cast<const HesaiSensorConfiguration>(sensor_configuration_), result);
  });
  t2.join();
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "End CheckAndSetConfig!!" << std::endl;
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
int HesaiHwInterface::NebulaModelToHesaiModelNo(nebula::drivers::SensorModel model)
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
void HesaiHwInterface::SetTargetModel(int model)
{
  target_model_no = model;
}
void HesaiHwInterface::SetTargetModel(nebula::drivers::SensorModel model)
{
  target_model_no = NebulaModelToHesaiModelNo(model);
}

bool HesaiHwInterface::UseHttpSetSpinRate(int model)
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
bool HesaiHwInterface::UseHttpSetSpinRate()
{
  return UseHttpSetSpinRate(target_model_no);
}
bool HesaiHwInterface::UseHttpGetLidarMonitor(int model)
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
bool HesaiHwInterface::UseHttpGetLidarMonitor()
{
  return UseHttpGetLidarMonitor(target_model_no);
}

void HesaiHwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger = logger;
}

void HesaiHwInterface::PrintInfo(std::string info)
{
  if (parent_node_logger) {
    RCLCPP_INFO_STREAM((*parent_node_logger), info);
  } else {
    std::cout << info << std::endl;
  }
}

void HesaiHwInterface::PrintError(std::string error)
{
  if (parent_node_logger) {
    RCLCPP_ERROR_STREAM((*parent_node_logger), error);
  } else {
    std::cerr << error << std::endl;
  }
}

void HesaiHwInterface::PrintDebug(std::string debug)
{
  if (parent_node_logger) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

void HesaiHwInterface::PrintDebug(const std::vector<uint8_t> & bytes)
{
  std::stringstream ss;
  for (const auto & b : bytes) {
    ss << static_cast<int>(b) << ", ";
  }
  ss << std::endl;
  PrintDebug(ss.str());
}

std::string HesaiHwInterface::PrettyPrintPTCError(ptc_error_t error_code)
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
    case PTC_ERROR_CODE_NO_ERROR:
      break;
    case PTC_ERROR_CODE_INVALID_INPUT_PARAM:
      ss << "Invalid input parameter";
      break;
    case PTC_ERROR_CODE_SERVER_CONN_FAILED:
      ss << "Failure to connect to server";
      break;
    case PTC_ERROR_CODE_INVALID_DATA:
      ss << "No valid data returned";
      break;
    case PTC_ERROR_CODE_OUT_OF_MEMORY:
      ss << "Server does not have enough memory";
      break;
    case PTC_ERROR_CODE_UNSUPPORTED_CMD:
      ss << "Server does not support this command yet";
      break;
    case PTC_ERROR_CODE_FPGA_COMM_FAILED:
      ss << "Server failed to communicate with FPGA";
      break;
    case PTC_ERROR_CODE_OTHER:
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

  if (error_flags & TCP_ERROR_INCOMPLETE_RESPONSE) {
    nebula_errors.emplace_back("Incomplete response payload");
  }
  if (error_flags & TCP_ERROR_TIMEOUT) {
    nebula_errors.emplace_back("Request timeout");
  }
  if (error_flags & TCP_ERROR_UNEXPECTED_PAYLOAD) {
    nebula_errors.emplace_back("Received payload but expected payload length 0");
  }
  if (error_flags & TCP_ERROR_UNRELATED_RESPONSE) {
    nebula_errors.emplace_back("Received unrelated response");
  }

  ss << boost::algorithm::join(nebula_errors, ", ");

  return ss.str();
}

template <typename T>
T HesaiHwInterface::CheckSizeAndParse(const std::vector<uint8_t> & data)
{
  if (data.size() < sizeof(T)) {
    throw std::runtime_error("Attempted to parse too-small payload");
  } else if (data.size() > sizeof(T)) {
    PrintError("Sensor returned longer payload than expected. Will parse anyway.");
  }

  T parsed;
  memcpy(&parsed, data.data(), sizeof(T));
  return parsed;
}

}  // namespace nebula::drivers
