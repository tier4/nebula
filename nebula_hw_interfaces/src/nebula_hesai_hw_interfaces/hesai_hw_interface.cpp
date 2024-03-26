#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp"

// #define WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE

#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
#include <chrono>
#include <ctime>
#endif

#include <boost/asio.hpp>

namespace nebula
{
namespace drivers
{
HesaiHwInterface::HesaiHwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  m_owned_ctx{new boost::asio::io_context(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  tcp_driver_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx)},
  scan_cloud_ptr_{std::make_unique<pandar_msgs::msg::PandarScan>()}
{
}
HesaiHwInterface::~HesaiHwInterface()
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << ".......................st: HesaiHwInterface::~HesaiHwInterface()" << std::endl;
#endif
  if (tcp_driver_) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << ".......................tcp_driver_ is available" << std::endl;
#endif
    if (tcp_driver_ && tcp_driver_->isOpen()) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      std::cout << ".......................st: tcp_driver_->close();" << std::endl;
#endif
      tcp_driver_->close();
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      std::cout << ".......................ed: tcp_driver_->close();" << std::endl;
#endif
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << ".......................ed: if(tcp_driver_)" << std::endl;
#endif
  }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << ".......................ed: HesaiHwInterface::~HesaiHwInterface()" << std::endl;
#endif
}

std::shared_ptr<std::vector<uint8_t>> HesaiHwInterface::SendReceive(
  const uint8_t command_id, const std::vector<uint8_t> & payload)
{
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

  auto recv_buf = std::make_shared<std::vector<uint8_t>>();
  bool success = false;

  std::stringstream ss;
  ss << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(command_id) << " (" << len << ") ";
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
    [this, log_tag, &success](const std::vector<uint8_t> & header_bytes) {
      size_t payload_len = (header_bytes[4] << 24) | (header_bytes[5] << 16) | (header_bytes[6] << 8) | header_bytes[7];
      PrintDebug(log_tag + "Received header (expecting " + std::to_string(payload_len) + "B payload)");
      if (payload_len == 0) { success = true; }
    },
    [this, log_tag, &recv_buf, &success](const std::vector<uint8_t> & payload_bytes) {
      PrintDebug(log_tag + "Received payload");

      // Header had payload length 0 (thus, header callback processed request successfully already),
      // but we still received a payload: invalid state
      if (success == true) {
        throw std::runtime_error("Received payload despite payload length 0 in header");
      }

      // Skip 8 header bytes
      recv_buf->insert(recv_buf->end(), std::next(payload_bytes.begin(), 8), payload_bytes.end());
      success = true;
    },
    [this, log_tag, &tm]() {
      PrintDebug(log_tag + "Unlocking mutex");
      tm.unlock(); 
      PrintDebug(log_tag + "Unlocked mutex");
    });
  this->IOContextRun();
  if (!tm.try_lock_for(std::chrono::seconds(1))) {
    PrintError(log_tag + "Request did not finish within 1s");
    return nullptr;
  }

  if (!success) {
    PrintError(log_tag + "Did not receive response");
    return nullptr;
  }

  PrintDebug(log_tag + "Received response");
  return recv_buf;
}

Status HesaiHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  HesaiStatus status = Status::OK;
  mtu_size_ = MTU_SIZE;
  is_solid_state = false;
  try {
    sensor_configuration_ =
      std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration);
    if (
      sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR40P ||
      sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR40P) {
      azimuth_index_ = 2;
      is_valid_packet_ = [](size_t packet_size) {
        return (
          packet_size == PANDAR40_PACKET_SIZE || packet_size == PANDAR40P_EXTENDED_PACKET_SIZE);
      };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARQT64) {
      azimuth_index_ = 12;  // 12 + 258 * [0-3]
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == PANDARQT64_PACKET_SIZE); };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARQT128) {
      azimuth_index_ = 12;  // 12 + 514 * [0-1]
      is_valid_packet_ = [](size_t packet_size) {
        return (packet_size == PANDARQT128_PACKET_SIZE);
      };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARXT32) {
      azimuth_index_ = 12;  // 12 + 130 * [0-7]
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == PANDARXT32_PACKET_SIZE); };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARXT32M) {
      azimuth_index_ = 12;  // 12 + 130 * [0-7]
      is_valid_packet_ = [](size_t packet_size) {
        return (packet_size == PANDARXT32M_PACKET_SIZE);
      };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARAT128) {
      azimuth_index_ = 12;  // 12 + 4 * 128 * [0-1]
      is_solid_state = true;
      is_valid_packet_ = [](size_t packet_size) {
        return (packet_size == PANDARAT128_PACKET_SIZE);
      };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR64) {
      azimuth_index_ = 8;  // 8 + 192 * [0-5]
      is_valid_packet_ = [](size_t packet_size) {
        return (
          packet_size == PANDAR64_PACKET_SIZE || packet_size == PANDAR64_EXTENDED_PACKET_SIZE);
      };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR128_E4X) {
      azimuth_index_ = 12;  // 12
      is_valid_packet_ = [](size_t packet_size) {
        return (
          packet_size == PANDAR128_E4X_EXTENDED_PACKET_SIZE ||
          packet_size == PANDAR128_E4X_PACKET_SIZE);
      };
    } else {
      status = Status::INVALID_SENSOR_MODEL;
    }
  } catch (const std::exception & ex) {
    status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << status << std::endl;
    return status;
  }
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
  std::function<void(std::unique_ptr<pandar_msgs::msg::PandarScan>)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
  return Status::OK;
}

void HesaiHwInterface::ReceiveSensorPacketCallback(const std::vector<uint8_t> & buffer)
{
  int scan_phase = static_cast<int>(sensor_configuration_->scan_phase * 100.0);
  if (!is_valid_packet_(buffer.size())) {
    PrintDebug("Invalid Packet: " + std::to_string(buffer.size()));
    return;
  }
  const uint32_t buffer_size = buffer.size();
  pandar_msgs::msg::PandarPacket pandar_packet;
  std::copy_n(std::make_move_iterator(buffer.begin()), buffer_size, pandar_packet.data.begin());
  pandar_packet.size = buffer_size;
  auto now = std::chrono::system_clock::now();
  auto now_secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  auto now_nanosecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  pandar_packet.stamp.sec = static_cast<int>(now_secs);
  pandar_packet.stamp.nanosec = static_cast<std::uint32_t>(now_nanosecs % 1'000'000'000);
  scan_cloud_ptr_->packets.emplace_back(pandar_packet);

  int current_phase = 0;
  bool comp_flg = false;

  const auto & data = scan_cloud_ptr_->packets.back().data;
  current_phase = (data[azimuth_index_] & 0xff) + ((data[azimuth_index_ + 1] & 0xff) << 8);
  if (is_solid_state) {
    current_phase = (static_cast<int>(current_phase) + 36000 - 0) % 12000;
    if (current_phase >= prev_phase_ || scan_cloud_ptr_->packets.size() < 2) {
      prev_phase_ = current_phase;
    } else {
      comp_flg = true;
    }
  } else {
    current_phase = (static_cast<int>(current_phase) + 36000 - scan_phase) % 36000;

    if (current_phase >= prev_phase_ || scan_cloud_ptr_->packets.size() < 2) {
      prev_phase_ = current_phase;
    } else {
      comp_flg = true;
    }
  }

  if (comp_flg) {  // Scan complete
    if (scan_reception_callback_) {
      scan_cloud_ptr_->header.stamp = scan_cloud_ptr_->packets.front().stamp;
      // Callback
      scan_reception_callback_(std::move(scan_cloud_ptr_));
      scan_cloud_ptr_ = std::make_unique<pandar_msgs::msg::PandarScan>();
    }
  }
}
Status HesaiHwInterface::SensorInterfaceStop()
{
  return Status::ERROR_1;
}

Status HesaiHwInterface::GetSensorConfiguration(SensorConfigurationBase & sensor_configuration)
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
    tcp_driver_->close();
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
  auto response_ptr = SendReceive(PTC_COMMAND_GET_LIDAR_CALIBRATION);
  return std::vector<uint8_t>(*response_ptr);
}

std::string HesaiHwInterface::GetLidarCalibrationString()
{
  auto response_ptr = SendReceive(PTC_COMMAND_GET_LIDAR_CALIBRATION);
  std::string calib_string(response_ptr->begin(), response_ptr->end());
  return calib_string;
}

HesaiPtpDiagStatus HesaiHwInterface::GetPtpDiagStatus()
{
  auto response_ptr = SendReceive(PTC_COMMAND_PTP_DIAGNOSTICS, {PTC_COMMAND_PTP_STATUS});
  auto & response = *response_ptr;

  HesaiPtpDiagStatus hesai_ptp_diag_status{};
  int payload_pos = 0;
  hesai_ptp_diag_status.master_offset = static_cast<long long>(response[payload_pos++]) << 56;
  hesai_ptp_diag_status.master_offset =
    hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 48;
  hesai_ptp_diag_status.master_offset =
    hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 40;
  hesai_ptp_diag_status.master_offset =
    hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 32;
  hesai_ptp_diag_status.master_offset =
    hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 24;
  hesai_ptp_diag_status.master_offset =
    hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 16;
  hesai_ptp_diag_status.master_offset =
    hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 8;
  hesai_ptp_diag_status.master_offset =
    hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]);
  hesai_ptp_diag_status.ptp_state = response[payload_pos++] << 24;
  hesai_ptp_diag_status.ptp_state = hesai_ptp_diag_status.ptp_state | response[payload_pos++] << 16;
  hesai_ptp_diag_status.ptp_state = hesai_ptp_diag_status.ptp_state | response[payload_pos++] << 8;
  hesai_ptp_diag_status.ptp_state = hesai_ptp_diag_status.ptp_state | response[payload_pos++];
  hesai_ptp_diag_status.elapsed_millisec = response[payload_pos++] << 24;
  hesai_ptp_diag_status.elapsed_millisec =
    hesai_ptp_diag_status.elapsed_millisec | response[payload_pos++] << 16;
  hesai_ptp_diag_status.elapsed_millisec =
    hesai_ptp_diag_status.elapsed_millisec | response[payload_pos++] << 8;
  hesai_ptp_diag_status.elapsed_millisec =
    hesai_ptp_diag_status.elapsed_millisec | response[payload_pos++];

  std::stringstream ss;
  ss << "HesaiHwInterface::GetPtpDiagStatus: " << hesai_ptp_diag_status;
  PrintInfo(ss.str());

  return hesai_ptp_diag_status;
}

HesaiPtpDiagPort HesaiHwInterface::GetPtpDiagPort()
{
  auto response_ptr = SendReceive(PTC_COMMAND_PTP_DIAGNOSTICS, {PTC_COMMAND_PTP_PORT_DATA_SET});
  auto & response = *response_ptr;

  HesaiPtpDiagPort hesai_ptp_diag_port;
  int payload_pos = 0;

  for (size_t i = 0; i < hesai_ptp_diag_port.portIdentity.size(); i++) {
    hesai_ptp_diag_port.portIdentity[i] = response[payload_pos++];
  }
  hesai_ptp_diag_port.portState = static_cast<int>(response[payload_pos++]);
  hesai_ptp_diag_port.logMinDelayReqInterval = static_cast<int>(response[payload_pos++]);
  hesai_ptp_diag_port.peerMeanPathDelay = static_cast<long long>(response[payload_pos++]) << 56;
  hesai_ptp_diag_port.peerMeanPathDelay =
    hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 48;
  hesai_ptp_diag_port.peerMeanPathDelay =
    hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 40;
  hesai_ptp_diag_port.peerMeanPathDelay =
    hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 32;
  hesai_ptp_diag_port.peerMeanPathDelay =
    hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 24;
  hesai_ptp_diag_port.peerMeanPathDelay =
    hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 16;
  hesai_ptp_diag_port.peerMeanPathDelay =
    hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 8;
  hesai_ptp_diag_port.peerMeanPathDelay =
    hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]);
  hesai_ptp_diag_port.logAnnounceInterval = static_cast<int>(response[payload_pos++]);
  hesai_ptp_diag_port.announceReceiptTimeout = static_cast<int>(response[payload_pos++]);
  hesai_ptp_diag_port.logSyncInterval = static_cast<int>(response[payload_pos++]);
  hesai_ptp_diag_port.delayMechanism = static_cast<int>(response[payload_pos++]);
  hesai_ptp_diag_port.logMinPdelayReqInterval = static_cast<int>(response[payload_pos++]);
  hesai_ptp_diag_port.versionNumber = static_cast<int>(response[payload_pos++]);

  std::stringstream ss;
  ss << "HesaiHwInterface::GetPtpDiagPort: " << hesai_ptp_diag_port;
  PrintInfo(ss.str());

  return hesai_ptp_diag_port;
}

HesaiPtpDiagTime HesaiHwInterface::GetPtpDiagTime()
{
  auto response_ptr = SendReceive(PTC_COMMAND_PTP_DIAGNOSTICS, {PTC_COMMAND_PTP_TIME_STATUS_NP});
  auto & response = *response_ptr;

  HesaiPtpDiagTime hesai_ptp_diag_time;
  int payload_pos = 0;
  hesai_ptp_diag_time.master_offset = static_cast<long long>(response[payload_pos++]) << 56;
  hesai_ptp_diag_time.master_offset =
    hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 48;
  hesai_ptp_diag_time.master_offset =
    hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 40;
  hesai_ptp_diag_time.master_offset =
    hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 32;
  hesai_ptp_diag_time.master_offset =
    hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 24;
  hesai_ptp_diag_time.master_offset =
    hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 16;
  hesai_ptp_diag_time.master_offset =
    hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 8;
  hesai_ptp_diag_time.master_offset =
    hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]);
  hesai_ptp_diag_time.ingress_time = static_cast<long long>(response[payload_pos++]) << 56;
  hesai_ptp_diag_time.ingress_time =
    hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 48;
  hesai_ptp_diag_time.ingress_time =
    hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 40;
  hesai_ptp_diag_time.ingress_time =
    hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 32;
  hesai_ptp_diag_time.ingress_time =
    hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 24;
  hesai_ptp_diag_time.ingress_time =
    hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 16;
  hesai_ptp_diag_time.ingress_time =
    hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 8;
  hesai_ptp_diag_time.ingress_time =
    hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]);
  hesai_ptp_diag_time.cumulativeScaledRateOffset = response[payload_pos++] << 24;
  hesai_ptp_diag_time.cumulativeScaledRateOffset =
    hesai_ptp_diag_time.cumulativeScaledRateOffset | response[payload_pos++] << 16;
  hesai_ptp_diag_time.cumulativeScaledRateOffset =
    hesai_ptp_diag_time.cumulativeScaledRateOffset | response[payload_pos++] << 8;
  hesai_ptp_diag_time.cumulativeScaledRateOffset =
    hesai_ptp_diag_time.cumulativeScaledRateOffset | response[payload_pos++];
  hesai_ptp_diag_time.scaledLastGmPhaseChange = response[payload_pos++] << 24;
  hesai_ptp_diag_time.scaledLastGmPhaseChange =
    hesai_ptp_diag_time.scaledLastGmPhaseChange | response[payload_pos++] << 16;
  hesai_ptp_diag_time.scaledLastGmPhaseChange =
    hesai_ptp_diag_time.scaledLastGmPhaseChange | response[payload_pos++] << 8;
  hesai_ptp_diag_time.scaledLastGmPhaseChange =
    hesai_ptp_diag_time.scaledLastGmPhaseChange | response[payload_pos++];
  hesai_ptp_diag_time.gmTimeBaseIndicator = response[payload_pos++] << 8;
  hesai_ptp_diag_time.gmTimeBaseIndicator =
    hesai_ptp_diag_time.gmTimeBaseIndicator | response[payload_pos++];
  for (size_t i = 0; i < hesai_ptp_diag_time.lastGmPhaseChange.size(); i++) {
    hesai_ptp_diag_time.lastGmPhaseChange[i] = response[payload_pos++];
  }
  hesai_ptp_diag_time.gmPresent = response[payload_pos++] << 24;
  hesai_ptp_diag_time.gmPresent = hesai_ptp_diag_time.gmPresent | response[payload_pos++] << 16;
  hesai_ptp_diag_time.gmPresent = hesai_ptp_diag_time.gmPresent | response[payload_pos++] << 8;
  hesai_ptp_diag_time.gmPresent = hesai_ptp_diag_time.gmPresent | response[payload_pos++];
  hesai_ptp_diag_time.gmIdentity = static_cast<long long>(response[payload_pos++]) << 56;
  hesai_ptp_diag_time.gmIdentity =
    hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 48;
  hesai_ptp_diag_time.gmIdentity =
    hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 40;
  hesai_ptp_diag_time.gmIdentity =
    hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 32;
  hesai_ptp_diag_time.gmIdentity =
    hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 24;
  hesai_ptp_diag_time.gmIdentity =
    hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 16;
  hesai_ptp_diag_time.gmIdentity =
    hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 8;
  hesai_ptp_diag_time.gmIdentity =
    hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]);

  std::stringstream ss;
  ss << "HesaiHwInterface::GetPtpDiagTime: " << hesai_ptp_diag_time;
  PrintInfo(ss.str());

  return hesai_ptp_diag_time;
}

HesaiPtpDiagGrandmaster HesaiHwInterface::GetPtpDiagGrandmaster()
{
  auto response_ptr =
    SendReceive(PTC_COMMAND_PTP_DIAGNOSTICS, {PTC_COMMAND_PTP_GRANDMASTER_SETTINGS_NP});
  auto & response = *response_ptr;

  HesaiPtpDiagGrandmaster hesai_ptp_diag_grandmaster;
  int payload_pos = 0;

  hesai_ptp_diag_grandmaster.clockQuality = response[payload_pos++] << 24;
  hesai_ptp_diag_grandmaster.clockQuality =
    hesai_ptp_diag_grandmaster.clockQuality | response[payload_pos++] << 16;
  hesai_ptp_diag_grandmaster.clockQuality =
    hesai_ptp_diag_grandmaster.clockQuality | response[payload_pos++] << 8;
  hesai_ptp_diag_grandmaster.clockQuality =
    hesai_ptp_diag_grandmaster.clockQuality | response[payload_pos++];
  hesai_ptp_diag_grandmaster.utc_offset = response[payload_pos++] << 8;
  hesai_ptp_diag_grandmaster.utc_offset =
    hesai_ptp_diag_grandmaster.utc_offset | response[payload_pos++];
  hesai_ptp_diag_grandmaster.time_flags = static_cast<int>(response[payload_pos++]);
  hesai_ptp_diag_grandmaster.time_source = static_cast<int>(response[payload_pos++]);

  std::stringstream ss;
  ss << "HesaiHwInterface::GetPtpDiagGrandmaster: " << hesai_ptp_diag_grandmaster;
  PrintInfo(ss.str());

  return hesai_ptp_diag_grandmaster;
}

HesaiInventory HesaiHwInterface::GetInventory()
{
  auto response_ptr = SendReceive(PTC_COMMAND_GET_INVENTORY_INFO);
  auto & response = *response_ptr;

  HesaiInventory hesai_inventory;
  int payload_pos = 0;
  for (size_t i = 0; i < hesai_inventory.sn.size(); i++) {
    hesai_inventory.sn[i] = response[payload_pos++];
  }
  for (size_t i = 0; i < hesai_inventory.date_of_manufacture.size(); i++) {
    hesai_inventory.date_of_manufacture[i] = response[payload_pos++];
  }
  for (size_t i = 0; i < hesai_inventory.mac.size(); i++) {
    hesai_inventory.mac[i] = response[payload_pos++];
  }
  for (size_t i = 0; i < hesai_inventory.sw_ver.size(); i++) {
    hesai_inventory.sw_ver[i] = response[payload_pos++];
  }
  for (size_t i = 0; i < hesai_inventory.hw_ver.size(); i++) {
    hesai_inventory.hw_ver[i] = response[payload_pos++];
  }
  for (size_t i = 0; i < hesai_inventory.control_fw_ver.size(); i++) {
    hesai_inventory.control_fw_ver[i] = response[payload_pos++];
  }
  for (size_t i = 0; i < hesai_inventory.sensor_fw_ver.size(); i++) {
    hesai_inventory.sensor_fw_ver[i] = response[payload_pos++];
  }
  hesai_inventory.angle_offset = response[payload_pos++] << 8;
  hesai_inventory.angle_offset = hesai_inventory.angle_offset | response[payload_pos++];
  hesai_inventory.model = static_cast<int>(response[payload_pos++]);
  hesai_inventory.motor_type = static_cast<int>(response[payload_pos++]);
  hesai_inventory.num_of_lines = static_cast<int>(response[payload_pos++]);
  for (size_t i = 0; i < hesai_inventory.reserved.size(); i++) {
    hesai_inventory.reserved[i] = static_cast<unsigned char>(response[payload_pos++]);
  }

  return hesai_inventory;
}

HesaiConfig HesaiHwInterface::GetConfig()
{
  auto response_ptr = SendReceive(PTC_COMMAND_GET_CONFIG_INFO);
  auto & response = *response_ptr;

  HesaiConfig hesai_config{};
  int payload_pos = 0;
  hesai_config.ipaddr[0] = static_cast<int>(response[payload_pos++]);
  hesai_config.ipaddr[1] = static_cast<int>(response[payload_pos++]);
  hesai_config.ipaddr[2] = static_cast<int>(response[payload_pos++]);
  hesai_config.ipaddr[3] = static_cast<int>(response[payload_pos++]);
  hesai_config.mask[0] = static_cast<int>(response[payload_pos++]);
  hesai_config.mask[1] = static_cast<int>(response[payload_pos++]);
  hesai_config.mask[2] = static_cast<int>(response[payload_pos++]);
  hesai_config.mask[3] = static_cast<int>(response[payload_pos++]);
  hesai_config.gateway[0] = static_cast<int>(response[payload_pos++]);
  hesai_config.gateway[1] = static_cast<int>(response[payload_pos++]);
  hesai_config.gateway[2] = static_cast<int>(response[payload_pos++]);
  hesai_config.gateway[3] = static_cast<int>(response[payload_pos++]);
  hesai_config.dest_ipaddr[0] = static_cast<int>(response[payload_pos++]);
  hesai_config.dest_ipaddr[1] = static_cast<int>(response[payload_pos++]);
  hesai_config.dest_ipaddr[2] = static_cast<int>(response[payload_pos++]);
  hesai_config.dest_ipaddr[3] = static_cast<int>(response[payload_pos++]);
  hesai_config.dest_LiDAR_udp_port = response[payload_pos++] << 8;
  hesai_config.dest_LiDAR_udp_port = hesai_config.dest_LiDAR_udp_port | response[payload_pos++];
  hesai_config.dest_gps_udp_port = response[payload_pos++] << 8;
  hesai_config.dest_gps_udp_port = hesai_config.dest_gps_udp_port | response[payload_pos++];
  hesai_config.spin_rate = response[payload_pos++] << 8;
  hesai_config.spin_rate = hesai_config.spin_rate | response[payload_pos++];
  hesai_config.sync = static_cast<int>(response[payload_pos++]);
  hesai_config.sync_angle = response[payload_pos++] << 8;
  hesai_config.sync_angle = hesai_config.sync_angle | response[payload_pos++];
  hesai_config.start_angle = response[payload_pos++] << 8;
  hesai_config.start_angle = hesai_config.start_angle | response[payload_pos++];
  hesai_config.stop_angle = response[payload_pos++] << 8;
  hesai_config.stop_angle = hesai_config.stop_angle | response[payload_pos++];
  hesai_config.clock_source = static_cast<int>(response[payload_pos++]);
  hesai_config.udp_seq = static_cast<int>(response[payload_pos++]);
  hesai_config.trigger_method = static_cast<int>(response[payload_pos++]);
  hesai_config.return_mode = static_cast<int>(response[payload_pos++]);
  hesai_config.standby_mode = static_cast<int>(response[payload_pos++]);
  hesai_config.motor_status = static_cast<int>(response[payload_pos++]);
  hesai_config.vlan_flag = static_cast<int>(response[payload_pos++]);
  hesai_config.vlan_id = response[payload_pos++] << 8;
  hesai_config.vlan_id = hesai_config.vlan_id | response[payload_pos++];
  hesai_config.clock_data_fmt = static_cast<int>(response[payload_pos++]);
  hesai_config.noise_filtering = static_cast<int>(response[payload_pos++]);
  hesai_config.reflectivity_mapping = static_cast<int>(response[payload_pos++]);
  hesai_config.reserved[0] = static_cast<unsigned char>(response[payload_pos++]);
  hesai_config.reserved[1] = static_cast<unsigned char>(response[payload_pos++]);
  hesai_config.reserved[2] = static_cast<unsigned char>(response[payload_pos++]);
  hesai_config.reserved[3] = static_cast<unsigned char>(response[payload_pos++]);
  hesai_config.reserved[4] = static_cast<unsigned char>(response[payload_pos++]);
  hesai_config.reserved[5] = static_cast<unsigned char>(response[payload_pos++]);

  std::cout << "Config: " << hesai_config << std::endl;
  return hesai_config;
}

HesaiLidarStatus HesaiHwInterface::GetLidarStatus()
{
  auto response_ptr = SendReceive(PTC_COMMAND_GET_LIDAR_STATUS);
  auto & response = *response_ptr;

  HesaiLidarStatus hesai_status;
  int payload_pos = 0;
  hesai_status.system_uptime = response[payload_pos++] << 24;
  hesai_status.system_uptime = hesai_status.system_uptime | response[payload_pos++] << 16;
  hesai_status.system_uptime = hesai_status.system_uptime | response[payload_pos++] << 8;
  hesai_status.system_uptime = hesai_status.system_uptime | response[payload_pos++];
  hesai_status.motor_speed = response[payload_pos++] << 8;
  hesai_status.motor_speed = hesai_status.motor_speed | response[payload_pos++];
  for (size_t i = 0; i < hesai_status.temperature.size(); i++) {
    hesai_status.temperature[i] = response[payload_pos++] << 24;
    hesai_status.temperature[i] = hesai_status.temperature[i] | response[payload_pos++] << 16;
    hesai_status.temperature[i] = hesai_status.temperature[i] | response[payload_pos++] << 8;
    hesai_status.temperature[i] = hesai_status.temperature[i] | response[payload_pos++];
  }
  hesai_status.gps_pps_lock = static_cast<int>(response[payload_pos++]);
  hesai_status.gps_gprmc_status = static_cast<int>(response[payload_pos++]);
  hesai_status.startup_times = response[payload_pos++] << 24;
  hesai_status.startup_times = hesai_status.startup_times | response[payload_pos++] << 16;
  hesai_status.startup_times = hesai_status.startup_times | response[payload_pos++] << 8;
  hesai_status.startup_times = hesai_status.startup_times | response[payload_pos++];
  hesai_status.total_operation_time = response[payload_pos++] << 24;
  hesai_status.total_operation_time = hesai_status.total_operation_time | response[payload_pos++]
                                                                            << 16;
  hesai_status.total_operation_time = hesai_status.total_operation_time | response[payload_pos++]
                                                                            << 8;
  hesai_status.total_operation_time = hesai_status.total_operation_time | response[payload_pos++];
  hesai_status.ptp_clock_status = static_cast<int>(response[payload_pos++]);
  for (size_t i = 0; i < hesai_status.reserved.size(); i++) {
    hesai_status.reserved[i] = static_cast<unsigned char>(response[payload_pos++]);
  }

  return hesai_status;
}

Status HesaiHwInterface::SetSpinRate(uint16_t rpm)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back((rpm >> 8) & 0xff);
  request_payload.emplace_back(rpm & 0xff);

  SendReceive(PTC_COMMAND_SET_SPIN_RATE, request_payload);
  return Status::OK;
}

Status HesaiHwInterface::SetSyncAngle(int sync_angle, int angle)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(sync_angle & 0xff);
  request_payload.emplace_back((angle >> 8) & 0xff);
  request_payload.emplace_back(angle & 0xff);

  SendReceive(PTC_COMMAND_SET_SYNC_ANGLE, request_payload);
  return Status::OK;
}

Status HesaiHwInterface::SetTriggerMethod(int trigger_method)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(trigger_method & 0xff);

  SendReceive(PTC_COMMAND_SET_TRIGGER_METHOD, request_payload);
  return Status::OK;
}

Status HesaiHwInterface::SetStandbyMode(int standby_mode)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(standby_mode & 0xff);

  SendReceive(PTC_COMMAND_SET_STANDBY_MODE, request_payload);
  return Status::OK;
}

Status HesaiHwInterface::SetReturnMode(int return_mode)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(return_mode & 0xff);

  SendReceive(PTC_COMMAND_SET_RETURN_MODE, request_payload);
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

  SendReceive(PTC_COMMAND_SET_DESTINATION_IP, request_payload);
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

  SendReceive(PTC_COMMAND_SET_CONTROL_PORT, request_payload);
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

  SendReceive(PTC_COMMAND_SET_LIDAR_RANGE, request_payload);
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

  SendReceive(PTC_COMMAND_SET_LIDAR_RANGE, request_payload);
  return Status::OK;
}

HesaiLidarRangeAll HesaiHwInterface::GetLidarRange()
{
  auto response_ptr = SendReceive(PTC_COMMAND_GET_LIDAR_RANGE);
  auto & response = *response_ptr;

  HesaiLidarRangeAll hesai_range_all;
  int payload_pos = 0;
  int method = static_cast<int>(response[payload_pos++]);
  switch (method) {
    case 0:  // for all channels
      hesai_range_all.method = method;
      hesai_range_all.start = response[payload_pos++] << 8;
      hesai_range_all.start = hesai_range_all.start | response[payload_pos++];
      hesai_range_all.end = response[payload_pos++] << 8;
      hesai_range_all.end = hesai_range_all.end | response[payload_pos++];
      break;
    case 1:  // for each channel
      hesai_range_all.method = method;
      break;
    case 2:  // multi-section FOV
      hesai_range_all.method = method;
      break;
  }

  return hesai_range_all;
}

Status HesaiHwInterface::SetClockSource(int clock_source)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(clock_source & 0xff);

  SendReceive(PTC_COMMAND_SET_CLOCK_SOURCE, request_payload);
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

  SendReceive(PTC_COMMAND_SET_PTP_CONFIG, request_payload);
  return Status::OK;
}

HesaiPtpConfig HesaiHwInterface::GetPtpConfig()
{
  auto response_ptr = SendReceive(PTC_COMMAND_GET_PTP_CONFIG);
  auto & response = *response_ptr;

  HesaiPtpConfig hesai_ptp_config{};

  int payload_pos = 0;
  hesai_ptp_config.status = static_cast<int>(response[payload_pos++]);
  hesai_ptp_config.profile = static_cast<int>(response[payload_pos++]);
  hesai_ptp_config.domain = static_cast<int>(response[payload_pos++]);
  hesai_ptp_config.network = static_cast<int>(response[payload_pos++]);
  if (hesai_ptp_config.status == 0) {
    hesai_ptp_config.logAnnounceInterval = static_cast<int>(response[payload_pos++]);
    hesai_ptp_config.logSyncInterval = static_cast<int>(response[payload_pos++]);
    hesai_ptp_config.logMinDelayReqInterval = static_cast<int>(response[payload_pos++]);
  }

  std::stringstream ss;
  ss << "HesaiHwInterface::GetPtpConfig: " << hesai_ptp_config;
  PrintInfo(ss.str());

  return hesai_ptp_config;
}

Status HesaiHwInterface::SendReset()
{
  SendReceive(PTC_COMMAND_RESET);
  return Status::OK;
}

Status HesaiHwInterface::SetRotDir(int mode)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(mode & 0xff);

  SendReceive(PTC_COMMAND_SET_ROTATE_DIRECTION, request_payload);
  return Status::OK;
}

HesaiLidarMonitor HesaiHwInterface::GetLidarMonitor()
{
  auto response_ptr = SendReceive(PTC_COMMAND_LIDAR_MONITOR);
  auto & response = *response_ptr;

  HesaiLidarMonitor hesai_lidar_monitor;
  int payload_pos = 0;
  hesai_lidar_monitor.input_voltage = response[payload_pos++] << 24;
  hesai_lidar_monitor.input_voltage = hesai_lidar_monitor.input_voltage | response[payload_pos++]
                                                                            << 16;
  hesai_lidar_monitor.input_voltage = hesai_lidar_monitor.input_voltage | response[payload_pos++]
                                                                            << 8;
  hesai_lidar_monitor.input_voltage = hesai_lidar_monitor.input_voltage | response[payload_pos++];
  hesai_lidar_monitor.input_current = response[payload_pos++] << 24;
  hesai_lidar_monitor.input_current = hesai_lidar_monitor.input_current | response[payload_pos++]
                                                                            << 16;
  hesai_lidar_monitor.input_current = hesai_lidar_monitor.input_current | response[payload_pos++]
                                                                            << 8;
  hesai_lidar_monitor.input_current = hesai_lidar_monitor.input_current | response[payload_pos++];
  hesai_lidar_monitor.input_power = response[payload_pos++] << 24;
  hesai_lidar_monitor.input_power = hesai_lidar_monitor.input_power | response[payload_pos++] << 16;
  hesai_lidar_monitor.input_power = hesai_lidar_monitor.input_power | response[payload_pos++] << 8;
  hesai_lidar_monitor.input_power = hesai_lidar_monitor.input_power | response[payload_pos++];

  for (size_t i = 0; i < hesai_lidar_monitor.reserved.size(); i++) {
    hesai_lidar_monitor.reserved[i] = static_cast<unsigned char>(response[payload_pos++]);
  }

  return hesai_lidar_monitor;
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
    [this, str_callback](const std::string & str) { str_callback(str); },
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
  std::shared_ptr<HesaiSensorConfiguration> sensor_configuration, HesaiConfig hesai_config)
{
  using namespace std::chrono_literals;
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
  if (sensor_configuration->rotation_speed != current_rotation_speed) {
    PrintInfo("current lidar rotation_speed: " + std::to_string(current_rotation_speed));
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
  ss << hesai_config.dest_ipaddr[0] << "." << hesai_config.dest_ipaddr[1] << "."
     << hesai_config.dest_ipaddr[2] << "." << hesai_config.dest_ipaddr[3];
  auto current_host_addr = ss.str();
  if (sensor_configuration->host_ip != current_host_addr) {
    set_flg = true;
    PrintInfo("current lidar dest_ipaddr: " + current_host_addr);
    PrintInfo("current configuration host_ip: " + sensor_configuration->host_ip);
  }

  auto current_host_dport = hesai_config.dest_LiDAR_udp_port;
  if (sensor_configuration->data_port != current_host_dport) {
    set_flg = true;
    PrintInfo("current lidar dest_LiDAR_udp_port: " + std::to_string(current_host_dport));
    PrintInfo(
      "current configuration data_port: " + std::to_string(sensor_configuration->data_port));
  }

  auto current_host_tport = hesai_config.dest_gps_udp_port;
  if (sensor_configuration->gnss_port != current_host_tport) {
    set_flg = true;
    PrintInfo("current lidar dest_gps_udp_port: " + std::to_string(current_host_tport));
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
    auto sync_angle = static_cast<int>(hesai_config.sync_angle / 100);
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
  std::shared_ptr<HesaiSensorConfiguration> sensor_configuration,
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
    if (static_cast<int>(sensor_configuration->cloud_min_angle * 10) != current_cloud_min_angle) {
      set_flg = true;
      PrintInfo("current lidar range.start: " + std::to_string(current_cloud_min_angle));
      PrintInfo(
        "current configuration cloud_min_angle: " +
        std::to_string(sensor_configuration->cloud_min_angle));
    }

    auto current_cloud_max_angle = hesai_lidar_range_all.end;
    if (static_cast<int>(sensor_configuration->cloud_max_angle * 10) != current_cloud_max_angle) {
      set_flg = true;
      PrintInfo("current lidar range.end: " + std::to_string(current_cloud_max_angle));
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
      std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
  });
  t.join();

  std::thread t2([this] {
    auto result = GetLidarRange();
    std::stringstream ss;
    ss << result;
    PrintInfo(ss.str());
    CheckAndSetConfig(
      std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
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

}  // namespace drivers
}  // namespace nebula
