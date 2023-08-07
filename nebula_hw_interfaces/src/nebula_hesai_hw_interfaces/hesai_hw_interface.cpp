#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp"

namespace nebula
{
namespace drivers
{
HesaiHwInterface::HesaiHwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  m_owned_ctx{new boost::asio::io_context(1)},
  m_owned_ctx_s{new boost::asio::io_context(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  tcp_driver_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx)},
  tcp_driver_s_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx_s)},
  scan_cloud_ptr_{std::make_unique<pandar_msgs::msg::PandarScan>()}
{
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

Status HesaiHwInterface::CloudInterfaceStart()
{
  try {
    std::cout << "Starting UDP server on: " << *sensor_configuration_ << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();

    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&HesaiHwInterface::ReceiveCloudPacketCallback, this, std::placeholders::_1));
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

void HesaiHwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer)
{
  int scan_phase = static_cast<int>(sensor_configuration_->scan_phase * 100.0);
  if (!is_valid_packet_(buffer.size())) {
    PrintDebug("Invalid Packet: " + std::to_string(buffer.size()));
    return;
  }
  uint32_t buffer_size = buffer.size();
  std::array<uint8_t, MTU_SIZE> packet_data{};
  std::copy_n(std::make_move_iterator(buffer.begin()), buffer_size, packet_data.begin());
  pandar_msgs::msg::PandarPacket pandar_packet;
  pandar_packet.data = packet_data;
  pandar_packet.size = buffer_size;
  auto now = std::chrono::system_clock::now();
  auto now_secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  auto now_nanosecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  pandar_packet.stamp.sec = static_cast<int>(now_secs);
  pandar_packet.stamp.nanosec =
    static_cast<int>((now_nanosecs / 1000000000. - static_cast<double>(now_secs)) * 1000000000);
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
Status HesaiHwInterface::CloudInterfaceStop() { return Status::ERROR_1; }

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

Status HesaiHwInterface::InitializeTcpDriver(bool setup_sensor)
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "HesaiHwInterface::InitializeTcpDriver" << std::endl;
#endif
  tcp_driver_->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  if (!tcp_driver_->open()) {
    return Status::ERROR_1;
  }
  if (setup_sensor) {
    tcp_driver_s_->init_socket(
      sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
      PandarTcpCommandPort);
    if (!tcp_driver_s_->open()) {
      return Status::ERROR_1;
    }
  }
  return Status::OK;
}

Status HesaiHwInterface::FinalizeTcpDriver() {
  try {
    tcp_driver_->close();
  }
  catch(std::exception &e) {
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

Status HesaiHwInterface::syncGetLidarCalibration(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
  std::function<void(const std::vector<uint8_t> & bytes)> bytes_callback)
{
  std::vector<unsigned char> buf_vec;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_GET_LIDAR_CALIBRATION);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  //  if (!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetLidarCalibration")) {
  //    return GetLidarCalibration(target_tcp_driver, with_run);
  //  }
  PrintDebug("GetLidarCalibration: start");

  target_tcp_driver->syncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver, bytes_callback](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetLidarCalib getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetLidarCalib getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<char>(b);
      }
      std::cout << std::endl;
#endif
      bytes_callback(received_bytes);
    },
    [this]() { CheckUnlock(tm_, "GetLidarCalibration"); });

  return Status::OK;
}
Status HesaiHwInterface::syncGetLidarCalibration(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
  std::function<void(const std::string & str)> str_callback)
{
  return syncGetLidarCalibration(
    target_tcp_driver, [this, str_callback](const std::vector<uint8_t> & received_bytes) {
      std::string calib_string =
        std::string(received_bytes.data(), received_bytes.data() + received_bytes.size());
      PrintInfo(calib_string);
      str_callback(calib_string);
    });
}
Status HesaiHwInterface::syncGetLidarCalibration(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver)
{
  return syncGetLidarCalibration(
    target_tcp_driver, [this](const std::vector<uint8_t> & received_bytes) {
      std::string calib_string =
        std::string(received_bytes.data(), received_bytes.data() + received_bytes.size());
      PrintInfo(calib_string);
    });
}
Status HesaiHwInterface::syncGetLidarCalibration(
  std::shared_ptr<boost::asio::io_context> ctx,
  std::function<void(const std::string & str)> str_callback)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  return syncGetLidarCalibration(tcp_driver_local, str_callback);
}
Status HesaiHwInterface::syncGetLidarCalibration(std::shared_ptr<boost::asio::io_context> ctx)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  return syncGetLidarCalibration(tcp_driver_local);
}
Status HesaiHwInterface::syncGetLidarCalibrationFromSensor(
  std::function<void(const std::vector<uint8_t> & received_bytes)> bytes_callback)
{
  return syncGetLidarCalibration(tcp_driver_, bytes_callback);
}
Status HesaiHwInterface::syncGetLidarCalibrationFromSensor(
  std::function<void(const std::string & str)> str_callback)
{
  return syncGetLidarCalibration(
    tcp_driver_, [this, str_callback](const std::vector<uint8_t> & received_bytes) {
      std::string calib_string =
        std::string(received_bytes.data(), received_bytes.data() + received_bytes.size());
      str_callback(calib_string);
    });
}
Status HesaiHwInterface::syncGetLidarCalibrationFromSensor()
{
  return syncGetLidarCalibrationFromSensor([this](const std::string & str) { PrintDebug(str); });
}

Status HesaiHwInterface::GetLidarCalibration(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
  std::function<void(const std::vector<uint8_t> & bytes)> bytes_callback, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_GET_LIDAR_CALIBRATION);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  PrintDebug("GetLidarCalibration: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver, bytes_callback](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetLidarCalib getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetLidarCalib getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<char>(b);
      }
      std::cout << std::endl;
#endif
      bytes_callback(received_bytes);
    },
    [this]() { CheckUnlock(tm_, "GetLidarCalibration"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::GetLidarCalibration: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetLidarCalib" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetLidarCalibration(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
  std::function<void(const std::string & str)> str_callback, bool with_run)
{
  return GetLidarCalibration(
    target_tcp_driver,
    [this, str_callback](const std::vector<uint8_t> & received_bytes) {
      std::string calib_string =
        std::string(received_bytes.data(), received_bytes.data() + received_bytes.size());
      PrintInfo(calib_string);
      str_callback(calib_string);
    },
    with_run);
}
Status HesaiHwInterface::GetLidarCalibration(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  return GetLidarCalibration(
    target_tcp_driver,
    [this](const std::vector<uint8_t> & received_bytes) {
      std::string calib_string =
        std::string(received_bytes.data(), received_bytes.data() + received_bytes.size());
      PrintInfo(calib_string);
    },
    with_run);
}
Status HesaiHwInterface::GetLidarCalibration(
  std::shared_ptr<boost::asio::io_context> ctx,
  std::function<void(const std::string & str)> str_callback, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  return GetLidarCalibration(tcp_driver_local, str_callback, with_run);
}
Status HesaiHwInterface::GetLidarCalibration(
  std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  return GetLidarCalibration(tcp_driver_local, with_run);
}
Status HesaiHwInterface::GetLidarCalibrationFromSensor(
  std::function<void(const std::vector<uint8_t> & received_bytes)> bytes_callback, bool with_run)
{
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetLidarCalibration(tcp_driver_, bytes_callback, with_run);
}
Status HesaiHwInterface::GetLidarCalibrationFromSensor(
  std::function<void(const std::string & str)> str_callback, bool with_run)
{
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetLidarCalibration(
    tcp_driver_,
    [this, str_callback](const std::vector<uint8_t> & received_bytes) {
      std::string calib_string =
        std::string(received_bytes.data(), received_bytes.data() + received_bytes.size());
      str_callback(calib_string);
    },
    with_run);
}
Status HesaiHwInterface::GetLidarCalibrationFromSensor(bool with_run)
{
  return GetLidarCalibrationFromSensor(
    [this](const std::string & str) { PrintDebug(str); }, with_run);
}

Status HesaiHwInterface::GetPtpDiagStatus(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_PTP_DIAGNOSTICS);  // Cmd PTC_COMMAND_PTP_DIAGNOSTICS
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);

  buf_vec.emplace_back(PTC_COMMAND_PTP_STATUS);  // PTP STATUS

  if (!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetPtpDiagStatus")) {
    return GetPtpDiagStatus(target_tcp_driver, with_run);
  }
  PrintDebug("GetPtpDiagStatus: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetPtpDiagStatus getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetPtpDiagStatus getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);

      auto response = target_tcp_driver->getPayload();
      HesaiPtpDiagStatus hesai_ptp_diag_status{};
      if (8 < response.size()) {
        int payload_pos = 8;
        hesai_ptp_diag_status.master_offset = static_cast<long long>(response[payload_pos++]) << 56;
        hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset |
                                              static_cast<long long>(response[payload_pos++]) << 48;
        hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset |
                                              static_cast<long long>(response[payload_pos++]) << 40;
        hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset |
                                              static_cast<long long>(response[payload_pos++]) << 32;
        hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset |
                                              static_cast<long long>(response[payload_pos++]) << 24;
        hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset |
                                              static_cast<long long>(response[payload_pos++]) << 16;
        hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset |
                                              static_cast<long long>(response[payload_pos++]) << 8;
        hesai_ptp_diag_status.master_offset =
          hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]);
        hesai_ptp_diag_status.ptp_state = response[payload_pos++] << 24;
        hesai_ptp_diag_status.ptp_state = hesai_ptp_diag_status.ptp_state | response[payload_pos++]
                                                                              << 16;
        hesai_ptp_diag_status.ptp_state = hesai_ptp_diag_status.ptp_state | response[payload_pos++]
                                                                              << 8;
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
      }
    },
    [this]() { CheckUnlock(tm_, "GetPtpDiagStatus"); });

  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::GetPtpDiagStatus: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetPtpDiagStatus" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetPtpDiagStatus(
  std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  return GetPtpDiagStatus(tcp_driver_local, with_run);
}
Status HesaiHwInterface::GetPtpDiagStatus(bool with_run)
{
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetPtpDiagStatus(tcp_driver_, with_run);
}

Status HesaiHwInterface::GetPtpDiagPort(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_PTP_DIAGNOSTICS);  // Cmd PTC_COMMAND_PTP_DIAGNOSTICS
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back(PTC_COMMAND_PTP_PORT_DATA_SET);  // PTP TLV PORT_DATA_SET

  if (!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetPtpDiagPort")) {
    return GetPtpDiagPort(target_tcp_driver, with_run);
  }
  PrintDebug("GetPtpDiagPort: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetPtpDiagPort getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetPtpDiagPort getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);

      auto response = target_tcp_driver->getPayload();
      HesaiPtpDiagPort hesai_ptp_diag_port;
      if (8 < response.size()) {
        int payload_pos = 8;

        for (size_t i = 0; i < hesai_ptp_diag_port.portIdentity.size(); i++) {
          hesai_ptp_diag_port.portIdentity[i] = response[payload_pos++];
        }
        hesai_ptp_diag_port.portState = static_cast<int>(response[payload_pos++]);
        hesai_ptp_diag_port.logMinDelayReqInterval = static_cast<int>(response[payload_pos++]);
        hesai_ptp_diag_port.peerMeanPathDelay = static_cast<long long>(response[payload_pos++])
                                                << 56;
        hesai_ptp_diag_port.peerMeanPathDelay =
          hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++])
                                                    << 48;
        hesai_ptp_diag_port.peerMeanPathDelay =
          hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++])
                                                    << 40;
        hesai_ptp_diag_port.peerMeanPathDelay =
          hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++])
                                                    << 32;
        hesai_ptp_diag_port.peerMeanPathDelay =
          hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++])
                                                    << 24;
        hesai_ptp_diag_port.peerMeanPathDelay =
          hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++])
                                                    << 16;
        hesai_ptp_diag_port.peerMeanPathDelay =
          hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++])
                                                    << 8;
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
      }
    },
    [this]() { CheckUnlock(tm_, "GetPtpDiagPort"); });

  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::GetPtpDiagPort: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetPtpDiagPort" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetPtpDiagPort(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  return GetPtpDiagPort(tcp_driver_local, with_run);
}
Status HesaiHwInterface::GetPtpDiagPort(bool with_run)
{
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetPtpDiagPort(tcp_driver_, with_run);
}

Status HesaiHwInterface::GetPtpDiagTime(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_PTP_DIAGNOSTICS);  // Cmd PTC_COMMAND_PTP_DIAGNOSTICS
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back(PTC_COMMAND_PTP_TIME_STATUS_NP);  // PTP TLV TIME_STATUS_NP
  if (!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetPtpDiagTime")) {
    return GetPtpDiagTime(target_tcp_driver, with_run);
  }
  PrintDebug("GetPtpDiagTime: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetPtpDiagTime getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetPtpDiagTime getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);

      auto response = target_tcp_driver->getPayload();
      HesaiPtpDiagTime hesai_ptp_diag_time;
      if (8 < response.size()) {
        int payload_pos = 8;
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
        hesai_ptp_diag_time.gmPresent = hesai_ptp_diag_time.gmPresent | response[payload_pos++]
                                                                          << 16;
        hesai_ptp_diag_time.gmPresent = hesai_ptp_diag_time.gmPresent | response[payload_pos++]
                                                                          << 8;
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
      }
    },
    [this]() { CheckUnlock(tm_, "GetPtpDiagTime"); });

  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::GetPtpDiagTime: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetPtpDiagTime" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetPtpDiagTime(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  return GetPtpDiagTime(tcp_driver_local, with_run);
}
Status HesaiHwInterface::GetPtpDiagTime(bool with_run)
{
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetPtpDiagTime(tcp_driver_, with_run);
}

Status HesaiHwInterface::GetPtpDiagGrandmaster(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_PTP_DIAGNOSTICS);  // Cmd PTC_COMMAND_PTP_DIAGNOSTICS
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back(PTC_COMMAND_PTP_GRANDMASTER_SETTINGS_NP);  // PTP TLV GRANDMASTER_SETTINGS_NP
  if (!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetPtpDiagGrandmaster")) {
    return GetPtpDiagGrandmaster(target_tcp_driver, with_run);
  }
  PrintDebug("GetPtpDiagGrandmaster: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetPtpDiagGrandmaster getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetPtpDiagGrandmaster getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);

      auto response = target_tcp_driver->getPayload();
      HesaiPtpDiagGrandmaster hesai_ptp_diag_grandmaster;
      if (8 < response.size()) {
        int payload_pos = 8;

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

        std::cout << hesai_ptp_diag_grandmaster << std::endl;
      }
    },
    [this]() { CheckUnlock(tm_, "GetPtpDiagGrandmaster"); });

  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::GetPtpDiagGrandmaster: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetPtpDiagGrandmaster" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetPtpDiagGrandmaster(
  std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  return GetPtpDiagGrandmaster(tcp_driver_local, with_run);
}
Status HesaiHwInterface::GetPtpDiagGrandmaster(bool with_run)
{
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetPtpDiagGrandmaster(tcp_driver_, with_run);
}

Status HesaiHwInterface::GetInventory(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
  std::function<void(HesaiInventory & result)> callback, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_GET_INVENTORY_INFO);  // Cmd PTC_COMMAND_GET_INVENTORY_INFO
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  if (!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetInventory")) {
    return GetInventory(target_tcp_driver, callback, with_run);
  }
  PrintDebug("GetInventory: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver, callback](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetInventory getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetInventory getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);

      auto response = target_tcp_driver->getPayload();
      HesaiInventory hesai_inventory;
      if (8 < response.size()) {
        int payload_pos = 8;
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
        callback(hesai_inventory);
      }
    },
    [this]() { CheckUnlock(tm_, "GetInventory"); });

  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::GetInventory: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetInventory" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetInventory(
  std::shared_ptr<boost::asio::io_context> ctx,
  std::function<void(HesaiInventory & result)> callback, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return GetInventory(tcp_driver_local, callback, with_run);
}
Status HesaiHwInterface::GetInventory(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  return GetInventory(
    ctx, [this](HesaiInventory & result) { std::cout << result << std::endl; }, with_run);
}
Status HesaiHwInterface::GetInventory(
  std::function<void(HesaiInventory & result)> callback, bool with_run)
{
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetInventory(tcp_driver_, callback, with_run);
}
Status HesaiHwInterface::GetInventory(bool with_run)
{
  return GetInventory(
    [this](HesaiInventory & result) { std::cout << result << std::endl; }, with_run);
}

Status HesaiHwInterface::GetConfig(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
  std::function<void(HesaiConfig & result)> callback, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_GET_CONFIG_INFO);  // Cmd PTC_COMMAND_GET_CONFIG_INFO
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);
  if (!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetConfig")) {
    return GetConfig(target_tcp_driver, callback, with_run);
  }
  PrintDebug("GetConfig: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver, callback](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetConfig getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetConfig getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);

      auto response = target_tcp_driver->getPayload();
      HesaiConfig hesai_config;
      if (8 < response.size()) {
        int payload_pos = 8;
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
        hesai_config.dest_LiDAR_udp_port =
          hesai_config.dest_LiDAR_udp_port | response[payload_pos++];
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

        callback(hesai_config);
      }
    },
    [this]() { CheckUnlock(tm_, "GetConfig"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::GetConfig: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetConfig" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetConfig(
  std::shared_ptr<boost::asio::io_context> ctx, std::function<void(HesaiConfig & result)> callback,
  bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return GetConfig(tcp_driver_local, callback, with_run);
}
Status HesaiHwInterface::GetConfig(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  return GetConfig(
    ctx, [this](HesaiConfig & result) { std::cout << result << std::endl; }, with_run);
}
Status HesaiHwInterface::GetConfig(
  std::function<void(HesaiConfig & result)> callback, bool with_run)
{
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetConfig(tcp_driver_, callback, with_run);
}
Status HesaiHwInterface::GetConfig(bool with_run)
{
  return GetConfig([this](HesaiConfig & result) { std::cout << result << std::endl; }, with_run);
}

Status HesaiHwInterface::GetLidarStatus(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
  std::function<void(HesaiLidarStatus & result)> callback, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_GET_LIDAR_STATUS);  // Cmd PTC_COMMAND_GET_LIDAR_STATUS
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  if (!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetLidarStatus")) {
    return GetLidarStatus(target_tcp_driver, callback, with_run);
  }
  PrintDebug("GetLidarStatus: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver, callback](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetLidarStatus getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetLidarStatus getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);

      auto response = target_tcp_driver->getPayload();
      HesaiLidarStatus hesai_status;
      if (8 < response.size()) {
        int payload_pos = 8;
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
        hesai_status.total_operation_time =
          hesai_status.total_operation_time | response[payload_pos++] << 16;
        hesai_status.total_operation_time =
          hesai_status.total_operation_time | response[payload_pos++] << 8;
        hesai_status.total_operation_time =
          hesai_status.total_operation_time | response[payload_pos++];
        hesai_status.ptp_clock_status = static_cast<int>(response[payload_pos++]);
        for (size_t i = 0; i < hesai_status.reserved.size(); i++) {
          hesai_status.reserved[i] = static_cast<unsigned char>(response[payload_pos++]);
        }

        callback(hesai_status);
      }
    },
    [this]() { CheckUnlock(tm_, "GetLidarStatus"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::GetLidarStatus: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetLidarStatus" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetLidarStatus(
  std::shared_ptr<boost::asio::io_context> ctx,
  std::function<void(HesaiLidarStatus & result)> callback, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return GetLidarStatus(tcp_driver_local, callback, with_run);
}
Status HesaiHwInterface::GetLidarStatus(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  return GetLidarStatus(
    ctx, [this](HesaiLidarStatus & result) { std::cout << result << std::endl; }, with_run);
}
Status HesaiHwInterface::GetLidarStatus(
  std::function<void(HesaiLidarStatus & result)> callback, bool with_run)
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "GetLidarStatus tcp_driver_->GetIOContext()->stopped()="
            << tcp_driver_->GetIOContext()->stopped() << std::endl;
#endif
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetLidarStatus(tcp_driver_, callback, with_run);
}
Status HesaiHwInterface::GetLidarStatus(bool with_run)
{
  return GetLidarStatus(
    [this](HesaiLidarStatus & result) { std::cout << result << std::endl; }, with_run);
}

Status HesaiHwInterface::SetSpinRate(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, uint16_t rpm, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 2;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_SPIN_RATE);  // Cmd PTC_COMMAND_SET_SPIN_RATE
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((rpm >> 8) & 0xff);
  buf_vec.emplace_back((rpm >> 0) & 0xff);

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetSpinRate")) {
    return SetSpinRate(target_tcp_driver, rpm, with_run);
  }
  PrintDebug("SetSpinRate: start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetSpinRate"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetSpinRate: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetSpinRate" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetSpinRate(
  std::shared_ptr<boost::asio::io_context> ctx, uint16_t rpm, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetSpinRate(tcp_driver_local, rpm, with_run);
}
Status HesaiHwInterface::SetSpinRate(uint16_t rpm, bool with_run)
{
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  return SetSpinRate(tcp_driver_s_, rpm, with_run);
}

Status HesaiHwInterface::SetSyncAngle(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int sync_angle, int angle,
  bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 3;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_SYNC_ANGLE);  // Cmd PTC_COMMAND_SET_SYNC_ANGLE
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((sync_angle >> 0) & 0xff);
  buf_vec.emplace_back((angle >> 8) & 0xff);
  buf_vec.emplace_back((angle >> 0) & 0xff);

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetSyncAngle")) {
    return SetSyncAngle(target_tcp_driver, sync_angle, angle, with_run);
  }
  PrintDebug("SetSyncAngle: start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetSyncAngle"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetSyncAngle: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetSyncAngle" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetSyncAngle(
  std::shared_ptr<boost::asio::io_context> ctx, int sync_angle, int angle, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetSyncAngle(tcp_driver_local, sync_angle, angle, with_run);
}
Status HesaiHwInterface::SetSyncAngle(int sync_angle, int angle, bool with_run)
{
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  return SetSyncAngle(tcp_driver_s_, sync_angle, angle, with_run);
}

Status HesaiHwInterface::SetTriggerMethod(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int trigger_method,
  bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_TRIGGER_METHOD);  // Cmd PTC_COMMAND_SET_TRIGGER_METHOD
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((trigger_method >> 0) & 0xff);

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetTriggerMethod")) {
    return SetTriggerMethod(target_tcp_driver, trigger_method, with_run);
  }
  PrintDebug("SetTriggerMethod: start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetTriggerMethod"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetTriggerMethod: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetTriggerMethod" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetTriggerMethod(
  std::shared_ptr<boost::asio::io_context> ctx, int trigger_method, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetTriggerMethod(tcp_driver_local, trigger_method, with_run);
}
Status HesaiHwInterface::SetTriggerMethod(int trigger_method, bool with_run)
{
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  return SetTriggerMethod(tcp_driver_s_, trigger_method, with_run);
}

Status HesaiHwInterface::SetStandbyMode(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int standby_mode,
  bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_STANDBY_MODE);  // Cmd PTC_COMMAND_SET_STANDBY_MODE
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((standby_mode >> 0) & 0xff);
  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetStandbyMode")) {
    return SetStandbyMode(target_tcp_driver, standby_mode, with_run);
  }
  std::cout << "start: SetStandbyMode" << std::endl;

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetStandbyMode"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetStandbyMode: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetStandbyMode" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetStandbyMode(
  std::shared_ptr<boost::asio::io_context> ctx, int standby_mode, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetStandbyMode(tcp_driver_local, standby_mode, with_run);
}
Status HesaiHwInterface::SetStandbyMode(int standby_mode, bool with_run)
{
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  return SetStandbyMode(tcp_driver_s_, standby_mode, with_run);
}

Status HesaiHwInterface::SetReturnMode(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int return_mode,
  bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_RETURN_MODE);  // Cmd PTC_COMMAND_SET_RETURN_MODE
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((return_mode >> 0) & 0xff);

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetReturnMode")) {
    return SetReturnMode(target_tcp_driver, return_mode, with_run);
  }
  PrintDebug("SetReturnMode: start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetReturnMode"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetReturnMode: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetReturnMode" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetReturnMode(
  std::shared_ptr<boost::asio::io_context> ctx, int return_mode, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetReturnMode(tcp_driver_local, return_mode, with_run);
}
Status HesaiHwInterface::SetReturnMode(int return_mode, bool with_run)
{
  //*
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  //*/
  return SetReturnMode(tcp_driver_s_, return_mode, with_run);
}

Status HesaiHwInterface::SetDestinationIp(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int dest_ip_1, int dest_ip_2,
  int dest_ip_3, int dest_ip_4, int port, int gps_port, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 8;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_DESTINATION_IP);  // Cmd PTC_COMMAND_SET_DESTINATION_IP
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((dest_ip_1 >> 0) & 0xff);
  buf_vec.emplace_back((dest_ip_2 >> 0) & 0xff);
  buf_vec.emplace_back((dest_ip_3 >> 0) & 0xff);
  buf_vec.emplace_back((dest_ip_4 >> 0) & 0xff);
  buf_vec.emplace_back((port >> 8) & 0xff);
  buf_vec.emplace_back((port >> 0) & 0xff);
  buf_vec.emplace_back((gps_port >> 8) & 0xff);
  buf_vec.emplace_back((gps_port >> 0) & 0xff);

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetDestinationIp")) {
    return SetDestinationIp(
      target_tcp_driver, dest_ip_1, dest_ip_2, dest_ip_3, dest_ip_4, port, gps_port, with_run);
  }
  PrintDebug("SetDestinationIp: start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetDestinationIp"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetDestinationIp: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetDestinationIp" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetDestinationIp(
  std::shared_ptr<boost::asio::io_context> ctx, int dest_ip_1, int dest_ip_2, int dest_ip_3,
  int dest_ip_4, int port, int gps_port, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetDestinationIp(
    tcp_driver_local, dest_ip_1, dest_ip_2, dest_ip_3, dest_ip_4, port, gps_port, with_run);
}
Status HesaiHwInterface::SetDestinationIp(
  int dest_ip_1, int dest_ip_2, int dest_ip_3, int dest_ip_4, int port, int gps_port, bool with_run)
{
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  return SetDestinationIp(
    tcp_driver_s_, dest_ip_1, dest_ip_2, dest_ip_3, dest_ip_4, port, gps_port, with_run);
}

Status HesaiHwInterface::SetControlPort(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int ip_1, int ip_2, int ip_3,
  int ip_4, int mask_1, int mask_2, int mask_3, int mask_4, int gateway_1, int gateway_2,
  int gateway_3, int gateway_4, int vlan_flg, int vlan_id, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 15;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_CONTROL_PORT);  // Cmd PTC_COMMAND_SET_CONTROL_PORT
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((ip_1 >> 0) & 0xff);
  buf_vec.emplace_back((ip_2 >> 0) & 0xff);
  buf_vec.emplace_back((ip_3 >> 0) & 0xff);
  buf_vec.emplace_back((ip_4 >> 0) & 0xff);
  buf_vec.emplace_back((mask_1 >> 0) & 0xff);
  buf_vec.emplace_back((mask_2 >> 0) & 0xff);
  buf_vec.emplace_back((mask_3 >> 0) & 0xff);
  buf_vec.emplace_back((mask_4 >> 0) & 0xff);
  buf_vec.emplace_back((gateway_1 >> 0) & 0xff);
  buf_vec.emplace_back((gateway_2 >> 0) & 0xff);
  buf_vec.emplace_back((gateway_3 >> 0) & 0xff);
  buf_vec.emplace_back((gateway_4 >> 0) & 0xff);
  buf_vec.emplace_back((vlan_flg >> 0) & 0xff);
  buf_vec.emplace_back((vlan_id >> 8) & 0xff);
  buf_vec.emplace_back((vlan_id >> 0) & 0xff);

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetControlPort")) {
    return SetControlPort(
      target_tcp_driver, ip_1, ip_2, ip_3, ip_4, mask_1, mask_2, mask_3, mask_4, gateway_1,
      gateway_2, gateway_3, gateway_4, vlan_flg, vlan_id, with_run);
  }
  PrintDebug("SetControlPort: start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetControlPort"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetControlPort: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetControlPort" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetControlPort(
  std::shared_ptr<boost::asio::io_context> ctx, int ip_1, int ip_2, int ip_3, int ip_4, int mask_1,
  int mask_2, int mask_3, int mask_4, int gateway_1, int gateway_2, int gateway_3, int gateway_4,
  int vlan_flg, int vlan_id, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetControlPort(
    tcp_driver_local, ip_1, ip_2, ip_3, ip_4, mask_1, mask_2, mask_3, mask_4, gateway_1, gateway_2,
    gateway_3, gateway_4, vlan_flg, vlan_id, with_run);
}
Status HesaiHwInterface::SetControlPort(
  int ip_1, int ip_2, int ip_3, int ip_4, int mask_1, int mask_2, int mask_3, int mask_4,
  int gateway_1, int gateway_2, int gateway_3, int gateway_4, int vlan_flg, int vlan_id,
  bool with_run)
{
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  return SetControlPort(
    tcp_driver_s_, ip_1, ip_2, ip_3, ip_4, mask_1, mask_2, mask_3, mask_4, gateway_1, gateway_2,
    gateway_3, gateway_4, vlan_flg, vlan_id, with_run);
}

Status HesaiHwInterface::SetLidarRange(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int method,
  std::vector<unsigned char> data, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1 + data.size();
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_LIDAR_RANGE);  // Cmd PTC_COMMAND_SET_LIDAR_RANGE
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  // 0 - for all channels : 5-1 bytes
  // 1 - for each channel : 323-1 bytes
  // 2 - multi-section FOV : 1347-1 bytes
  buf_vec.emplace_back((method >> 0) & 0xff);
  for (int d : data) {
    buf_vec.emplace_back(d);
  }

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetLidarRange")) {
    return SetLidarRange(target_tcp_driver, method, data, with_run);
  }
  PrintDebug("SetLidarRange: start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetLidarRange"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetLidarRange: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetLidarRange" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetLidarRange(
  std::shared_ptr<boost::asio::io_context> ctx, int method, std::vector<unsigned char> data,
  bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetLidarRange(tcp_driver_local, method, data, with_run);
}
Status HesaiHwInterface::SetLidarRange(int method, std::vector<unsigned char> data, bool with_run)
{
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  return SetLidarRange(tcp_driver_s_, method, data, with_run);
}

Status HesaiHwInterface::SetLidarRange(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int start, int end,
  bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 5;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_LIDAR_RANGE);  // Cmd PTC_COMMAND_SET_LIDAR_RANGE
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  // 0 - for all channels : 5-1 bytes
  // 1 - for each channel : 323-1 bytes
  // 2 - multi-section FOV : 1347-1 bytes
  int method = 0;
  buf_vec.emplace_back((method >> 0) & 0xff);
  buf_vec.emplace_back((start >> 8) & 0xff);
  buf_vec.emplace_back((start >> 0) & 0xff);
  buf_vec.emplace_back((end >> 8) & 0xff);
  buf_vec.emplace_back((end >> 0) & 0xff);

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetLidarRange(All)")) {
    return SetLidarRange(target_tcp_driver, start, end, with_run);
  }
  PrintDebug("SetLidarRange(All): start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetLidarRange(All)"); });
  if (with_run) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "start ctx->run(): SetLidarRange(All)" << std::endl;
#endif
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetLidarRange(All): " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetLidarRange(All)" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetLidarRange(
  std::shared_ptr<boost::asio::io_context> ctx, int start, int end, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetLidarRange(tcp_driver_local, start, end, with_run);
}
Status HesaiHwInterface::SetLidarRange(int start, int end, bool with_run)
{
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  return SetLidarRange(tcp_driver_s_, start, end, with_run);
}

Status HesaiHwInterface::GetLidarRange(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
  std::function<void(HesaiLidarRangeAll & result)> callback, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_GET_LIDAR_RANGE);  // Cmd PTC_COMMAND_GET_LIDAR_RANGE
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);
  if (!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetLidarRange")) {
    return GetLidarRange(target_tcp_driver, callback, with_run);
  }
  PrintDebug("SetLidarRange: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver, callback](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetLidarRange getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetLidarRange getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);

      auto response = target_tcp_driver->getPayload();
      if (8 < response.size()) {
        int payload_pos = 8;
        int method = static_cast<int>(response[payload_pos++]);
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
        std::cout << "GetLidarRange method: " << method << std::endl;
#endif
        if (method == 0)  // for all channels
        {
          HesaiLidarRangeAll hesai_range_all;
          hesai_range_all.method = method;
          hesai_range_all.start = response[payload_pos++] << 8;
          hesai_range_all.start = hesai_range_all.start | response[payload_pos++];
          hesai_range_all.end = response[payload_pos++] << 8;
          hesai_range_all.end = hesai_range_all.end | response[payload_pos++];
          callback(hesai_range_all);
        } else if (method == 1)  // for each channel
        {
          HesaiLidarRangeAll hesai_range_all;
          hesai_range_all.method = method;
          callback(hesai_range_all);
        } else if (method == 2)  // multi-section FOV
        {
          HesaiLidarRangeAll hesai_range_all;
          hesai_range_all.method = method;
          callback(hesai_range_all);
        }
      }
    },
    [this]() { CheckUnlock(tm_, "GetLidarRange"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::GetLidarRange: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetLidarRange" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetLidarRange(
  std::shared_ptr<boost::asio::io_context> ctx,
  std::function<void(HesaiLidarRangeAll & result)> callback, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return GetLidarRange(tcp_driver_local, callback, with_run);
}
Status HesaiHwInterface::GetLidarRange(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  return GetLidarRange(
    ctx, [this](HesaiLidarRangeAll & result) { std::cout << result << std::endl; }, with_run);
}
Status HesaiHwInterface::GetLidarRange(
  std::function<void(HesaiLidarRangeAll & result)> callback, bool with_run)
{
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetLidarRange(tcp_driver_, callback, with_run);
}
Status HesaiHwInterface::GetLidarRange(bool with_run)
{
  return GetLidarRange(
    [this](HesaiLidarRangeAll & result) { std::cout << result << std::endl; }, with_run);
}

Status HesaiHwInterface::SetClockSource(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int clock_source,
  bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_CLOCK_SOURCE);
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((clock_source >> 0) & 0xff);

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetClockSource")) {
    return SetClockSource(target_tcp_driver, clock_source, with_run);
  }
  PrintDebug("SetClockSource: start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetClockSource"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetClockSource: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetReturnMode" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetClockSource(
  std::shared_ptr<boost::asio::io_context> ctx, int clock_source, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetClockSource(tcp_driver_local, clock_source, with_run);
}
Status HesaiHwInterface::SetClockSource(int clock_source, bool with_run)
{
  //*
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  //*/
  return SetClockSource(tcp_driver_s_, clock_source, with_run);
}

Status HesaiHwInterface::SetPtpConfig(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int profile, int domain,
  int network, int logAnnounceInterval = 1, int logSyncInterval = 1, int logMinDelayReqInterval = 0,
  bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 6;
  if (profile == 0) {
  } else if (profile == 1) {
    len = 3;
  } else {
    return Status::ERROR_1;
  }
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_PTP_CONFIG);  // Cmd PTC_COMMAND_SET_PTP_CONFIG
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((profile >> 0) & 0xff);
  buf_vec.emplace_back((domain >> 0) & 0xff);
  buf_vec.emplace_back((network >> 0) & 0xff);
  if (profile == 0) {
    buf_vec.emplace_back((logAnnounceInterval >> 0) & 0xff);
    buf_vec.emplace_back((logSyncInterval >> 0) & 0xff);
    buf_vec.emplace_back((logMinDelayReqInterval >> 0) & 0xff);
  }

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetPtpConfig")) {
    return SetPtpConfig(
      target_tcp_driver, profile, domain, network, logAnnounceInterval, logSyncInterval,
      logMinDelayReqInterval, with_run);
  }
  PrintDebug("SetPtpConfig: start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetPtpConfig"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetPtpConfig: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetPtpConfig" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetPtpConfig(
  std::shared_ptr<boost::asio::io_context> ctx, int profile, int domain, int network,
  int logAnnounceInterval = 1, int logSyncInterval = 1, int logMinDelayReqInterval = 0,
  bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetPtpConfig(
    tcp_driver_local, profile, domain, network, logAnnounceInterval, logSyncInterval,
    logMinDelayReqInterval, with_run);
}
Status HesaiHwInterface::SetPtpConfig(
  int profile, int domain, int network, int logAnnounceInterval, int logSyncInterval,
  int logMinDelayReqInterval, bool with_run)
{
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  return SetPtpConfig(
    tcp_driver_s_, profile, domain, network, logAnnounceInterval, logSyncInterval,
    logMinDelayReqInterval, with_run);
}

Status HesaiHwInterface::GetPtpConfig(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_GET_PTP_CONFIG);  // Cmd PTC_COMMAND_GET_PTP_CONFIG
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  if (!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetPtpConfig")) {
    return GetPtpConfig(target_tcp_driver, with_run);
  }
  PrintDebug("GetPtpConfig: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetPtpConfig getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetPtpConfig getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);

      auto response = target_tcp_driver->getPayload();
      HesaiPtpConfig hesai_ptp_config{};
      if (8 < response.size()) {
        int payload_pos = 8;
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
      }
    },
    [this]() { CheckUnlock(tm_, "GetPtpConfig"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::GetPtpConfig: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetPtpConfig" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetPtpConfig(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return GetPtpConfig(tcp_driver_local, with_run);
}
Status HesaiHwInterface::GetPtpConfig(bool with_run)
{
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  return GetPtpConfig(tcp_driver_s_, with_run);
}

Status HesaiHwInterface::SendReset(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_RESET);  // Cmd PTC_COMMAND_RESET
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SendReset")) {
    return SendReset(target_tcp_driver, with_run);
  }
  PrintDebug("SendReset: start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SendReset"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SendReset: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SendReset" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SendReset(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SendReset(tcp_driver_local, with_run);
}
Status HesaiHwInterface::SendReset(bool with_run)
{
  //*
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  //*/
  return SendReset(tcp_driver_s_, with_run);
}

Status HesaiHwInterface::SetRotDir(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int mode, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_SET_ROTATE_DIRECTION);  // Cmd PTC_COMMAND_SET_ROTATE_DIRECTION
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((mode >> 0) & 0xff);

  if (!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetRotDir")) {
    return SetRotDir(target_tcp_driver, mode, with_run);
  }
  PrintDebug("SetRotDir: start");

  target_tcp_driver->asyncSend(buf_vec, [this]() { CheckUnlock(tms_, "SetRotDir"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::SetRotDir: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetRotDir" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetRotDir(
  std::shared_ptr<boost::asio::io_context> ctx, int mode, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return SetRotDir(tcp_driver_local, mode, with_run);
}
Status HesaiHwInterface::SetRotDir(int mode, bool with_run)
{
  if (with_run) {
    if (tcp_driver_s_->GetIOContext()->stopped()) {
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
  return SetRotDir(tcp_driver_s_, mode, with_run);
}

Status HesaiHwInterface::GetLidarMonitor(
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
  std::function<void(HesaiLidarMonitor & result)> callback, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(PTC_COMMAND_HEADER_HIGH);
  buf_vec.emplace_back(PTC_COMMAND_HEADER_LOW);
  buf_vec.emplace_back(PTC_COMMAND_LIDAR_MONITOR);  // Cmd PTC_COMMAND_LIDAR_MONITOR
  buf_vec.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  if (!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetLidarMonitor")) {
    return GetLidarMonitor(target_tcp_driver, callback, with_run);
  }
  PrintDebug("GetLidarMonitor: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(
    buf_vec,
    [this](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);
    },
    [this, target_tcp_driver, callback](const std::vector<uint8_t> & received_bytes) {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      for (const auto & b : received_bytes) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;

      std::cout << "GetLidarMonitor getHeader: ";
      for (const auto & b : target_tcp_driver->getHeader()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
      std::cout << "GetLidarMonitor getPayload: ";
      for (const auto & b : target_tcp_driver->getPayload()) {
        std::cout << static_cast<int>(b) << ", ";
      }
      std::cout << std::endl;
#endif
      PrintDebug(received_bytes);

      auto response = target_tcp_driver->getPayload();
      HesaiLidarMonitor hesai_lidar_monitor;
      if (8 < response.size()) {
        int payload_pos = 8;
        hesai_lidar_monitor.input_voltage = response[payload_pos++] << 24;
        hesai_lidar_monitor.input_voltage =
          hesai_lidar_monitor.input_voltage | response[payload_pos++] << 16;
        hesai_lidar_monitor.input_voltage =
          hesai_lidar_monitor.input_voltage | response[payload_pos++] << 8;
        hesai_lidar_monitor.input_voltage =
          hesai_lidar_monitor.input_voltage | response[payload_pos++];
        hesai_lidar_monitor.input_current = response[payload_pos++] << 24;
        hesai_lidar_monitor.input_current =
          hesai_lidar_monitor.input_current | response[payload_pos++] << 16;
        hesai_lidar_monitor.input_current =
          hesai_lidar_monitor.input_current | response[payload_pos++] << 8;
        hesai_lidar_monitor.input_current =
          hesai_lidar_monitor.input_current | response[payload_pos++];
        hesai_lidar_monitor.input_power = response[payload_pos++] << 24;
        hesai_lidar_monitor.input_power = hesai_lidar_monitor.input_power | response[payload_pos++]
                                                                              << 16;
        hesai_lidar_monitor.input_power = hesai_lidar_monitor.input_power | response[payload_pos++]
                                                                              << 8;
        hesai_lidar_monitor.input_power = hesai_lidar_monitor.input_power | response[payload_pos++];

        for (size_t i = 0; i < hesai_lidar_monitor.reserved.size(); i++) {
          hesai_lidar_monitor.reserved[i] = static_cast<unsigned char>(response[payload_pos++]);
        }
        callback(hesai_lidar_monitor);
      }
    },
    [this]() { CheckUnlock(tm_, "GetLidarMonitor"); });
  if (with_run) {
    boost::system::error_code ec = target_tcp_driver->run();
    if (ec) {
      PrintError("HesaiHwInterface::GetLidarMonitor: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetLidarMonitor" << std::endl;
#endif
  }
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetLidarMonitor(
  std::shared_ptr<boost::asio::io_context> ctx,
  std::function<void(HesaiLidarMonitor & result)> callback, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(
    sensor_configuration_->sensor_ip, PandarTcpCommandPort, sensor_configuration_->host_ip,
    PandarTcpCommandPort);
  return GetLidarMonitor(tcp_driver_local, callback, with_run);
}
Status HesaiHwInterface::GetLidarMonitor(
  std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  return GetLidarMonitor(
    ctx, [this](HesaiLidarMonitor & result) { std::cout << result << std::endl; }, with_run);
}
Status HesaiHwInterface::GetLidarMonitor(
  std::function<void(HesaiLidarMonitor & result)> callback, bool with_run)
{
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetLidarMonitor(tcp_driver_, callback, with_run);
}
Status HesaiHwInterface::GetLidarMonitor(bool with_run)
{
  return GetLidarMonitor(
    [this](HesaiLidarMonitor & result) { std::cout << result << std::endl; }, with_run);
}

void HesaiHwInterface::IOContextRun() { m_owned_ctx->run(); }

std::shared_ptr<boost::asio::io_context> HesaiHwInterface::GetIOContext() { return m_owned_ctx; }

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

void HesaiHwInterface::str_cb(const std::string & str) { PrintInfo(str); }

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
  std::shared_ptr<boost::asio::io_context> ctx,
  int profile,
  int domain,
  int network,
  int logAnnounceInterval,
  int logSyncInterval,
  int logMinDelayReqInterval)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if (st != Status::OK) {
    return st;
  }

  auto response = hcd->get(
    (boost::format("/pandar.cgi?action=set&object=lidar&key=ptp_configuration&value={" \
              "\"Profile\": %d," \
              "\"Domain\": %d," \
              "\"Network\": %d," \
              "\"LogAnnounceInterval\": %d," \
              "\"LogSyncInterval\": %d," \
              "\"LogMinDelayReqInterval\": %d," \
              "\"tsn_switch\": %d" \
              "}")
              % profile % domain % network % logAnnounceInterval % logSyncInterval % logMinDelayReqInterval % 0
    ).str());
  ctx->run();
  PrintInfo(response);
  return Status::OK;
}

HesaiStatus HesaiHwInterface::SetPtpConfigSyncHttp(int profile,
                                                   int domain,
                                                   int network,
                                                   int logAnnounceInterval,
                                                   int logSyncInterval,
                                                   int logMinDelayReqInterval)
{
  return SetPtpConfigSyncHttp(std::make_shared<boost::asio::io_context>(),
                              profile,
                              domain,
                              network,
                              logAnnounceInterval,
                              logSyncInterval,
                              logMinDelayReqInterval);
}

HesaiStatus HesaiHwInterface::SetSyncAngleSyncHttp(
  std::shared_ptr<boost::asio::io_context> ctx,
  int enable,
  int angle)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if (st != Status::OK) {
    return st;
  }
  auto tmp_str = (boost::format("/pandar.cgi?action=set&object=lidar_sync&key=sync_angle&value={" \
            "\"sync\": %d," \
            "\"syncAngle\": %d" \
            "}") % enable % angle).str();
  PrintInfo(tmp_str);
  auto response = hcd->get(tmp_str);
  ctx->run();
  PrintInfo(response);
  return Status::OK;
}

HesaiStatus HesaiHwInterface::SetSyncAngleSyncHttp(int enable, int angle)
{
  return SetSyncAngleSyncHttp(std::make_shared<boost::asio::io_context>(),
                              enable,
                              angle);
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
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "Start CheckAndSetConfig(HesaiConfig)!!" << std::endl;
#endif
  auto current_return_mode = nebula::drivers::ReturnModeFromIntHesai(
    hesai_config.return_mode, sensor_configuration->sensor_model);
  if (sensor_configuration->return_mode != current_return_mode) {
    std::stringstream ss;
    ss << current_return_mode;
    PrintInfo("current lidar return_mode: " + ss.str());
    std::stringstream ss2;
    ss2 << sensor_configuration->return_mode;
    PrintInfo("current configuration return_mode: " + ss2.str());
    std::thread t([this, sensor_configuration] {
      SetReturnMode(nebula::drivers::IntFromReturnModeHesai(
        sensor_configuration->return_mode, sensor_configuration->sensor_model));
    });
    t.join();
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
      std::thread t(
        [this, sensor_configuration] { SetSpinRate(sensor_configuration->rotation_speed); });
      t.join();
    }
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
  }

  if (sensor_configuration->sensor_model != SensorModel::HESAI_PANDARAT128){
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
      std::thread t([this, sync_flg, scan_phase] {
        SetSyncAngle(sync_flg, scan_phase);
      });
      t.join();
    }

    std::thread t([this] {
      PrintInfo("Trying to set Clock source to PTP");
      SetClockSource(HESAI_LIDAR_PTP_CLOCK_SOURCE);
      PrintInfo("Trying to set PTP Config: IEEE 1588 v2, Domain: 0, Transport: UDP/IP");
      SetPtpConfig(PTP_PROFILE,
                   PTP_DOMAIN_ID,
                   PTP_NETWORK_TRANSPORT,
                   PTP_LOG_ANNOUNCE_INTERVAL,
                   PTP_SYNC_INTERVAL,
                   PTP_LOG_MIN_DELAY_INTERVAL
      );
    });
    t.join();
  }
  else { //AT128 only supports PTP setup via HTTP
    PrintInfo("Trying to set SyncAngle via HTTP");
    SetSyncAngleSyncHttp(1,
                         static_cast<int>(sensor_configuration->scan_phase));
    PrintInfo("Trying to set PTP Config: IEEE 1588 v2, Domain: 0, Transport: UDP/IP via HTTP");
    SetPtpConfigSyncHttp(PTP_PROFILE,
                         PTP_DOMAIN_ID,
                         PTP_NETWORK_TRANSPORT,
                         PTP_LOG_ANNOUNCE_INTERVAL,
                         PTP_SYNC_INTERVAL,
                         PTP_LOG_MIN_DELAY_INTERVAL);

  }

#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "End CheckAndSetConfig(HesaiConfig)!!" << std::endl;
#endif
  return Status::WAITING_FOR_SENSOR_RESPONSE;
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
  if (true) {
    std::thread t([this] {
      GetConfig(  // ctx,
        [this](HesaiConfig & result) {
          std::stringstream ss;
          ss << result;
          PrintInfo(ss.str());
          CheckAndSetConfig(
            std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
        });
    });
    t.join();

    std::thread t2([this] {
      GetLidarRange(  // ctx,
        [this](HesaiLidarRangeAll & result) {
          std::stringstream ss;
          ss << result;
          PrintInfo(ss.str());
          CheckAndSetConfig(
            std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
        });
    });
    t2.join();
  } else if (false) {
    GetConfig([this](HesaiConfig & result) {
      std::cout << result << std::endl;
      CheckAndSetConfig(
        std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
    });
    GetLidarRange([this](HesaiLidarRangeAll & result) {
      std::cout << result << std::endl;
      CheckAndSetConfig(
        std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
    });
    std::cout << "make thread t2" << std::endl;
    std::thread t2([this] {
      tcp_driver_->GetIOContext()->restart();
      tcp_driver_->run();
    });
    std::cout << "made thread t2" << std::endl;
    t2.join();
    std::cout << "joined thread t2" << std::endl;

  } else {
    bool stopped = tcp_driver_->GetIOContext()->stopped();
    std::cout << "stopped: " << stopped << std::endl;
    if (stopped) tcp_driver_->GetIOContext()->restart();
    GetConfig(
      [this](HesaiConfig & result) {
        std::cout << result << std::endl;
        CheckAndSetConfig(
          std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
      },
      false);
    if (stopped) tcp_driver_->run();
    stopped = tcp_driver_->GetIOContext()->stopped();
    std::cout << "stopped2: " << stopped << std::endl;
    if (stopped) tcp_driver_->GetIOContext()->restart();
    GetLidarRange(
      [this](HesaiLidarRangeAll & result) {
        std::cout << result << std::endl;
        CheckAndSetConfig(
          std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
      },
      false);
    if (stopped) tcp_driver_->run();
  }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "End CheckAndSetConfig!!" << std::endl;
#endif
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

void HesaiHwInterface::SetTargetModel(int model) { target_model_no = model; }

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
    case 48:
      return false;
      break;
    default:
      return true;
      break;
  }
}
bool HesaiHwInterface::UseHttpSetSpinRate() { return UseHttpSetSpinRate(target_model_no); }
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
    case 48:
      return false;
      break;
    default:
      return true;
      break;
  }
}
bool HesaiHwInterface::UseHttpGetLidarMonitor() { return UseHttpGetLidarMonitor(target_model_no); }

bool HesaiHwInterface::CheckLock(
  std::timed_mutex & tm, int & fail_cnt, const int & fail_cnt_max, std::string name)
{
  if (wl) {
    if (!tm.try_lock_for(std::chrono::milliseconds(timeout_))) {
      PrintDebug("timeout: " + name);
      fail_cnt++;
      if (fail_cnt_max < fail_cnt) {
        tm.unlock();
        tcp_driver_->close();
        tcp_driver_->open();
        tcp_driver_s_->close();
        tcp_driver_s_->open();
      }
      else {
        return true;
      }
      return false;
    }
    fail_cnt = 0;
  }
  return true;
}

void HesaiHwInterface::CheckUnlock(std::timed_mutex & tm, std::string name)
{
  if (wl) {
    tm.unlock();
    PrintDebug(name + ": finished");
  }
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
