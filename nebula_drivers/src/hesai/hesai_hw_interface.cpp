#include "hesai/hesai_hw_interface.hpp"

#include <pandar_msgs/msg/pandar_jumbo_packet.hpp>
#include <pandar_msgs/msg/pandar_packet.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>

#include <memory>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <strstream>

#define PANDARGENERALSDK_TCP_COMMAND_PORT (9347)

//#define WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE

namespace nebula
{
namespace drivers
{
HesaiHwInterface::HesaiHwInterface()
: cloud_io_context_{new IoContext(1)},
  m_owned_ctx{new boost::asio::io_context(1)},
  m_owned_ctx_s{new boost::asio::io_context(1)},
//  m_owned_ctx{new boost::asio::io_service(3)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  tcp_driver_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx)},
  tcp_driver_s_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx_s)},
//  tcp_driver_s_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx)},
  scan_cloud_ptr_{std::make_unique<pandar_msgs::msg::PandarScan>()}
{
}

Status HesaiHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  HesaiStatus status = Status::OK;
  mtu_size_ = 1500;
  is_solid_state = false;
  try {
    sensor_configuration_ =
      std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration);
    if (
      sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR40P ||
      sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR40P) {
      azimuth_index_ = 2;  // 2 + 124 * [0-9]
      is_valid_packet_ = [](size_t packet_size) {
        return (packet_size == 1262 || packet_size == 1266);
      };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARQT64) {
      azimuth_index_ = 12;  // 12 + 258 * [0-3]
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1072); };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARXT32) {
      azimuth_index_ = 12;  // 12 + 130 * [0-7]
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1080); };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARXT32M) {
      azimuth_index_ = 12;  // 12 + 130 * [0-7]
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == 820); };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARAT128) {
      azimuth_index_ = 12;  // 12 + 4 * 128 * [0-1]
      is_solid_state = true;
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1118); };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR64) {
      azimuth_index_ = 8;  // 8 + 192 * [0-5]
      is_valid_packet_ = [](size_t packet_size) {
        return (packet_size == 1194 || packet_size == 1198);
      };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR128_V14) {
      azimuth_index_ = 12;  // 12 + 386 * [0-1]
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == 893); };  // version 1.4
      mtu_size_ = 1800;
    } else {
      status = Status::INVALID_SENSOR_MODEL;
    }
  } catch (const std::exception & ex) {
    status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << status << std::endl;
    return status;
  }
//  status = CheckAndSetConfig();
//  Status rt = status;
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
  if (is_valid_packet_(buffer.size())) {
    uint32_t buffer_size = buffer.size();
    std::array<uint8_t, 1500> packet_data{};
    std::copy_n(std::make_move_iterator(buffer.begin()), buffer_size, packet_data.begin());
    pandar_msgs::msg::PandarPacket pandar_packet;
    pandar_packet.data = packet_data;
    pandar_packet.size = buffer_size;
    auto now = std::chrono::system_clock::now();
    auto now_secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    auto now_nanosecs = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    pandar_packet.stamp.sec = static_cast<int>(now_secs);
    pandar_packet.stamp.nanosec = static_cast<int>((now_nanosecs/1000000000. - static_cast<double>(now_secs))*1000000000);
    scan_cloud_ptr_->packets.emplace_back(pandar_packet);
  }
  int current_phase = 0;
  bool comp_flg = false;
  {
    const auto & data = scan_cloud_ptr_->packets.back().data;
    /*
    int index = 6+6+1034+6+1+11+2+4+1+1+6;
    std::cout << index << std::endl;
    unsigned int udp_sec = static_cast<unsigned int>((data[index] & 0xff) | (data[index + 1] & 0xff) << 8 |
                  ((data[index + 2] & 0xff) << 16) | ((data[index + 3] & 0xff) << 24));
    std::cout << udp_sec << std::endl;
    */
    current_phase = (data[azimuth_index_] & 0xff) | ((data[azimuth_index_ + 1] & 0xff) << 8);
    if(is_solid_state)// && false)//120
    {
      current_phase = (static_cast<int>(current_phase) + 36000 - 0) % 12000;
//      std::cout << "current_phase=" << current_phase << std::endl;

      if (current_phase >= prev_phase_ || scan_cloud_ptr_->packets.size() < 2) {
        prev_phase_ = current_phase;
      } else {
        comp_flg = true;
      }
    }
    else
    {
      current_phase = (static_cast<int>(current_phase) + 36000 - scan_phase) % 36000;
//      std::cout << "current_phase=" << current_phase << std::endl;

      if (current_phase >= prev_phase_ || scan_cloud_ptr_->packets.size() < 2) {
        prev_phase_ = current_phase;
      } else {
        comp_flg = true;
      }
    }
  }
  if(comp_flg) {  // Scan complete
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


Status HesaiHwInterface::InitializeTcpDriver()
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "HesaiHwInterface::InitializeTcpDriver" << std::endl;
#endif
  tcp_driver_->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  tcp_driver_->open();
  tcp_driver_s_->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  tcp_driver_s_->open();
  return Status::OK;
}

boost::property_tree::ptree HesaiHwInterface::ParseJson(const std::string &str)
{
  boost::property_tree::ptree tree;
  try {
      std::istrstream is(str.c_str());
      boost::property_tree::read_json(is, tree);
  } catch (boost::property_tree::json_parser_error &e) {
      std::cerr << e.what() << std::endl;
  }
  return tree;
}


Status HesaiHwInterface::GetLidarCalib(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x05);//Cmd PTC_COMMAND_GET_LIDAR_CALIBRATION
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  /*
 if(wl){
    std::cout << "try_lock_for: GetLidarCalib" << std::endl;
    if (!tm_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: GetLidarCalib" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetLidarCalib"))
  {
    return GetLidarCalib(target_tcp_driver, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: GetLidarCalib" << std::endl;
  PrintDebug("GetLidarCalib: start");

//  auto tcp_driver_local = new ::drivers::tcp_driver::TcpDriver(ctx);
//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
//  tcp_driver_local->socket()->open();
//  auto sct = tcp_driver_local->socket();

  target_tcp_driver->asyncSendReceiveHeaderPayload(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);
  },
  [this, target_tcp_driver](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;

    std::cout << "GetLidarCalib getHeader: ";
    for(const auto &b :target_tcp_driver->getHeader()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
    std::cout << "GetLidarCalib getPayload: ";
    for(const auto &b :target_tcp_driver->getPayload()){
      std::cout << static_cast<char>(b);
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);

//    sct->close();
//    target_tcp_driver->socket()->close();
  },
  [this]()
  {
    /*
    if(wl){
      tm_.unlock();
      std::cout << "unlocked: GetLidarCalib" << std::endl;
    }
    */
    CheckUnlock(tm_, "GetLidarCalib");
  });
  if(with_run){
//  ctx->run();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::GetLidarCalib: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::GetLidarCalib: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetLidarCalib" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetLidarCalib(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  return GetLidarCalib(tcp_driver_local, with_run);
}
Status HesaiHwInterface::GetLidarCalib(bool with_run)
{
if(with_run){
    if(tcp_driver_->GetIOContext()->stopped()){
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetLidarCalib(tcp_driver_, with_run);
}

Status HesaiHwInterface::GetPtpDiagStatus(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x06);//Cmd PTC_COMMAND_PTP_DIAGNOSTICS
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);
  
  buf_vec.emplace_back(0x01);//PTP STATUS

  /*
 if(wl){
    std::cout << "try_lock_for: GetPtpDiagStatus" << std::endl;
    if (!tm_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: GetPtpDiagStatus" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetPtpDiagStatus"))
  {
    return GetPtpDiagStatus(target_tcp_driver, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: GetPtpDiagStatus" << std::endl;
  PrintDebug("GetPtpDiagStatus: start");

//  auto tcp_driver_local = new ::drivers::tcp_driver::TcpDriver(ctx);
//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
//  tcp_driver_local->socket()->open();
//  auto sct = tcp_driver_local->socket();

  target_tcp_driver->asyncSendReceiveHeaderPayload(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);
  },
  [this, target_tcp_driver](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;

    std::cout << "GetPtpDiagStatus getHeader: ";
    for(const auto &b :target_tcp_driver->getHeader()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
    std::cout << "GetPtpDiagStatus getPayload: ";
    for(const auto &b :target_tcp_driver->getPayload()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);

    auto response = target_tcp_driver->getPayload();
    HesaiPtpDiagStatus hesai_ptp_diag_status;
    if(8 < response.size()){
      int payload_pos = 8;
      /*
      hesai_ptp_diag_status.master_offset = response[payload_pos++] << 56 | 
        response[payload_pos++] << 48 | 
        response[payload_pos++] << 40 | 
        response[payload_pos++] << 32 | 
        response[payload_pos++] << 24 | 
        response[payload_pos++] << 16 | 
        response[payload_pos++] << 8 | 
        response[payload_pos++];
      */
      hesai_ptp_diag_status.master_offset = static_cast<long long>(response[payload_pos++]) << 56;
      hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 48;
      hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 40;
      hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 32;
      hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 24;
      hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 16;
      hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]) << 8;
      hesai_ptp_diag_status.master_offset = hesai_ptp_diag_status.master_offset | static_cast<long long>(response[payload_pos++]);
//      hesai_ptp_diag_status.ptp_state = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_ptp_diag_status.ptp_state = response[payload_pos++] << 24;
      hesai_ptp_diag_status.ptp_state = hesai_ptp_diag_status.ptp_state | response[payload_pos++] << 16;
      hesai_ptp_diag_status.ptp_state = hesai_ptp_diag_status.ptp_state | response[payload_pos++] << 8;
      hesai_ptp_diag_status.ptp_state = hesai_ptp_diag_status.ptp_state | response[payload_pos++];
//      hesai_ptp_diag_status.elapsed_millisec = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_ptp_diag_status.elapsed_millisec = response[payload_pos++] << 24;
      hesai_ptp_diag_status.elapsed_millisec = hesai_ptp_diag_status.elapsed_millisec | response[payload_pos++] << 16;
      hesai_ptp_diag_status.elapsed_millisec = hesai_ptp_diag_status.elapsed_millisec | response[payload_pos++] << 8;
      hesai_ptp_diag_status.elapsed_millisec = hesai_ptp_diag_status.elapsed_millisec | response[payload_pos++];

//      std::cout << hesai_ptp_diag_status << std::endl;
      std::stringstream ss;
      ss << "HesaiHwInterface::GetPtpDiagStatus: " << hesai_ptp_diag_status;
      PrintInfo(ss.str());
    }

//    target_tcp_driver->close();
  },
  [this]()
  {
    /*
    if(wl){
      tm_.unlock();
      std::cout << "unlocked: GetPtpDiagStatus" << std::endl;
    }
    */
    CheckUnlock(tm_, "GetPtpDiagStatus");
  });

  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::GetPtpDiagStatus: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::GetPtpDiagStatus: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetPtpDiagStatus" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetPtpDiagStatus(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  return GetPtpDiagStatus(tcp_driver_local, with_run);
}
Status HesaiHwInterface::GetPtpDiagStatus(bool with_run)
{
if(with_run){
    if(tcp_driver_->GetIOContext()->stopped()){
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetPtpDiagStatus(tcp_driver_, with_run);
}

Status HesaiHwInterface::GetPtpDiagPort(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x06);//Cmd PTC_COMMAND_PTP_DIAGNOSTICS
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);
  
  buf_vec.emplace_back(0x02);//PTP TLV PORT_DATA_SET

  /*
 if(wl){
    std::cout << "try_lock_for: GetPtpDiagPort" << std::endl;
    if (!tm_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: GetPtpDiagPort" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetPtpDiagPort"))
  {
    return GetPtpDiagPort(target_tcp_driver, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: GetPtpDiagPort" << std::endl;
  PrintDebug("GetPtpDiagPort: start");

//  auto tcp_driver_local = new ::drivers::tcp_driver::TcpDriver(ctx);
//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
//  tcp_driver_local->socket()->open();
//  auto sct = tcp_driver_local->socket();

  target_tcp_driver->asyncSendReceiveHeaderPayload(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);
  },
  [this, target_tcp_driver](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;

    std::cout << "GetPtpDiagPort getHeader: ";
    for(const auto &b :target_tcp_driver->getHeader()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
    std::cout << "GetPtpDiagPort getPayload: ";
    for(const auto &b :target_tcp_driver->getPayload()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);

    auto response = target_tcp_driver->getPayload();
    HesaiPtpDiagPort hesai_ptp_diag_port;
    if(8 < response.size()){
      int payload_pos = 8;

      for(size_t i=0;i<hesai_ptp_diag_port.portIdentity.size();i++){
        hesai_ptp_diag_port.portIdentity[i] = response[payload_pos++];
      }
      hesai_ptp_diag_port.portState = static_cast<int>(response[payload_pos++]);
      hesai_ptp_diag_port.logMinDelayReqInterval = static_cast<int>(response[payload_pos++]);
      /*
      hesai_ptp_diag_port.peerMeanPathDelay = response[payload_pos++] << 56 | 
        response[payload_pos++] << 48 | 
        response[payload_pos++] << 40 | 
        response[payload_pos++] << 32 | 
        response[payload_pos++] << 24 | 
        response[payload_pos++] << 16 | 
        response[payload_pos++] << 8 | 
        response[payload_pos++];
      */
      hesai_ptp_diag_port.peerMeanPathDelay = static_cast<long long>(response[payload_pos++]) << 56;
      hesai_ptp_diag_port.peerMeanPathDelay = hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 48;
      hesai_ptp_diag_port.peerMeanPathDelay = hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 40;
      hesai_ptp_diag_port.peerMeanPathDelay = hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 32;
      hesai_ptp_diag_port.peerMeanPathDelay = hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 24;
      hesai_ptp_diag_port.peerMeanPathDelay = hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 16;
      hesai_ptp_diag_port.peerMeanPathDelay = hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]) << 8;
      hesai_ptp_diag_port.peerMeanPathDelay = hesai_ptp_diag_port.peerMeanPathDelay | static_cast<long long>(response[payload_pos++]);
      hesai_ptp_diag_port.logAnnounceInterval = static_cast<int>(response[payload_pos++]);
      hesai_ptp_diag_port.announceReceiptTimeout = static_cast<int>(response[payload_pos++]);
      hesai_ptp_diag_port.logSyncInterval = static_cast<int>(response[payload_pos++]);
      hesai_ptp_diag_port.delayMechanism = static_cast<int>(response[payload_pos++]);
      hesai_ptp_diag_port.logMinPdelayReqInterval = static_cast<int>(response[payload_pos++]);
      hesai_ptp_diag_port.versionNumber = static_cast<int>(response[payload_pos++]);


//      std::cout << hesai_ptp_diag_port << std::endl;
      std::stringstream ss;
      ss << "HesaiHwInterface::GetPtpDiagPort: " << hesai_ptp_diag_port;
      PrintInfo(ss.str());
    }

//    tcp_driver_local->socket()->close();
  },
  [this]()
  {
    /*
    if(wl){
      tm_.unlock();
      std::cout << "unlocked: GetPtpDiagPort" << std::endl;
    }
    */
    CheckUnlock(tm_, "GetPtpDiagPort");
  });

  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::GetPtpDiagPort: " << ec.message() << std::endl;
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
if(with_run){
    if(tcp_driver_->GetIOContext()->stopped()){
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetPtpDiagPort(tcp_driver_, with_run);
}

Status HesaiHwInterface::GetPtpDiagTime(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x06);//Cmd PTC_COMMAND_PTP_DIAGNOSTICS
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);
  
  buf_vec.emplace_back(0x03);//PTP TLV TIME_STATUS_NP

/*
 if(wl){
    std::cout << "try_lock_for: GetPtpDiagTime" << std::endl;
    if (!tm_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: GetPtpDiagTime" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetPtpDiagTime"))
  {
    return GetPtpDiagTime(target_tcp_driver, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: GetPtpDiagTime" << std::endl;
  PrintDebug("GetPtpDiagTime: start");

//  auto tcp_driver_local = new ::drivers::tcp_driver::TcpDriver(ctx);
//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
//  tcp_driver_local->socket()->open();
//  auto sct = tcp_driver_local->socket();

  target_tcp_driver->asyncSendReceiveHeaderPayload(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);
  },
  [this, target_tcp_driver](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;

    std::cout << "GetPtpDiagTime getHeader: ";
    for(const auto &b :target_tcp_driver->getHeader()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
    std::cout << "GetPtpDiagTime getPayload: ";
    for(const auto &b :target_tcp_driver->getPayload()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);

    auto response = target_tcp_driver->getPayload();
    HesaiPtpDiagTime hesai_ptp_diag_time;
    if(8 < response.size()){
      int payload_pos = 8;
      /*
      hesai_ptp_diag_time.master_offset = response[payload_pos++] << 56 | 
        response[payload_pos++] << 48 | 
        response[payload_pos++] << 40 | 
        response[payload_pos++] << 32 | 
        response[payload_pos++] << 24 | 
        response[payload_pos++] << 16 | 
        response[payload_pos++] << 8 | 
        response[payload_pos++];
      */
      hesai_ptp_diag_time.master_offset = static_cast<long long>(response[payload_pos++]) << 56;
      hesai_ptp_diag_time.master_offset = hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 48;
      hesai_ptp_diag_time.master_offset = hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 40;
      hesai_ptp_diag_time.master_offset = hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 32;
      hesai_ptp_diag_time.master_offset = hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 24;
      hesai_ptp_diag_time.master_offset = hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 16;
      hesai_ptp_diag_time.master_offset = hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]) << 8;
      hesai_ptp_diag_time.master_offset = hesai_ptp_diag_time.master_offset | static_cast<long long>(response[payload_pos++]);
      /*
      hesai_ptp_diag_time.ingress_time = response[payload_pos++] << 56 | 
        response[payload_pos++] << 48 | 
        response[payload_pos++] << 40 | 
        response[payload_pos++] << 32 | 
        response[payload_pos++] << 24 | 
        response[payload_pos++] << 16 | 
        response[payload_pos++] << 8 | 
        response[payload_pos++];
      */
      hesai_ptp_diag_time.ingress_time = static_cast<long long>(response[payload_pos++]) << 56;
      hesai_ptp_diag_time.ingress_time = hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 48;
      hesai_ptp_diag_time.ingress_time = hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 40;
      hesai_ptp_diag_time.ingress_time = hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 32;
      hesai_ptp_diag_time.ingress_time = hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 24;
      hesai_ptp_diag_time.ingress_time = hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 16;
      hesai_ptp_diag_time.ingress_time = hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]) << 8;
      hesai_ptp_diag_time.ingress_time = hesai_ptp_diag_time.ingress_time | static_cast<long long>(response[payload_pos++]);
//      hesai_ptp_diag_time.cumulativeScaledRateOffset = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_ptp_diag_time.cumulativeScaledRateOffset = response[payload_pos++] << 24;
      hesai_ptp_diag_time.cumulativeScaledRateOffset = hesai_ptp_diag_time.cumulativeScaledRateOffset | response[payload_pos++] << 16;
      hesai_ptp_diag_time.cumulativeScaledRateOffset = hesai_ptp_diag_time.cumulativeScaledRateOffset | response[payload_pos++] << 8;
      hesai_ptp_diag_time.cumulativeScaledRateOffset = hesai_ptp_diag_time.cumulativeScaledRateOffset | response[payload_pos++];
//      hesai_ptp_diag_time.scaledLastGmPhaseChange = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_ptp_diag_time.scaledLastGmPhaseChange = response[payload_pos++] << 24;
      hesai_ptp_diag_time.scaledLastGmPhaseChange = hesai_ptp_diag_time.scaledLastGmPhaseChange | response[payload_pos++] << 16;
      hesai_ptp_diag_time.scaledLastGmPhaseChange = hesai_ptp_diag_time.scaledLastGmPhaseChange | response[payload_pos++] << 8;
      hesai_ptp_diag_time.scaledLastGmPhaseChange = hesai_ptp_diag_time.scaledLastGmPhaseChange | response[payload_pos++];
//      hesai_ptp_diag_time.gmTimeBaseIndicator = response[payload_pos++] << 8 | response[payload_pos++];
      hesai_ptp_diag_time.gmTimeBaseIndicator = response[payload_pos++] << 8;
      hesai_ptp_diag_time.gmTimeBaseIndicator = hesai_ptp_diag_time.gmTimeBaseIndicator | response[payload_pos++];
      for(size_t i=0;i<hesai_ptp_diag_time.lastGmPhaseChange.size();i++){
        hesai_ptp_diag_time.lastGmPhaseChange[i] = response[payload_pos++];
      }
//      hesai_ptp_diag_time.gmPresent = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_ptp_diag_time.gmPresent = response[payload_pos++] << 24;
      hesai_ptp_diag_time.gmPresent = hesai_ptp_diag_time.gmPresent | response[payload_pos++] << 16;
      hesai_ptp_diag_time.gmPresent = hesai_ptp_diag_time.gmPresent | response[payload_pos++] << 8;
      hesai_ptp_diag_time.gmPresent = hesai_ptp_diag_time.gmPresent | response[payload_pos++];
      /*
      hesai_ptp_diag_time.gmIdentity = response[payload_pos++] << 56 | 
        response[payload_pos++] << 48 | 
        response[payload_pos++] << 40 | 
        response[payload_pos++] << 32 | 
        response[payload_pos++] << 24 | 
        response[payload_pos++] << 16 | 
        response[payload_pos++] << 8 | 
        response[payload_pos++];
      */
      hesai_ptp_diag_time.gmIdentity = static_cast<long long>(response[payload_pos++]) << 56;
      hesai_ptp_diag_time.gmIdentity = hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 48;
      hesai_ptp_diag_time.gmIdentity = hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 40;
      hesai_ptp_diag_time.gmIdentity = hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 32;
      hesai_ptp_diag_time.gmIdentity = hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 24;
      hesai_ptp_diag_time.gmIdentity = hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 16;
      hesai_ptp_diag_time.gmIdentity = hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]) << 8;
      hesai_ptp_diag_time.gmIdentity = hesai_ptp_diag_time.gmIdentity | static_cast<long long>(response[payload_pos++]);

//      std::cout << hesai_ptp_diag_time << std::endl;
      std::stringstream ss;
      ss << "HesaiHwInterface::GetPtpDiagTime: " << hesai_ptp_diag_time;
      PrintInfo(ss.str());
    }

//    target_tcp_driver->socket()->close();
  },
  [this]()
  {
    /*
    if(wl){
      tm_.unlock();
      std::cout << "unlocked: GetPtpDiagTime" << std::endl;
    }
    */
    CheckUnlock(tm_, "GetPtpDiagTime");
  });

  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::GetPtpDiagTime: " << ec.message() << std::endl;
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
if(with_run){
    if(tcp_driver_->GetIOContext()->stopped()){
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetPtpDiagTime(tcp_driver_, with_run);
}

Status HesaiHwInterface::GetPtpDiagGrandmaster(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x06);//Cmd PTC_COMMAND_PTP_DIAGNOSTICS
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);
  
  buf_vec.emplace_back(0x04);//PTP TLV GRANDMASTER_SETTINGS_NP

  /*
 if(wl){
    std::cout << "try_lock_for: GetPtpDiagGrandmaster" << std::endl;
    if (!tm_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: GetPtpDiagGrandmaster" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetPtpDiagGrandmaster"))
  {
    return GetPtpDiagGrandmaster(target_tcp_driver, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: GetPtpDiagGrandmaster" << std::endl;
  PrintDebug("GetPtpDiagGrandmaster: start");

//  auto tcp_driver_local = new ::drivers::tcp_driver::TcpDriver(ctx);
//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
//  tcp_driver_local->socket()->open();
//  auto sct = tcp_driver_local->socket();

  target_tcp_driver->asyncSendReceiveHeaderPayload(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);
  },
  [this, target_tcp_driver](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;

    std::cout << "GetPtpDiagGrandmaster getHeader: ";
    for(const auto &b :target_tcp_driver->getHeader()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
    std::cout << "GetPtpDiagGrandmaster getPayload: ";
    for(const auto &b :target_tcp_driver->getPayload()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);

    auto response = target_tcp_driver->getPayload();
    HesaiPtpDiagGrandmaster hesai_ptp_diag_grandmaster;
    if(8 < response.size()){
      int payload_pos = 8;
      
//      hesai_ptp_diag_grandmaster.clockQuality = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_ptp_diag_grandmaster.clockQuality = response[payload_pos++] << 24;
      hesai_ptp_diag_grandmaster.clockQuality = hesai_ptp_diag_grandmaster.clockQuality | response[payload_pos++] << 16;
      hesai_ptp_diag_grandmaster.clockQuality = hesai_ptp_diag_grandmaster.clockQuality | response[payload_pos++] << 8;
      hesai_ptp_diag_grandmaster.clockQuality = hesai_ptp_diag_grandmaster.clockQuality | response[payload_pos++];
//      hesai_ptp_diag_grandmaster.utc_offset = response[payload_pos++] << 8 | response[payload_pos++];
      hesai_ptp_diag_grandmaster.utc_offset = response[payload_pos++] << 8;
      hesai_ptp_diag_grandmaster.utc_offset = hesai_ptp_diag_grandmaster.utc_offset | response[payload_pos++];
      hesai_ptp_diag_grandmaster.time_flags = static_cast<int>(response[payload_pos++]);
      hesai_ptp_diag_grandmaster.time_source = static_cast<int>(response[payload_pos++]);

      std::cout << hesai_ptp_diag_grandmaster << std::endl;
    }

//    tcp_driver_local->socket()->close();
  },
  [this]()
  {
    /*
    if(wl){
      tm_.unlock();
      std::cout << "unlocked: GetPtpDiagGrandmaster" << std::endl;
    }
    */
    CheckUnlock(tm_, "GetPtpDiagGrandmaster");
  });

  if(with_run){
//    ctx->run();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::GetPtpDiagGrandmaster: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::GetPtpDiagGrandmaster: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetPtpDiagGrandmaster" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetPtpDiagGrandmaster(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  return GetPtpDiagGrandmaster(tcp_driver_local, with_run);
}
Status HesaiHwInterface::GetPtpDiagGrandmaster(bool with_run)
{
if(with_run){
    if(tcp_driver_->GetIOContext()->stopped()){
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetPtpDiagGrandmaster(tcp_driver_, with_run);
}

Status HesaiHwInterface::GetInventory(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, std::function<void(HesaiInventory &result)> callback, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x07);//Cmd PTC_COMMAND_GET_INVENTORY_INFO
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  /*
  std::unique_lock<std::timed_mutex> ul(tm_,
                  std::chrono::milliseconds(timeout_)
                  );
  */
 /*
 if(wl){
    std::cout << "try_lock_for: GetInventory" << std::endl;
    if (!tm_.try_lock_for(std::chrono::milliseconds(timeout_))) {
  //    std::error_code ec(static_cast<int>(std::errc::device_or_resource_busy), std::generic_category());
  //    throw std::system_error(ec);
      std::cout << "timeout: GetInventory" << std::endl;
  //    m_owned_ctx->run();
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetInventory"))
  {
    return GetInventory(target_tcp_driver, callback, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: GetInventory" << std::endl;
  PrintDebug("GetInventory: start");

//  auto sct = tcp_driver_->socket();
//  auto tcp_driver_local = new ::drivers::tcp_driver::TcpDriver(ctx);
//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
//  tcp_driver_local->socket()->open();
//  auto sct = tcp_driver_local->socket();

  target_tcp_driver->asyncSendReceiveHeaderPayload(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);
  },
  [this, target_tcp_driver, callback](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;

    std::cout << "GetInventory getHeader: ";
    for(const auto &b :target_tcp_driver->getHeader()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
    std::cout << "GetInventory getPayload: ";
    for(const auto &b :target_tcp_driver->getPayload()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);

    auto response = target_tcp_driver->getPayload();
    HesaiInventory hesai_inventory;
    if(8 < response.size()){
      int payload_pos = 8;
      for(size_t i=0;i<hesai_inventory.sn.size();i++){
        hesai_inventory.sn[i] = response[payload_pos++];
      }
  //    std::cout << std::string(hesai_inventory.sn.begin(), hesai_inventory.sn.end()) << std::endl;
      for(size_t i=0;i<hesai_inventory.date_of_manufacture.size();i++){
        hesai_inventory.date_of_manufacture[i] = response[payload_pos++];
      }
  //    std::cout << std::string(hesai_inventory.date_of_manufacture.begin(), hesai_inventory.date_of_manufacture.end()) << std::endl;
      for(size_t i=0;i<hesai_inventory.mac.size();i++){
        hesai_inventory.mac[i] = response[payload_pos++];
      }
  //    std::cout << std::string(hesai_inventory.mac.begin(), hesai_inventory.mac.end()) << std::endl;
      for(size_t i=0;i<hesai_inventory.sw_ver.size();i++){
        hesai_inventory.sw_ver[i] = response[payload_pos++];
      }
      for(size_t i=0;i<hesai_inventory.hw_ver.size();i++){
        hesai_inventory.hw_ver[i] = response[payload_pos++];
      }
      for(size_t i=0;i<hesai_inventory.control_fw_ver.size();i++){
        hesai_inventory.control_fw_ver[i] = response[payload_pos++];
      }
      for(size_t i=0;i<hesai_inventory.sensor_fw_ver.size();i++){
        hesai_inventory.sensor_fw_ver[i] = response[payload_pos++];
      }
//      hesai_inventory.angle_offset = response[payload_pos++] << 8 | response[payload_pos++];
      hesai_inventory.angle_offset = response[payload_pos++] << 8;
      hesai_inventory.angle_offset = hesai_inventory.angle_offset | response[payload_pos++];
      hesai_inventory.model = static_cast<int>(response[payload_pos++]);
      hesai_inventory.motor_type = static_cast<int>(response[payload_pos++]);
      hesai_inventory.num_of_lines = static_cast<int>(response[payload_pos++]);
      for(size_t i=0;i<hesai_inventory.reserved.size();i++){
        hesai_inventory.reserved[i] = static_cast<unsigned char>(response[payload_pos++]);
      }


  //    std::cout << hesai_inventory << std::endl;
      callback(hesai_inventory);
    }


//    tcp_driver_local->socket()->close();
  },
  [this]()
  {
    /*
    if(wl){
      tm_.unlock();
      std::cout << "unlocked: GetInventory" << std::endl;
    }
    */
    CheckUnlock(tm_, "GetInventory");
  });
//  m_owned_ctx->run();
//  std::cout << "m_owned_ctx->run(): GetInventory" << std::endl;
//  m_owned_ctx->poll();
//  std::cout << "m_owned_ctx->poll(): GetInventory" << std::endl;
//  boost::system::error_code ec;
//  do m_owned_ctx->run_one(); while (ec == boost::asio::error::would_block);

  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::GetInventory: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::GetInventory: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetInventory" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetInventory(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(HesaiInventory &result)> callback, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return GetInventory(tcp_driver_local, callback, with_run);
}
Status HesaiHwInterface::GetInventory(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  return GetInventory(ctx,
  [this](HesaiInventory &result)
  {
    std::cout << result << std::endl;
  }, with_run);
}
Status HesaiHwInterface::GetInventory(std::function<void(HesaiInventory &result)> callback, bool with_run)
{
if(with_run){
    if(tcp_driver_->GetIOContext()->stopped()){
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetInventory(tcp_driver_, callback, with_run);
}
Status HesaiHwInterface::GetInventory(bool with_run)
{
  return GetInventory(
  [this](HesaiInventory &result)
  {
    std::cout << result << std::endl;
  }, with_run);
}

Status HesaiHwInterface::GetConfig(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, std::function<void(HesaiConfig &result)> callback, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x08);//Cmd PTC_COMMAND_GET_CONFIG_INFO
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  /*
  if(wl){
    std::cout << "try_lock_for: GetConfig" << std::endl;
    if (!tm_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: GetConfig" << std::endl;
  //    m_owned_ctx->run();
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetConfig"))
  {
    return GetConfig(target_tcp_driver, callback, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: GetConfig" << std::endl;
  PrintDebug("GetConfig: start");

//  auto sct = tcp_driver_->socket();
//  auto tcp_driver_local = new ::drivers::tcp_driver::TcpDriver(ctx);
//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
//  tcp_driver_local->socket()->open();
//  auto sct = tcp_driver_local->socket();

  target_tcp_driver->asyncSendReceiveHeaderPayload(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);
  },
  [this, target_tcp_driver, callback](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;

    std::cout << "GetConfig getHeader: ";
    for(const auto &b :target_tcp_driver->getHeader()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
    std::cout << "GetConfig getPayload: ";
    for(const auto &b :target_tcp_driver->getPayload()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);

    auto response = target_tcp_driver->getPayload();
    HesaiConfig hesai_config;
    if(8 < response.size()){
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
//      hesai_config.dest_LiDAR_udp_port = response[payload_pos++] << 8 | response[payload_pos++];
      hesai_config.dest_LiDAR_udp_port = response[payload_pos++] << 8;
      hesai_config.dest_LiDAR_udp_port = hesai_config.dest_LiDAR_udp_port | response[payload_pos++];
//      hesai_config.dest_gps_udp_port = response[payload_pos++] << 8 | response[payload_pos++];
      hesai_config.dest_gps_udp_port = response[payload_pos++] << 8;
      hesai_config.dest_gps_udp_port = hesai_config.dest_gps_udp_port | response[payload_pos++];
//      hesai_config.spin_rate = response[payload_pos++] << 8 | response[payload_pos++];
      hesai_config.spin_rate = response[payload_pos++] << 8;
      hesai_config.spin_rate = hesai_config.spin_rate | response[payload_pos++];
      hesai_config.sync = static_cast<int>(response[payload_pos++]);
//      hesai_config.sync_angle = response[payload_pos++] << 8 | response[payload_pos++];
      hesai_config.sync_angle = response[payload_pos++] << 8;
      hesai_config.sync_angle = hesai_config.sync_angle | response[payload_pos++];
//      hesai_config.start_angle = response[payload_pos++] << 8 | response[payload_pos++];
      hesai_config.start_angle = response[payload_pos++] << 8;
      hesai_config.start_angle = hesai_config.start_angle | response[payload_pos++];
//      hesai_config.stop_angle = response[payload_pos++] << 8 | response[payload_pos++];
      hesai_config.stop_angle = response[payload_pos++] << 8;
      hesai_config.stop_angle = hesai_config.stop_angle | response[payload_pos++];
      hesai_config.clock_source = static_cast<int>(response[payload_pos++]);
      hesai_config.udp_seq = static_cast<int>(response[payload_pos++]);
      hesai_config.trigger_method = static_cast<int>(response[payload_pos++]);
      hesai_config.return_mode = static_cast<int>(response[payload_pos++]);
      hesai_config.standby_mode = static_cast<int>(response[payload_pos++]);
      hesai_config.motor_status = static_cast<int>(response[payload_pos++]);
      hesai_config.vlan_flag = static_cast<int>(response[payload_pos++]);
//      hesai_config.vlan_id = response[payload_pos++] << 8 | response[payload_pos++];
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

  //    std::cout << hesai_config << std::endl;
      callback(hesai_config);
    }

//    tcp_driver_local->socket()->close();
  },
  [this]()
  {
    /*
    if(wl){
      tm_.unlock();
      std::cout << "unlocked: GetConfig" << std::endl;
    }
    */
    CheckUnlock(tm_, "GetConfig");
  });
  if(with_run){
  //  m_owned_ctx->run();
  //  std::cout << "m_owned_ctx->run(): GetConfig" << std::endl;
  //  m_owned_ctx->poll();
  //  std::cout << "m_owned_ctx->poll(): GetConfig" << std::endl;
  //  ctx->run();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::GetConfig: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::GetConfig: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetConfig" << std::endl;
#endif
  }


  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetConfig(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(HesaiConfig &result)> callback, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return GetConfig(tcp_driver_local, callback, with_run);
}
Status HesaiHwInterface::GetConfig(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  return GetConfig(ctx,
  [this](HesaiConfig &result)
  {
    std::cout << result << std::endl;
  }, with_run);
}
Status HesaiHwInterface::GetConfig(std::function<void(HesaiConfig &result)> callback, bool with_run)
{
if(with_run){
    if(tcp_driver_->GetIOContext()->stopped()){
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetConfig(tcp_driver_, callback, with_run);
}
Status HesaiHwInterface::GetConfig(bool with_run)
{
  return GetConfig([this](HesaiConfig &result)
  {
    std::cout << result << std::endl;
  }, with_run);
}

Status HesaiHwInterface::GetLidarStatus(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, std::function<void(HesaiLidarStatus &result)> callback, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x09);//Cmd PTC_COMMAND_GET_LIDAR_STATUS
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  /*
  if(wl){
    std::cout << "try_lock_for: GetLidarStatus" << std::endl;
    if (!tm_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: GetLidarStatus" << std::endl;
  //    m_owned_ctx->run();
      tm_fail_cnt++;
      if(tm_fail_cnt_max < tm_fail_cnt){
        tm_.unlock();
      }
      return Status::ERROR_1;
    }
    tm_fail_cnt = 0;
  }
  */
  if(!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetLidarStatus"))
  {
    return GetLidarStatus(target_tcp_driver, callback, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: GetLidarStatus" << std::endl;
  PrintDebug("GetLidarStatus: start");

//  auto sct = tcp_driver_->socket();
//  auto tcp_driver_local = new ::drivers::tcp_driver::TcpDriver(ctx);
//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_drivers_status.emplace_back(tcp_driver_local);
//  tcp_drivers_status.push_back(tcp_drivers_status);
//  target_tcp_driver->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
//  tcp_driver_local->socket()->open();
//  auto sct = tcp_driver_local->socket();

  target_tcp_driver->asyncSendReceiveHeaderPayload(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);
  },
  [this, target_tcp_driver, callback](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;

    std::cout << "GetLidarStatus getHeader: ";
    for(const auto &b :target_tcp_driver->getHeader()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
    std::cout << "GetLidarStatus getPayload: ";
    for(const auto &b :target_tcp_driver->getPayload()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);

    auto response = target_tcp_driver->getPayload();
    HesaiLidarStatus hesai_status;
    if(8 < response.size()){
      int payload_pos = 8;
//      hesai_status.system_uptime = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_status.system_uptime = response[payload_pos++] << 24;
      hesai_status.system_uptime = hesai_status.system_uptime | response[payload_pos++] << 16;
      hesai_status.system_uptime = hesai_status.system_uptime | response[payload_pos++] << 8;
      hesai_status.system_uptime = hesai_status.system_uptime | response[payload_pos++];
//      hesai_status.motor_speed = response[payload_pos++] << 8 | response[payload_pos++];
      hesai_status.motor_speed = response[payload_pos++] << 8;
      hesai_status.motor_speed = hesai_status.motor_speed | response[payload_pos++];
      for(size_t i=0;i<hesai_status.temperature.size();i++){
//        hesai_status.temperature[i] = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
        hesai_status.temperature[i] = response[payload_pos++] << 24;
        hesai_status.temperature[i] = hesai_status.temperature[i] | response[payload_pos++] << 16;
        hesai_status.temperature[i] = hesai_status.temperature[i] | response[payload_pos++] << 8;
        hesai_status.temperature[i] = hesai_status.temperature[i] | response[payload_pos++];
      }
      hesai_status.gps_pps_lock = static_cast<int>(response[payload_pos++]);
      hesai_status.gps_gprmc_status = static_cast<int>(response[payload_pos++]);
//      hesai_status.startup_times = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_status.startup_times = response[payload_pos++] << 24;
      hesai_status.startup_times = hesai_status.startup_times | response[payload_pos++] << 16;
      hesai_status.startup_times = hesai_status.startup_times | response[payload_pos++] << 8;
      hesai_status.startup_times = hesai_status.startup_times | response[payload_pos++];
//      hesai_status.total_operation_time = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_status.total_operation_time = response[payload_pos++] << 24;
      hesai_status.total_operation_time = hesai_status.total_operation_time | response[payload_pos++] << 16;
      hesai_status.total_operation_time = hesai_status.total_operation_time | response[payload_pos++] << 8;
      hesai_status.total_operation_time = hesai_status.total_operation_time | response[payload_pos++];
      hesai_status.ptp_clock_status = static_cast<int>(response[payload_pos++]);
      for(size_t i=0;i<hesai_status.reserved.size();i++){
        hesai_status.reserved[i] = static_cast<unsigned char>(response[payload_pos++]);
      }

  //    std::cout << hesai_status << std::endl;
      callback(hesai_status);
    }
//    tcp_driver_local->socket()->close();
/*
    tcp_driver_local->close();
    tcp_drivers_status.erase(tcp_drivers_status.begin());
    */
//    tcp_driver_local.reset();
  },
  [this]()
  {
    /*
    if(wl){
      tm_.unlock();
      std::cout << "unlocked: GetLidarStatus" << std::endl;
    }
    */
    CheckUnlock(tm_, "GetLidarStatus");
  });
  if(with_run){
  //  m_owned_ctx->run();
  //  std::cout << "m_owned_ctx->run(): GetLidarStatus" << std::endl;
  //  m_owned_ctx->poll();
  //  std::cout << "m_owned_ctx->poll(): GetLidarStatus" << std::endl;
  //  ctx->run();
    boost::system::error_code ec = target_tcp_driver->run();
  //  ctx->run(ec);
    if(ec){
//      std::cerr << "HesaiHwInterface::GetLidarStatus: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::GetLidarStatus: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetLidarStatus" << std::endl;
#endif
  }


  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetLidarStatus(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(HesaiLidarStatus &result)> callback, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return GetLidarStatus(tcp_driver_local, callback, with_run);
}
Status HesaiHwInterface::GetLidarStatus(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  return GetLidarStatus(ctx,
  [this](HesaiLidarStatus &result)
  {
    std::cout << result << std::endl;
  }, with_run);
}
Status HesaiHwInterface::GetLidarStatus(std::function<void(HesaiLidarStatus &result)> callback, bool with_run)
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "GetLidarStatus tcp_driver_->GetIOContext()->stopped()=" << tcp_driver_->GetIOContext()->stopped() << std::endl;
#endif
if(with_run){
    if(tcp_driver_->GetIOContext()->stopped()){
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetLidarStatus(tcp_driver_, callback, with_run);
  /*
  std::thread t([this, callback]{GetLidarStatus(tcp_driver_, callback);});
  boost::system::error_code ec;
  m_owned_ctx->run(ec);
  std::cout << "m_owned_ctx->run(ec): GetLidarStatus" << std::endl;
  t.join();
  std::cout << "t.join(): GetLidarStatus" << std::endl;
  if(ec){
    std::cerr << "HesaiHwInterface::GetLidarStatus: " << ec.message() << std::endl;
  }
  return Status::WAITING_FOR_SENSOR_RESPONSE;
  */
}
Status HesaiHwInterface::GetLidarStatus(bool with_run)
{
  return GetLidarStatus([this](HesaiLidarStatus &result)
  {
    std::cout << result << std::endl;
  }, with_run);
}

Status HesaiHwInterface::SetSpinRate(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, uint16_t rpm, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 2;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x17);//Cmd PTC_COMMAND_SET_SPIN_RATE
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((rpm >> 8) & 0xff);
  buf_vec.emplace_back((rpm >> 0) & 0xff);

  /*
  if(wl){
    std::cout << "try_lock_for: SetSpinRate" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SetSpinRate" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetSpinRate"))
  {
    return SetSpinRate(target_tcp_driver, rpm, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: SetSpinRate" << std::endl;
  PrintDebug("SetSpinRate: start");

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SetSpinRate" << std::endl;
      }
      */
      CheckUnlock(tms_, "SetSpinRate");
    });
  if(with_run){
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SetSpinRate: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::SetSpinRate: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetSpinRate" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetSpinRate(std::shared_ptr<boost::asio::io_context> ctx, uint16_t rpm, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SetSpinRate(tcp_driver_local, rpm, with_run);
}
Status HesaiHwInterface::SetSpinRate(uint16_t rpm, bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SetSpinRate(tcp_driver_s_, rpm, with_run);
}

Status HesaiHwInterface::SetSyncAngle(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int sync_angle, int angle, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 3;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x18);//Cmd PTC_COMMAND_SET_SYNC_ANGLE
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((sync_angle >> 0) & 0xff);
  buf_vec.emplace_back((angle >> 8) & 0xff);
  buf_vec.emplace_back((angle >> 0) & 0xff);

  /*
  if(wl){
    std::cout << "try_lock_for: SetSyncAngle" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SetSyncAngle" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetSyncAngle"))
  {
    return SetSyncAngle(target_tcp_driver, sync_angle, angle, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: SetSyncAngle" << std::endl;
  PrintDebug("SetSyncAngle: start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SetSyncAngle" << std::endl;
      }
      */
      CheckUnlock(tms_, "SetSyncAngle");
    });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SetSyncAngle: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::SetSyncAngle: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetSyncAngle" << std::endl;
#endif
  }


  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetSyncAngle(std::shared_ptr<boost::asio::io_context> ctx, int sync_angle, int angle, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SetSyncAngle(tcp_driver_local, sync_angle, angle, with_run);
}
Status HesaiHwInterface::SetSyncAngle(int sync_angle, int angle, bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SetSyncAngle(tcp_driver_s_, sync_angle, angle, with_run);
}

Status HesaiHwInterface::SetTriggerMethod(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int trigger_method, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x1b);//Cmd PTC_COMMAND_SET_TRIGGER_METHOD
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((trigger_method >> 0) & 0xff);

  /*
  if(wl){
    std::cout << "try_lock_for: SetTriggerMethod" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SetTriggerMethod" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetTriggerMethod"))
  {
    return SetTriggerMethod(target_tcp_driver, trigger_method, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: SetTriggerMethod" << std::endl;
  PrintDebug("SetTriggerMethod: start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SetTriggerMethod" << std::endl;
      }
      */
      CheckUnlock(tms_, "SetTriggerMethod");
    });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SetTriggerMethod: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::SetTriggerMethod: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetTriggerMethod" << std::endl;
#endif
  }


  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetTriggerMethod(std::shared_ptr<boost::asio::io_context> ctx, int trigger_method, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SetTriggerMethod(tcp_driver_local, trigger_method, with_run);
}
Status HesaiHwInterface::SetTriggerMethod(int trigger_method, bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SetTriggerMethod(tcp_driver_s_, trigger_method, with_run);
}

Status HesaiHwInterface::SetStandbyMode(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int standby_mode, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x1c);//Cmd PTC_COMMAND_SET_STANDBY_MODE
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((standby_mode >> 0) & 0xff);

/*
  if(wl){
    std::cout << "try_lock_for: SetStandbyMode" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SetStandbyMode" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetStandbyMode"))
  {
    return SetStandbyMode(target_tcp_driver, standby_mode, with_run);
//    return Status::ERROR_1;
  }
  std::cout << "start: SetStandbyMode" << std::endl;

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SetStandbyMode" << std::endl;
      }
      */
      CheckUnlock(tms_, "SetStandbyMode");
    });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SetStandbyMode: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::SetStandbyMode: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetStandbyMode" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetStandbyMode(std::shared_ptr<boost::asio::io_context> ctx, int standby_mode, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SetStandbyMode(tcp_driver_local, standby_mode, with_run);
}
Status HesaiHwInterface::SetStandbyMode(int standby_mode, bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SetStandbyMode(tcp_driver_s_, standby_mode, with_run);
}

Status HesaiHwInterface::SetReturnMode(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int return_mode, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x1e);//Cmd PTC_COMMAND_SET_RETURN_MODE
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((return_mode >> 0) & 0xff);

  /*
  if(wl){
    std::cout << "try_lock_for: SetReturnMode" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SetReturnMode" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetReturnMode"))
  {
    return SetReturnMode(target_tcp_driver, return_mode, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: SetReturnMode" << std::endl;
  PrintDebug("SetReturnMode: start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SetReturnMode" << std::endl;
      }
      */
      CheckUnlock(tms_, "SetReturnMode");
    });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SetReturnMode: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::SetReturnMode: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetReturnMode" << std::endl;
#endif
  }


  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetReturnMode(std::shared_ptr<boost::asio::io_context> ctx, int return_mode, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SetReturnMode(tcp_driver_local, return_mode, with_run);
}
Status HesaiHwInterface::SetReturnMode(int return_mode, bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SetReturnMode(tcp_driver_s_, return_mode, with_run);
}

Status HesaiHwInterface::SetDestinationIp(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int dest_ip_1, int dest_ip_2, int dest_ip_3, int dest_ip_4, int port, int gps_port, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 8;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x20);//Cmd PTC_COMMAND_SET_DESTINATION_IP
  buf_vec.emplace_back(0x00);
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

  /*
  if(wl){
    std::cout << "try_lock_for: SetDestinationIp" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SetDestinationIp" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetDestinationIp"))
  {
    return SetDestinationIp(target_tcp_driver, dest_ip_1, dest_ip_2, dest_ip_3, dest_ip_4, port, gps_port, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: SetDestinationIp" << std::endl;
  PrintDebug("SetDestinationIp: start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SetDestinationIp" << std::endl;
      }
      */
      CheckUnlock(tms_, "SetDestinationIp");
    });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SetDestinationIp: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::SetDestinationIp: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetDestinationIp" << std::endl;
#endif
  }


  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetDestinationIp(std::shared_ptr<boost::asio::io_context> ctx, int dest_ip_1, int dest_ip_2, int dest_ip_3, int dest_ip_4, int port, int gps_port, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SetDestinationIp(tcp_driver_local, dest_ip_1, dest_ip_2, dest_ip_3, dest_ip_4, port, gps_port, with_run);
}
Status HesaiHwInterface::SetDestinationIp(int dest_ip_1, int dest_ip_2, int dest_ip_3, int dest_ip_4, int port, int gps_port, bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SetDestinationIp(tcp_driver_s_, dest_ip_1, dest_ip_2, dest_ip_3, dest_ip_4, port, gps_port, with_run);
}

Status HesaiHwInterface::SetControlPort(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
int ip_1, int ip_2, int ip_3, int ip_4,
int mask_1, int mask_2, int mask_3, int mask_4,
int gateway_1, int gateway_2, int gateway_3, int gateway_4,
int vlan_flg, int vlan_id, bool with_run
)
{
  std::vector<unsigned char> buf_vec;
  int len = 15;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x21);//Cmd PTC_COMMAND_SET_CONTROL_PORT
  buf_vec.emplace_back(0x00);
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

  /*
  if(wl){
    std::cout << "try_lock_for: SetControlPort" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SetControlPort" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetControlPort"))
  {
    return SetControlPort(target_tcp_driver,
    ip_1, ip_2, ip_3, ip_4,
    mask_1, mask_2, mask_3, mask_4,
    gateway_1, gateway_2, gateway_3, gateway_4,
    vlan_flg, vlan_id, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: SetControlPort" << std::endl;
  PrintDebug("SetControlPort: start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SetControlPort" << std::endl;
      }
      */
      CheckUnlock(tms_, "SetControlPort");
    });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SetControlPort: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::SetControlPort: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetControlPort" << std::endl;
#endif
  }


  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetControlPort(std::shared_ptr<boost::asio::io_context> ctx,
int ip_1, int ip_2, int ip_3, int ip_4,
int mask_1, int mask_2, int mask_3, int mask_4,
int gateway_1, int gateway_2, int gateway_3, int gateway_4,
int vlan_flg, int vlan_id, bool with_run
){
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SetControlPort(tcp_driver_local, 
  ip_1, ip_2, ip_3, ip_4,
  mask_1, mask_2, mask_3, mask_4,
  gateway_1, gateway_2, gateway_3, gateway_4,
  vlan_flg, vlan_id, with_run);
}
Status HesaiHwInterface::SetControlPort(
  int ip_1, int ip_2, int ip_3, int ip_4,
  int mask_1, int mask_2, int mask_3, int mask_4,
  int gateway_1, int gateway_2, int gateway_3, int gateway_4,
  int vlan_flg, int vlan_id, bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SetControlPort(tcp_driver_s_, 
  ip_1, ip_2, ip_3, ip_4,
  mask_1, mask_2, mask_3, mask_4,
  gateway_1, gateway_2, gateway_3, gateway_4,
  vlan_flg, vlan_id, with_run);
}

Status HesaiHwInterface::SetLidarRange(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int method, std::vector<unsigned char> data, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1 + data.size();
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x22);//Cmd PTC_COMMAND_SET_LIDAR_RANGE
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  // 0 - for all channels : 5-1 bytes
  // 1 - for each channel : 323-1 bytes
  // 2 - multi-section FOV : 1347-1 bytes
  buf_vec.emplace_back((method >> 0) & 0xff);
	for(int d : data){
    buf_vec.emplace_back(d);
	}

  /*
  if(wl){
    std::cout << "try_lock_for: SetLidarRange" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SetLidarRange" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetLidarRange"))
  {
    return SetLidarRange(target_tcp_driver, method, data, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: SetLidarRange" << std::endl;
  PrintDebug("SetLidarRange: start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SetLidarRange" << std::endl;
      }
      */
      CheckUnlock(tms_, "SetLidarRange");
    });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SetLidarRange: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::SetLidarRange: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetLidarRange" << std::endl;
#endif
  }


  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetLidarRange(std::shared_ptr<boost::asio::io_context> ctx, int method, std::vector<unsigned char> data, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SetLidarRange(tcp_driver_local, method, data, with_run);
}
Status HesaiHwInterface::SetLidarRange(int method, std::vector<unsigned char> data, bool with_run)
{
  //*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SetLidarRange(tcp_driver_s_, method, data, with_run);
}

Status HesaiHwInterface::SetLidarRange(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int start, int end, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 5;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x22);//Cmd PTC_COMMAND_SET_LIDAR_RANGE
  buf_vec.emplace_back(0x00);
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

  /*
  if(wl){
    std::cout << "try_lock_for: SetLidarRange(All)" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SetLidarRange(All)" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetLidarRange(All)"))
  {
    return SetLidarRange(target_tcp_driver, start, end, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: SetLidarRange(All)" << std::endl;
  PrintDebug("SetLidarRange(All): start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SetLidarRange(All)" << std::endl;
      }
      */
      CheckUnlock(tms_, "SetLidarRange(All)");
    });
  if(with_run){
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "start ctx->run(): SetLidarRange(All)" << std::endl;
#endif
  //  ctx->run();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SetLidarRange(All): " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::SetLidarRange(All): " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetLidarRange(All)" << std::endl;
#endif
  }


  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetLidarRange(std::shared_ptr<boost::asio::io_context> ctx, int start, int end, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SetLidarRange(tcp_driver_local, start, end, with_run);
}
Status HesaiHwInterface::SetLidarRange(int start, int end, bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SetLidarRange(tcp_driver_s_, start, end, with_run);
}

Status HesaiHwInterface::GetLidarRange(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, std::function<void(HesaiLidarRangeAll &result)> callback, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x23);//Cmd PTC_COMMAND_GET_LIDAR_RANGE
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  /*
  if(wl){
    std::cout << "try_lock_for: GetLidarRange" << std::endl;
    if (!tm_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: GetLidarRange" << std::endl;
  //    m_owned_ctx->run();
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetLidarRange"))
  {
    return GetLidarRange(target_tcp_driver, callback, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: GetLidarRange" << std::endl;
  PrintDebug("SetLidarRange: start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSendReceiveHeaderPayload(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);
  },
  [this, target_tcp_driver, callback](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;

    std::cout << "GetLidarRange getHeader: ";
    for(const auto &b :target_tcp_driver->getHeader()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
    std::cout << "GetLidarRange getPayload: ";
    for(const auto &b :target_tcp_driver->getPayload()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);

    auto response = target_tcp_driver->getPayload();
    if(8 < response.size()){
      int payload_pos = 8;
      int method = static_cast<int>(response[payload_pos++]);
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
      std::cout << "GetLidarRange method: " << method << std::endl;
#endif
      if(method == 0)//for all channels
      {
        HesaiLidarRangeAll hesai_range_all;
        hesai_range_all.method = method;
//        hesai_range_all.start = response[payload_pos++] << 8 | response[payload_pos++];
        hesai_range_all.start = response[payload_pos++] << 8;
        hesai_range_all.start = hesai_range_all.start | response[payload_pos++];
//        hesai_range_all.end = response[payload_pos++] << 8 | response[payload_pos++];
        hesai_range_all.end = response[payload_pos++] << 8;
        hesai_range_all.end = hesai_range_all.end | response[payload_pos++];
  //      std::cout << hesai_range_all << std::endl;
        callback(hesai_range_all);
      }else if(method == 1)//for each channel
      {
        HesaiLidarRangeAll hesai_range_all;
        hesai_range_all.method = method;
        callback(hesai_range_all);
      }else if(method == 2)//multi-section FOV
      {
        HesaiLidarRangeAll hesai_range_all;
        hesai_range_all.method = method;
        callback(hesai_range_all);
      }
    }

//    tcp_driver_local->socket()->close();
  },
  [this]()
  {
    /*
    if(wl){
      tm_.unlock();
      std::cout << "unlocked: GetLidarRange" << std::endl;
    }
    */
    CheckUnlock(tm_, "GetLidarRange");
  });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::GetLidarRange: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::GetLidarRange: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetLidarRange" << std::endl;
#endif
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetLidarRange(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(HesaiLidarRangeAll &result)> callback, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return GetLidarRange(tcp_driver_local, callback, with_run);
}
Status HesaiHwInterface::GetLidarRange(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  return GetLidarRange(ctx,
  [this](HesaiLidarRangeAll &result)
  {
    std::cout << result << std::endl;
  }, with_run);
}
Status HesaiHwInterface::GetLidarRange(std::function<void(HesaiLidarRangeAll &result)> callback, bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_->GetIOContext()->stopped()){
      tcp_driver_->GetIOContext()->restart();
    }
  }
//*/
  return GetLidarRange(tcp_driver_, callback, with_run);
}
Status HesaiHwInterface::GetLidarRange(bool with_run)
{
  return GetLidarRange(
  [this](HesaiLidarRangeAll &result)
  {
    std::cout << result << std::endl;
  }, with_run);
}

Status HesaiHwInterface::SetPtpConfig(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
int profile,
int domain,
int network,
int logAnnounceInterval=1,
int logSyncInterval=1,
int logMinDelayReqInterval=0,
bool with_run
)
{
  std::vector<unsigned char> buf_vec;
  int len = 6;
  if(profile==0){

  }else if(profile == 1){
    len = 3;
  }else{
    return Status::ERROR_1;
  }
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x24);//Cmd PTC_COMMAND_SET_PTP_CONFIG
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((profile >> 0) & 0xff);
  buf_vec.emplace_back((domain >> 0) & 0xff);
  buf_vec.emplace_back((network >> 0) & 0xff);
  if(profile==0){
    buf_vec.emplace_back((logAnnounceInterval >> 0) & 0xff);
    buf_vec.emplace_back((logSyncInterval >> 0) & 0xff);
    buf_vec.emplace_back((logMinDelayReqInterval >> 0) & 0xff);
  }

  /*
  if(wl){
    std::cout << "try_lock_for: SetPtpConfig" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SetPtpConfig" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetPtpConfig"))
  {
    return SetPtpConfig(target_tcp_driver, 
    profile,
    domain,
    network,
    logAnnounceInterval,
    logSyncInterval,
    logMinDelayReqInterval,
    with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: SetPtpConfig" << std::endl;
  PrintDebug("SetPtpConfig: start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SetPtpConfig" << std::endl;
      }
      */
      CheckUnlock(tms_, "SetPtpConfig");
    });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SetPtpConfig: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::SetPtpConfig: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetPtpConfig" << std::endl;
#endif
  }


  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetPtpConfig(std::shared_ptr<boost::asio::io_context> ctx,
int profile,
int domain,
int network,
int logAnnounceInterval=1,
int logSyncInterval=1,
int logMinDelayReqInterval=0,
bool with_run
)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SetPtpConfig(tcp_driver_local, 
  profile,
  domain,
  network,
  logAnnounceInterval,
  logSyncInterval,
  logMinDelayReqInterval,
  with_run);
}
Status HesaiHwInterface::SetPtpConfig(
  int profile,
  int domain,
  int network,
  int logAnnounceInterval,
  int logSyncInterval,
  int logMinDelayReqInterval,
  bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SetPtpConfig(tcp_driver_s_, 
  profile,
  domain,
  network,
  logAnnounceInterval,
  logSyncInterval,
  logMinDelayReqInterval,
  with_run);
}

Status HesaiHwInterface::GetPtpConfig(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x26);//Cmd PTC_COMMAND_GET_PTP_CONFIG
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  /*
  if(wl){
    std::cout << "try_lock_for: GetPtpConfig" << std::endl;
    if (!tm_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: GetPtpConfig" << std::endl;
  //    m_owned_ctx->run();
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetPtpConfig"))
  {
    return GetPtpConfig(target_tcp_driver, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: GetPtpConfig" << std::endl;
  PrintDebug("GetPtpConfig: start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSendReceiveHeaderPayload(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);
  },
  [this, target_tcp_driver](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;

    std::cout << "GetPtpConfig getHeader: ";
    for(const auto &b :target_tcp_driver->getHeader()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
    std::cout << "GetPtpConfig getPayload: ";
    for(const auto &b :target_tcp_driver->getPayload()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);

    auto response = target_tcp_driver->getPayload();
    HesaiPtpConfig hesai_ptp_config;
    if(8 < response.size()){
      int payload_pos = 8;
      hesai_ptp_config.status = static_cast<int>(response[payload_pos++]);
      hesai_ptp_config.profile = static_cast<int>(response[payload_pos++]);
      hesai_ptp_config.domain = static_cast<int>(response[payload_pos++]);
      hesai_ptp_config.network = static_cast<int>(response[payload_pos++]);
      if(hesai_ptp_config.status == 0){
        hesai_ptp_config.logAnnounceInterval = static_cast<int>(response[payload_pos++]);
        hesai_ptp_config.logSyncInterval = static_cast<int>(response[payload_pos++]);
        hesai_ptp_config.logMinDelayReqInterval = static_cast<int>(response[payload_pos++]);
      }

//      std::cout << hesai_ptp_config << std::endl;
      std::stringstream ss;
      ss << "HesaiHwInterface::GetPtpConfig: " << hesai_ptp_config;
      PrintInfo(ss.str());
    }
//    tcp_driver_local->socket()->close();
  },
  [this]()
  {
    /*
    if(wl){
      tm_.unlock();
      std::cout << "unlocked: GetPtpConfig" << std::endl;
    }
    */
    CheckUnlock(tm_, "GetPtpConfig");
  });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::GetPtpConfig: " << ec.message() << std::endl;
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
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return GetPtpConfig(tcp_driver_local, with_run);
}
Status HesaiHwInterface::GetPtpConfig(bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return GetPtpConfig(tcp_driver_s_, with_run);
}

Status HesaiHwInterface::SendReset(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x25);//Cmd PTC_COMMAND_RESET
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  /*
  if(wl){
    std::cout << "try_lock_for: SendReset" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SendReset" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SendReset"))
  {
    return SendReset(target_tcp_driver, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: SendReset" << std::endl;
  PrintDebug("SendReset: start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SendReset" << std::endl;
      }
      */
      CheckUnlock(tms_, "SendReset");
    });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SendReset: " << ec.message() << std::endl;
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
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SendReset(tcp_driver_local, with_run);
}
Status HesaiHwInterface::SendReset(bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SendReset(tcp_driver_s_, with_run);
}


Status HesaiHwInterface::SetRotDir(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int mode, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 1;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x2a);//Cmd PTC_COMMAND_SET_ROTATE_DIRECTION
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  buf_vec.emplace_back((mode >> 0) & 0xff);

  /*
  if(wl){
    std::cout << "try_lock_for: SetRotDir" << std::endl;
    if (!tms_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: SetRotDir" << std::endl;
      return Status::ERROR_1;
    }
  }
  */
  if(!CheckLock(tms_, tms_fail_cnt, tms_fail_cnt_max, "SetRotDir"))
  {
    return SetRotDir(target_tcp_driver, mode, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: SetRotDir" << std::endl;
  PrintDebug("SetRotDir: start");

//  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
//  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);

  target_tcp_driver->asyncSend(buf_vec,
    [this]()
    {
      /*
      if(wl){
        tms_.unlock();
        std::cout << "unlocked: SetRotDir" << std::endl;
      }
      */
      CheckUnlock(tms_, "SetRotDir");
    });
  if(with_run){// && target_tcp_driver->GetIOContext()->stopped()){
  //  ctx->run();
  //  target_tcp_driver->GetIOContext()->restart();
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::SetRotDir: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::SetRotDir: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): SetRotDir" << std::endl;
#endif
  }


  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::SetRotDir(std::shared_ptr<boost::asio::io_context> ctx, int mode, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return SetRotDir(tcp_driver_local, mode, with_run);
}
Status HesaiHwInterface::SetRotDir(int mode, bool with_run)
{
//*
  if(with_run){
    if(tcp_driver_s_->GetIOContext()->stopped()){
      tcp_driver_s_->GetIOContext()->restart();
    }
  }
//*/
  return SetRotDir(tcp_driver_s_, mode, with_run);
}




Status HesaiHwInterface::GetLidarMonitor(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, std::function<void(HesaiLidarMonitor &result)> callback, bool with_run)
{
  std::vector<unsigned char> buf_vec;
  int len = 0;
  buf_vec.emplace_back(0x47);
  buf_vec.emplace_back(0x74);
  buf_vec.emplace_back(0x27);//Cmd PTC_COMMAND_LIDAR_MONITOR
  buf_vec.emplace_back(0x00);
  buf_vec.emplace_back((len >> 24) & 0xff);
  buf_vec.emplace_back((len >> 16) & 0xff);
  buf_vec.emplace_back((len >> 8) & 0xff);
  buf_vec.emplace_back((len >> 0) & 0xff);

  /*
  if(wl){
    std::cout << "try_lock_for: GetLidarMonitor" << std::endl;
    if (!tm_.try_lock_for(std::chrono::milliseconds(timeout_))) {
      std::cout << "timeout: GetLidarMonitor" << std::endl;
      return Status::ERROR_1;
    }
    tm_fail_cnt = 0;
  }
  */
  if(!CheckLock(tm_, tm_fail_cnt, tm_fail_cnt_max, "GetLidarMonitor"))
  {
    return GetLidarMonitor(target_tcp_driver, callback, with_run);
//    return Status::ERROR_1;
  }
//  std::cout << "start: GetLidarMonitor" << std::endl;
  PrintDebug("GetLidarMonitor: start");

  target_tcp_driver->asyncSendReceiveHeaderPayload(buf_vec,
  [this](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);
  },
  [this, target_tcp_driver, callback](const std::vector<uint8_t> & received_bytes)
  {
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    for(const auto &b :received_bytes){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;

    std::cout << "GetLidarMonitor getHeader: ";
    for(const auto &b :target_tcp_driver->getHeader()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
    std::cout << "GetLidarMonitor getPayload: ";
    for(const auto &b :target_tcp_driver->getPayload()){
      std::cout << static_cast<int>(b) << ", ";
    }
    std::cout << std::endl;
#endif
    PrintDebug(received_bytes);

    auto response = target_tcp_driver->getPayload();
    HesaiLidarMonitor hesai_lidar_monitor;
    if(8 < response.size()){
      int payload_pos = 8;
//      hesai_lidar_monitor.input_voltage = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_lidar_monitor.input_voltage = response[payload_pos++] << 24;
      hesai_lidar_monitor.input_voltage = hesai_lidar_monitor.input_voltage | response[payload_pos++] << 16;
      hesai_lidar_monitor.input_voltage = hesai_lidar_monitor.input_voltage | response[payload_pos++] << 8;
      hesai_lidar_monitor.input_voltage = hesai_lidar_monitor.input_voltage | response[payload_pos++];
//      hesai_lidar_monitor.input_current = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_lidar_monitor.input_current = response[payload_pos++] << 24;
      hesai_lidar_monitor.input_current = hesai_lidar_monitor.input_current | response[payload_pos++] << 16;
      hesai_lidar_monitor.input_current = hesai_lidar_monitor.input_current | response[payload_pos++] << 8;
      hesai_lidar_monitor.input_current = hesai_lidar_monitor.input_current | response[payload_pos++];
//      hesai_lidar_monitor.input_power = response[payload_pos++] << 24 | response[payload_pos++] << 16 | response[payload_pos++] << 8 | response[payload_pos++];
      hesai_lidar_monitor.input_power = response[payload_pos++] << 24;
      hesai_lidar_monitor.input_power = hesai_lidar_monitor.input_power | response[payload_pos++] << 16;
      hesai_lidar_monitor.input_power = hesai_lidar_monitor.input_power | response[payload_pos++] << 8;
      hesai_lidar_monitor.input_power = hesai_lidar_monitor.input_power | response[payload_pos++];

      for(size_t i=0;i<hesai_lidar_monitor.reserved.size();i++){
        hesai_lidar_monitor.reserved[i] = static_cast<unsigned char>(response[payload_pos++]);
      }

      callback(hesai_lidar_monitor);
    }
  },
  [this]()
  {
    /*
    if(wl){
      tm_.unlock();
        std::cout << "unlocked: GetLidarMonitor" << std::endl;
    }
    */
    CheckUnlock(tm_, "GetLidarMonitor");
  });
  if(with_run){
    boost::system::error_code ec = target_tcp_driver->run();
    if(ec){
//      std::cerr << "HesaiHwInterface::GetLidarMonitor: " << ec.message() << std::endl;
      PrintError("HesaiHwInterface::GetLidarMonitor: " + ec.message());
    }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "ctx->run(): GetLidarMonitor" << std::endl;
#endif
  }
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}
Status HesaiHwInterface::GetLidarMonitor(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(HesaiLidarMonitor &result)> callback, bool with_run)
{
  auto tcp_driver_local = std::make_shared<::drivers::tcp_driver::TcpDriver>(ctx);
  tcp_driver_local->init_socket(sensor_configuration_->sensor_ip, PANDARGENERALSDK_TCP_COMMAND_PORT, sensor_configuration_->host_ip, PANDARGENERALSDK_TCP_COMMAND_PORT);
  return GetLidarMonitor(tcp_driver_local, callback, with_run);
}
Status HesaiHwInterface::GetLidarMonitor(std::shared_ptr<boost::asio::io_context> ctx, bool with_run)
{
  return GetLidarMonitor(ctx,
  [this](HesaiLidarMonitor &result)
  {
    std::cout << result << std::endl;
  }, with_run);
}
Status HesaiHwInterface::GetLidarMonitor(std::function<void(HesaiLidarMonitor &result)> callback, bool with_run)
{
if(with_run){
    if(tcp_driver_->GetIOContext()->stopped()){
      tcp_driver_->GetIOContext()->restart();
    }
  }
  return GetLidarMonitor(tcp_driver_, callback, with_run);
}
Status HesaiHwInterface::GetLidarMonitor(bool with_run)
{
  return GetLidarMonitor([this](HesaiLidarMonitor &result)
  {
    std::cout << result << std::endl;
  }, with_run);
}



void HesaiHwInterface::IOContextRun()
{
  m_owned_ctx->run();
}

std::shared_ptr<boost::asio::io_context> HesaiHwInterface::GetIOContext()
{
  return m_owned_ctx;
}

HesaiStatus HesaiHwInterface::GetHttpClientDriverOnce(std::shared_ptr<boost::asio::io_context> ctx, std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>& hcd){
  hcd = std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>(new ::drivers::tcp_driver::HttpClientDriver(ctx));
  try {
    hcd->init_client(sensor_configuration_->sensor_ip, 80);
  } catch (const std::exception & ex) {
    Status status = Status::HTTP_CONNECTION_ERROR;
//    std::cerr << status << sensor_configuration_->sensor_ip << ","
//              << 80 << std::endl;
    std::stringstream ss;
    ss << "HesaiHwInterface::GetHttpClientDriverOnce: " << status << sensor_configuration_->sensor_ip << "," << 80 << std::endl;
    PrintError(ss.str());
    return Status::HTTP_CONNECTION_ERROR;
  }
  return Status::OK;
}

HesaiStatus HesaiHwInterface::GetHttpClientDriverOnce(std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>& hcd){
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd_tmp;
  auto st = GetHttpClientDriverOnce(std::make_shared<boost::asio::io_context>(), hcd_tmp);
  hcd = std::move(hcd_tmp);
  return st;
}

void HesaiHwInterface::str_cb(const std::string &str)
{
//  std::cout << "HesaiHwInterface::str_cb: " << str <<std::endl;
  PrintInfo(str);
}

HesaiStatus HesaiHwInterface::SetSpinSpeedAsyncHttp(std::shared_ptr<boost::asio::io_context> ctx, uint16_t rpm)
{
//  auto ctx = std::make_shared<boost::asio::io_context>();
//  auto hcd = GetHttpClientDriverOnce(ctx);
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
//    HesaiStatus rt = st;
//    return rt;
    return st;
  }

  int rpm_key = 2;
  switch (rpm)
  {
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
//  http_client_driver_->asyncPost(
  hcd->asyncGet(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    (boost::format("/pandar.cgi?action=set&object=lidar&key=spin_speed&value=%d") % rpm_key).str()
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

HesaiStatus HesaiHwInterface::SetSpinSpeedAsyncHttp(uint16_t rpm)
{
  return SetSpinSpeedAsyncHttp(std::make_shared<boost::asio::io_context>(), rpm);
}

HesaiStatus HesaiHwInterface::GetLidarMonitorAsyncHttp(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(const std::string &str)> str_callback)
{
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
//    std::cerr << "HesaiHwInterface::GetLidarMonitorAsyncHttp: " << "cannnot GetHttpClientDriverOnce" << std::endl;
    PrintError("HesaiHwInterface::GetLidarMonitorAsyncHttp: cannnot GetHttpClientDriverOnce");
    return st;
  }

  hcd->asyncGet(
    [this, str_callback](const std::string &str)
    {
      str_callback(str);
    },
    "/pandar.cgi?action=get&object=lidar_monitor"
    );
  boost::system::error_code ec;
  ctx->run(ec);
  if(ec){
//    std::cerr << "HesaiHwInterface::GetLidarMonitorAsyncHttp: " << ec.message() << std::endl;
    PrintError("HesaiHwInterface::GetLidarMonitorAsyncHttp: " + ec.message());
  }
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

HesaiStatus HesaiHwInterface::GetLidarMonitorAsyncHttp(std::function<void(const std::string &str)> str_callback)
{
  return GetLidarMonitorAsyncHttp(std::make_shared<boost::asio::io_context>(), str_callback);
}

HesaiStatus HesaiHwInterface::CheckAndSetConfig(std::shared_ptr<HesaiSensorConfiguration> sensor_configuration, HesaiConfig hesai_config)
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "Start CheckAndSetConfig(HesaiConfig)!!" << std::endl;
#endif
  auto current_return_mode = nebula::drivers::ReturnModeFromIntHesai(hesai_config.return_mode);
  if(sensor_configuration->return_mode != current_return_mode)
  {
//    std::cout << "current_return_mode: " << current_return_mode << std::endl;
//    std::cout << "sensor_configuration->return_mode: " << sensor_configuration->return_mode << std::endl;
    std::stringstream ss;
    ss << current_return_mode;
    PrintInfo("current lidar return_mode: " + ss.str());
    std::stringstream ss2;
    ss2 << sensor_configuration->return_mode;
    PrintInfo("current configuration return_mode: " + ss2.str());
//    if(wl){
//      tm_.unlock();
//    }
    std::thread t([this, sensor_configuration]{
    SetReturnMode(nebula::drivers::IntFromReturnModeHesai(sensor_configuration->return_mode));//, false);
//    SetReturnMode(nebula::drivers::IntFromReturnModeHesai(sensor_configuration->return_mode), false);
    });
    t.join();
    /*
    std::thread t([this, sensor_configuration]{SetReturnMode(nebula::drivers::IntFromReturnModeHesai(sensor_configuration->return_mode));});
    boost::system::error_code ec;
    m_owned_ctx->run(ec);
    if(ec){
      std::cerr << "HesaiHwInterface::CheckAndSetConfig(ReturnMode): " << ec.message() << std::endl;
    }
    t.join();
    */
//    if(wl){
//      tm_.lock();
//    }
  }
  /*
  auto current_rotation_speed_key = hesai_config.spin_rate;
  auto current_rotation_speed = 300;
  switch (current_rotation_speed_key)
  {
  case 1:
    current_rotation_speed = 300;
    break;
  case 2:
    current_rotation_speed = 600;
    break;
  case 3:
    current_rotation_speed = 1200;
    break;
  default:
  std::cout << "hesai_config.spin_rate=" << hesai_config.spin_rate << std::endl;
    break;
  }
  */
  auto current_rotation_speed = hesai_config.spin_rate;
  if(sensor_configuration->rotation_speed != current_rotation_speed)
  {
//    SetRpmAsync(sensor_configuration->rotation_speed);
    // needs http client...?
//    std::cout << "current_rotation_speed: " << current_rotation_speed << std::endl;
//    std::cout << "sensor_configuration->rotation_speed: " << sensor_configuration->rotation_speed << std::endl;
    PrintInfo("current lidar rotation_speed: " + std::to_string(current_rotation_speed));
    PrintInfo("current configuration rotation_speed: " + std::to_string(sensor_configuration->rotation_speed));
    if(UseHttpSetSpinRate())
    {
      SetSpinSpeedAsyncHttp(sensor_configuration->rotation_speed);
    }else{
//      if(wl){
//        tm_.unlock();
//      }
      std::thread t([this, sensor_configuration]{
      SetSpinRate(sensor_configuration->rotation_speed);//, false);
//      SetSpinRate(sensor_configuration->rotation_speed, false);
      });
      t.join();
      /*
      std::thread t([this, sensor_configuration]{SetSpinRate(sensor_configuration->rotation_speed);});
      boost::system::error_code ec;
      m_owned_ctx->run(ec);
      if(ec){
        std::cerr << "HesaiHwInterface::CheckAndSetConfig(RotationSpeed): " << ec.message() << std::endl;
      }
      t.join();
      */
//      if(wl){
//        tm_.lock();
//      }
    }
  }
  /*
  //PTC_COMMAND_SET_LIDAR_RANGE
  target_key = "config.fov.start";
  auto current_cloud_min_angle = tree.get<std::uint16_t>(target_key);
  if(sensor_configuration->cloud_min_angle != current_cloud_min_angle)
  {
    SetFovStartAsync(sensor_configuration->cloud_min_angle);
    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_cloud_min_angle << std::endl;
    std::cout << "sensor_configuration->cloud_min_angle: " << sensor_configuration->cloud_min_angle << std::endl;
  }

  target_key = "config.fov.end";
  auto current_cloud_max_angle = tree.get<std::uint16_t>(target_key);
  if(sensor_configuration->cloud_max_angle != current_cloud_max_angle)
  {
    SetFovEndAsync(sensor_configuration->cloud_max_angle);
    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_cloud_max_angle << std::endl;
    std::cout << "sensor_configuration->cloud_max_angle: " << sensor_configuration->cloud_max_angle << std::endl;
  }
  */

  bool set_flg = false;
  std::stringstream ss;
  ss << hesai_config.dest_ipaddr[0] << "." << hesai_config.dest_ipaddr[1] << "." << hesai_config.dest_ipaddr[2] << "." << hesai_config.dest_ipaddr[3];
  auto current_host_addr = ss.str();
  if(sensor_configuration->host_ip != current_host_addr)
  {
    set_flg = true;
//    std::cout << "current dest_ipaddr: " << current_host_addr << std::endl;
//    std::cout << "sensor_configuration->host_ip: " << sensor_configuration->host_ip << std::endl;
    PrintInfo("current lidar dest_ipaddr: " + current_host_addr);
    PrintInfo("current configuration host_ip: " + sensor_configuration->host_ip);
  }

  auto current_host_dport = hesai_config.dest_LiDAR_udp_port;
  if(sensor_configuration->data_port != current_host_dport)
  {
    set_flg = true;
//    std::cout << "current dest_LiDAR_udp_port: " << current_host_dport << std::endl;
//    std::cout << "sensor_configuration->data_port: " << sensor_configuration->data_port << std::endl;
    PrintInfo("current lidar dest_LiDAR_udp_port: " + std::to_string(current_host_dport));
    PrintInfo("current configuration data_port: " + std::to_string(sensor_configuration->data_port));
  }

  auto current_host_tport = hesai_config.dest_gps_udp_port;
  if(sensor_configuration->gnss_port != current_host_tport)
  {
    set_flg = true;
//    std::cout << "current dest_gps_udp_port: " << current_host_tport << std::endl;
//    std::cout << "sensor_configuration->gnss_port: " << sensor_configuration->gnss_port << std::endl;
    PrintInfo("current lidar dest_gps_udp_port: " + std::to_string(current_host_tport));
    PrintInfo("current configuration gnss_port: " + std::to_string(sensor_configuration->gnss_port));
  }

  if(set_flg){
    std::vector<std::string> list_string;
    boost::split(list_string, sensor_configuration->host_ip, boost::is_any_of("."));
//    if(wl){
//      tm_.unlock();
//    }
    std::thread t([this, sensor_configuration, list_string]{
    SetDestinationIp(
      std::stoi(list_string[0]),
      std::stoi(list_string[1]),
      std::stoi(list_string[2]),
      std::stoi(list_string[3]),
      sensor_configuration->data_port,
      sensor_configuration->gnss_port//,
//      false
    );
    });
    t.join();
//    if(wl){
//      tm_.lock();
//    }
  }

  set_flg = false;
  auto sync_angle = static_cast<int>(hesai_config.sync_angle / 100);
  auto scan_phase = static_cast<int>(sensor_configuration->scan_phase);
  int sync_flg = 0 < scan_phase ? 1 : 0;
  if(hesai_config.sync != sync_flg){
    set_flg = true;
  }else if(0 < sync_flg && scan_phase != sync_angle){
    set_flg = true;
  }
  if(set_flg){
//    std::cout << "current sync: " << hesai_config.sync << std::endl;
//    std::cout << "current sync_angle: " << hesai_config.sync_angle << std::endl;
//    std::cout << "sensor_configuration->scan_phase: " << scan_phase << std::endl;
    PrintInfo("current lidar sync: " + std::to_string(hesai_config.sync));
    PrintInfo("current lidar sync_angle: " + std::to_string(hesai_config.sync_angle));
    PrintInfo("current configuration scan_phase: " + std::to_string(scan_phase));
    std::thread t([this, sync_flg, scan_phase]{
    SetSyncAngle(
      sync_flg,
      scan_phase//,
//      false
    );
    });
    t.join();
  }

#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "End CheckAndSetConfig(HesaiConfig)!!" << std::endl;
#endif
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

HesaiStatus HesaiHwInterface::CheckAndSetConfig(std::shared_ptr<HesaiSensorConfiguration> sensor_configuration, HesaiLidarRangeAll hesai_lidar_range_all)
{
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "Start CheckAndSetConfig(HesaiLidarRangeAll)!!" << std::endl;
#endif
  //*
  //PTC_COMMAND_SET_LIDAR_RANGE
  bool set_flg = false;
  if(hesai_lidar_range_all.method!=0){
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
    std::cout << "current hesai_lidar_range_all.method: " << hesai_lidar_range_all.method << std::endl;
#endif
    set_flg = true;
  }else{
    auto current_cloud_min_angle = hesai_lidar_range_all.start;
    if(static_cast<int>(sensor_configuration->cloud_min_angle * 10) != current_cloud_min_angle)
    {
      set_flg = true;
//      std::cout << "current hesai_lidar_range_all.start: " << current_cloud_min_angle << std::endl;
//      std::cout << "sensor_configuration->cloud_min_angle: " << sensor_configuration->cloud_min_angle << std::endl;
      PrintInfo("current lidar range.start: " + std::to_string(current_cloud_min_angle));
      PrintInfo("current configuration cloud_min_angle: " + std::to_string(sensor_configuration->cloud_min_angle));
    }

    auto current_cloud_max_angle = hesai_lidar_range_all.end;
    if(static_cast<int>(sensor_configuration->cloud_max_angle * 10) != current_cloud_max_angle)
    {
      set_flg = true;
//      std::cout << "current hesai_lidar_range_all.end: " << current_cloud_max_angle << std::endl;
//      std::cout << "current configuration->cloud_max_angle: " << sensor_configuration->cloud_max_angle << std::endl;
      PrintInfo("current lidar range.end: " + std::to_string(current_cloud_max_angle));
      PrintInfo("current configuration cloud_max_angle: " + std::to_string(sensor_configuration->cloud_max_angle));
    }
    }
  //*/

  if(set_flg){
//    if(wl){
//      tm_.unlock();
//    }
    std::thread t([this, sensor_configuration]{
    SetLidarRange(
      static_cast<int>(sensor_configuration->cloud_min_angle * 10),
      static_cast<int>(sensor_configuration->cloud_max_angle * 10)//,
//      false
    );
    });
    t.join();
//    if(wl){
//      tm_.lock();
//    }
  }

//  tcp_driver_->run();
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
  /*
  std::vector<std::thread> thread_pool{};
  thread_pool.emplace_back([this]{
//      auto ctx = std::make_shared<boost::asio::io_service>();
      GetConfig(//ctx,
      [this](HesaiConfig &result)
      {
        std::cout << result << std::endl;
        CheckAndSetConfig(std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
      });
    });
  thread_pool.emplace_back([this]{
//      auto ctx = std::make_shared<boost::asio::io_service>();
      GetLidarRange(//ctx,
      [this](HesaiLidarRangeAll &result)
      {
        std::cout << result << std::endl;
        CheckAndSetConfig(std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
      });
    });
  boost::system::error_code ec;
  m_owned_ctx->run(ec);
  if(ec){
    std::cerr << "HesaiHwInterface::CheckAndSetConfig: " << ec.message() << std::endl;
  }
  for (std::thread &th : thread_pool) {
    th.join();
  }
  */
 //*
//  auto ctx = std::make_shared<boost::asio::io_service>();
  if(true){
  std::thread t([this]{
//      if(tcp_driver_->GetIOContext()->stopped()){
//        tcp_driver_->GetIOContext()->restart();
//      }
      GetConfig(//ctx,
      [this](HesaiConfig &result)
      {
//        std::cout << result << std::endl;
        std::stringstream ss;
        ss << result;
        PrintInfo(ss.str());
        CheckAndSetConfig(std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
      }
      );
//      , false);
//      boost::system::error_code ec = tcp_driver_->run();
//      if(ec){
//        std::cerr << "HesaiHwInterface::CheckAndSetConfig: " << ec.message() << std::endl;
//      }
    });
//  boost::system::error_code ec;
//  ctx->run(ec);
//  if(ec){
//    std::cerr << "HesaiHwInterface::CheckAndSetConfig: " << ec.message() << std::endl;
//  }
  t.join();


  std::thread t2([this]{
//      if(tcp_driver_->GetIOContext()->stopped()){
//        tcp_driver_->GetIOContext()->restart();
//      }
      GetLidarRange(//ctx,
      [this](HesaiLidarRangeAll &result)
      {
//        std::cout << result << std::endl;
        std::stringstream ss;
        ss << result;
        PrintInfo(ss.str());
        CheckAndSetConfig(std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
      }
      );
//  , false);
//      boost::system::error_code ec = tcp_driver_->run();
//      if(ec){
//        std::cerr << "HesaiHwInterface::CheckAndSetConfig: " << ec.message() << std::endl;
//      }
    });
//  ctx->run(ec);
//  if(ec){
//    std::cerr << "HesaiHwInterface::CheckAndSetConfig: " << ec.message() << std::endl;
//  }
  t2.join();
  //*/
  /*
  tcp_driver_s_->GetIOContext()->restart();
  tcp_driver_s_->run();
  tcp_driver_->GetIOContext()->restart();
  tcp_driver_->run();
  */
  }else if(false){

//    auto w( boost::make_shared<boost::asio::io_service::work>( *tcp_driver_->GetIOContext() ) );
//    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> w = boost::asio::make_work_guard(tcp_driver_->GetIOContext());
//    auto w = boost::asio::make_work_guard(tcp_driver_->GetIOContext());
//    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> w(boost::asio::make_work_guard(*(tcp_driver_->GetIOContext())));
    /*
    tcp_driver_->GetIOContext()->post([w]{
      GetConfig(
      [this](HesaiConfig &result)
      {
        std::cout << result << std::endl;
        CheckAndSetConfig(std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
      }
      , false
      );
    });
      */
    GetConfig(
    [this](HesaiConfig &result)
    {
      std::cout << result << std::endl;
      CheckAndSetConfig(std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
    }
//    , false
    );
    /*
    std::cout << "make thread t" << std::endl;
    std::thread t([this]{
      tcp_driver_->GetIOContext()->restart();
      tcp_driver_->run();
    });
    std::cout << "made thread t" << std::endl;
    t.join();
    std::cout << "joined thread t" << std::endl;
    */
//    w.reset( boost::make_shared<boost::asio::io_service::work>( *tcp_driver_->GetIOContext() ) );
//    w.reset();
//    w = boost::asio::make_work_guard(tcp_driver_->GetIOContext());
//    w(boost::asio::make_work_guard(*tcp_driver_->GetIOContext()))
//    w = boost::asio::make_work_guard(*(tcp_driver_->GetIOContext()));
//    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> w2(boost::asio::make_work_guard(*(tcp_driver_->GetIOContext())));
    GetLidarRange(
    [this](HesaiLidarRangeAll &result)
    {
      std::cout << result << std::endl;
      CheckAndSetConfig(std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
    }
//    , false
    );
    std::cout << "make thread t2" << std::endl;
    std::thread t2([this]{
      tcp_driver_->GetIOContext()->restart();
      tcp_driver_->run();
    });
    std::cout << "made thread t2" << std::endl;
    t2.join();
    std::cout << "joined thread t2" << std::endl;
//    w2.reset();

  }else{
    bool stopped = tcp_driver_->GetIOContext()->stopped();
    std::cout << "stopped: " << stopped << std::endl;
    if(stopped)
      tcp_driver_->GetIOContext()->restart();
    GetConfig(
    [this](HesaiConfig &result)
    {
      std::cout << result << std::endl;
      CheckAndSetConfig(std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
    }
    , false
    );
    if(stopped)
      tcp_driver_->run();
    stopped = tcp_driver_->GetIOContext()->stopped();
    std::cout << "stopped2: " << stopped << std::endl;
    if(stopped)
      tcp_driver_->GetIOContext()->restart();
    GetLidarRange(
    [this](HesaiLidarRangeAll &result)
    {
      std::cout << result << std::endl;
      CheckAndSetConfig(std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration_), result);
    }
    , false
    );
    if(stopped)
      tcp_driver_->run();
  }
#ifdef WITH_DEBUG_STDOUT_HESAI_HW_INTERFACE
  std::cout << "End CheckAndSetConfig!!" << std::endl;
#endif
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

void HesaiHwInterface::SetTargetModel(int model){
  target_model_no = model;
}

bool HesaiHwInterface::UseHttpSetSpinRate(int model)
{
  switch (model)
  {
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
bool HesaiHwInterface::UseHttpSetSpinRate()
{
  return UseHttpSetSpinRate(target_model_no);
}
bool HesaiHwInterface::UseHttpGetLidarMonitor(int model)
{
  switch (model)
  {
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
bool HesaiHwInterface::UseHttpGetLidarMonitor()
{
  return UseHttpGetLidarMonitor(target_model_no);
}

bool HesaiHwInterface::CheckLock(std::timed_mutex &tm, int &fail_cnt, const int &fail_cnt_max, std::string name)
{
 if(wl){
//    std::cout << "try_lock_for: " << name << std::endl;
    if (!tm.try_lock_for(std::chrono::milliseconds(timeout_))) {
//      std::cout << "timeout: " << name << std::endl;
      PrintDebug("timeout: " + name);
      fail_cnt ++;
      if(fail_cnt_max < fail_cnt){
        /*
        for(auto td : tcp_drivers_status){
          td->close();
//          td->reset();
        }
        tcp_drivers_status.clear();
        tcp_drivers_status.shrink_to_fit();
        */
        tm.unlock();

        /*
        if(tm == tm_){
          tcp_driver_->close();
          tcp_driver_->open();
        }
        if(tm == tms_){
          tcp_driver_s_->close();
          tcp_driver_s_->open();
        }
        */
        tcp_driver_->close();
        tcp_driver_->open();
        tcp_driver_s_->close();
        tcp_driver_s_->open();

//      return Status::ERROR_1;
      }
      return false;
    }
    fail_cnt = 0;
  }
  return true;
}

void HesaiHwInterface::CheckUnlock(std::timed_mutex &tm, std::string name)
{
  if(wl){
    tm.unlock();
//    std::cout << "unlocked: " << name << std::endl;
    PrintDebug(name + ": finished");
  }
}

void HesaiHwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger = logger;
}

void HesaiHwInterface::PrintInfo(std::string info)
{
  if(parent_node_logger)
  {
    RCLCPP_INFO_STREAM((*parent_node_logger), info);
  }
  else
  {
    std::cout << info << std::endl;
  }
}

void HesaiHwInterface::PrintError(std::string error)
{
  if(parent_node_logger)
  {
    RCLCPP_ERROR_STREAM((*parent_node_logger), error);
  }
  else
  {
    std::cerr << error << std::endl;
  }
}

void HesaiHwInterface::PrintDebug(std::string debug)
{
  if(parent_node_logger)
  {
    RCLCPP_DEBUG_STREAM((*parent_node_logger), debug);
  }
  else
  {
    std::cout << debug << std::endl;
  }
}

void HesaiHwInterface::PrintDebug(const std::vector<uint8_t> & bytes)
{
  std::stringstream ss;
  for(const auto &b :bytes){
    ss << static_cast<int>(b) << ", ";
  }
  ss << std::endl;
  PrintDebug(ss.str());
}


}  // namespace drivers
}  // namespace nebula
