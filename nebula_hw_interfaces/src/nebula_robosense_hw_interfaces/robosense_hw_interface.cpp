#include "nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp"

namespace nebula
{
namespace drivers
{
RobosenseHwInterface::RobosenseHwInterface()
: msop_io_context_{new ::drivers::common::IoContext(1)},
  difop_io_context_{new ::drivers::common::IoContext(1)},
  m_owned_ctx{new boost::asio::io_context(1)},
  msop_udp_driver_{new ::drivers::udp_driver::UdpDriver(*msop_io_context_)},
  difop_udp_driver_{new ::drivers::udp_driver::UdpDriver(*difop_io_context_)},
  tcp_driver_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx)},
  scan_cloud_ptr_{std::make_unique<robosense_msgs::msg::RobosenseScan>()}
{
}

Status RobosenseHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  RobosenseStatus status = Status::OK;
  try {
    sensor_configuration_ =
      std::static_pointer_cast<RobosenseSensorConfiguration>(sensor_configuration);
    if (sensor_configuration_ != nullptr) {
      if (
        sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
        sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
        is_valid_packet_ = [](size_t packet_size) { return (packet_size == HELIOS_PACKET_SIZE); };
        azimuth_index_ = 44;
      }
    } else {
      status = Status::SENSOR_CONFIG_ERROR;
      std::cerr << status << std::endl;
      return status;
    }
  } catch (const std::exception & ex) {
    status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << status << std::endl;
    return status;
  }
  return Status::OK;
}

Status RobosenseHwInterface::CloudInterfaceStart()
{
  try {
    msop_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
    msop_udp_driver_->receiver()->open();
    msop_udp_driver_->receiver()->bind();

    msop_udp_driver_->receiver()->asyncReceive(
      std::bind(&RobosenseHwInterface::ReceiveCloudPacketCallback, this, std::placeholders::_1));

    difop_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->difop_port);
    difop_udp_driver_->receiver()->open();
    difop_udp_driver_->receiver()->bind();
    difop_udp_driver_->receiver()->asyncReceive(
      std::bind(&RobosenseHwInterface::ReceiveDifopPacketCallback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    return status;
  }

  // fov
  // SetFovRange(0, 360);
  return Status::OK;
}

Status RobosenseHwInterface::RegisterScanCallback(
  std::function<void(std::unique_ptr<robosense_msgs::msg::RobosenseScan>)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
  return Status::OK;
}

void RobosenseHwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer)
{
  std::lock_guard<std::mutex> lg(scan_cloud_ptr_mtx_);
  if (!is_valid_packet_(buffer.size())) {
    PrintDebug("Invalid Packet: " + std::to_string(buffer.size()));
    return;
  }

  uint32_t buffer_size = buffer.size();
  std::array<uint8_t, MTU_SIZE> packet_data{};
  std::copy_n(std::make_move_iterator(buffer.begin()), buffer_size, packet_data.begin());
  robosense_msgs::msg::RobosensePacket robosense_packet;
  robosense_packet.is_difop = 0;
  robosense_packet.data = packet_data;
  robosense_packet.size = buffer_size;
  auto now = std::chrono::system_clock::now();
  auto now_secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  auto now_nanosecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  robosense_packet.stamp.sec = static_cast<int>(now_secs);
  robosense_packet.stamp.nanosec =
    static_cast<int>((now_nanosecs / 1000000000. - static_cast<double>(now_secs)) * 1000000000);
  scan_cloud_ptr_->packets.emplace_back(robosense_packet);

  int current_phase = 0;
  bool comp_flg = false;
  const auto & data = scan_cloud_ptr_->packets.back().data;
  current_phase = (data[azimuth_index_ + 1] & 0xff) + ((data[azimuth_index_] & 0xff) << 8);

  current_phase = (static_cast<int>(current_phase) + 36000) % 36000;
  if (current_phase >= prev_phase_ || scan_cloud_ptr_->packets.size() < 3) {
    prev_phase_ = current_phase;
  } else {
    comp_flg = true;
  }

  if (comp_flg) {  // Scan complete
    if (scan_reception_callback_) {
      scan_cloud_ptr_->header.stamp = scan_cloud_ptr_->packets.front().stamp;
      // Callback
      scan_reception_callback_(std::move(scan_cloud_ptr_));
      scan_cloud_ptr_ = std::make_unique<robosense_msgs::msg::RobosenseScan>();
    }
  }
}

void RobosenseHwInterface::ReceiveDifopPacketCallback(const std::vector<uint8_t> & buffer)
{
  std::lock_guard<std::mutex> lg(scan_cloud_ptr_mtx_);
  if (!is_valid_packet_(buffer.size())) {
    PrintDebug("Invalid Packet: " + std::to_string(buffer.size()));
    return;
  }
  uint32_t buffer_size = buffer.size();
  std::array<uint8_t, MTU_SIZE> packet_data{};
  std::copy_n(std::make_move_iterator(buffer.begin()), buffer_size, packet_data.begin());
  robosense_msgs::msg::RobosensePacket robosense_packet;
  robosense_packet.is_difop = 1;
  robosense_packet.data = packet_data;
  robosense_packet.size = buffer_size;
  auto now = std::chrono::system_clock::now();
  auto now_secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  auto now_nanosecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  robosense_packet.stamp.sec = static_cast<int>(now_secs);
  robosense_packet.stamp.nanosec =
    static_cast<int>((now_nanosecs / 1000000000. - static_cast<double>(now_secs)) * 1000000000);
  if (scan_cloud_ptr_ != nullptr) {
    scan_cloud_ptr_->packets.emplace_back(robosense_packet);
  }
}

void RobosenseHwInterface::ReceiveTcpPacketCallback(const std::vector<uint8_t> & buffer)
{
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    if (
      buffer.size() == 304 && buffer[0] == 0x52 && buffer[1] == 0x25 && buffer[2] == 0xaa &&
      buffer[3] == 0x55) {
      RobosenseNetFrameHead frameHead;
      unsigned char * buffer_ptr = const_cast<unsigned char *>(buffer.data());
      memcpy(&configParameter_, buffer_ptr + sizeof(frameHead), sizeof(configParameter_));
    }
  }
}

Status RobosenseHwInterface::GetLidarInfo()
{
  std::vector<unsigned char> buf_tcp_send_vec_;
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    buf_tcp_send_vec_ = FrameHeadPack(NET_CMD_READ_CONFIG, 0);
    SendAndGetConfig(buf_tcp_send_vec_);
  }
  return Status::OK;
}

Status RobosenseHwInterface::SetLidarIp(const uint8_t lidar_ip[4])
{
  GetLidarInfo();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::vector<unsigned char> buf_tcp_send_vec_;
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    if (configParameter_.msopPort == 0 || configParameter_.difopPort == 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    buf_tcp_send_vec_ = FrameHeadPack(NET_CMD_WRITE_CONFIG, sizeof(configParameter_));
    memcpy(configParameter_.ipLocal, lidar_ip, 4);
    unsigned char * m = reinterpret_cast<unsigned char *>(&configParameter_);
    for (unsigned int i = 0; i < sizeof(configParameter_); ++i) {
      buf_tcp_send_vec_.push_back(m[i]);
    }
    SendAndGetConfig(buf_tcp_send_vec_);
  }
  return Status::OK;
}

Status RobosenseHwInterface::SetRemoteIp(const uint8_t remote_ip[4])
{
  GetLidarInfo();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::vector<unsigned char> buf_tcp_send_vec_;
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    if (configParameter_.msopPort == 0 || configParameter_.difopPort == 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    buf_tcp_send_vec_ = FrameHeadPack(NET_CMD_WRITE_CONFIG, sizeof(configParameter_));
    memcpy(configParameter_.ipRemote, remote_ip, 4);
    unsigned char * m = reinterpret_cast<unsigned char *>(&configParameter_);
    for (unsigned int i = 0; i < sizeof(configParameter_); ++i) {
      buf_tcp_send_vec_.push_back(m[i]);
    }
    SendAndGetConfig(buf_tcp_send_vec_);
  }
  return Status::OK;
}

Status RobosenseHwInterface::SetNetMask(const uint8_t mask[4])
{
  GetLidarInfo();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::vector<unsigned char> buf_tcp_send_vec_;
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    if (configParameter_.msopPort == 0 || configParameter_.difopPort == 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    buf_tcp_send_vec_ = FrameHeadPack(NET_CMD_WRITE_CONFIG, sizeof(configParameter_));
    memcpy(configParameter_.netmaskLocal, mask, 4);
    unsigned char * m = reinterpret_cast<unsigned char *>(&configParameter_);
    for (unsigned int i = 0; i < sizeof(configParameter_); ++i) {
      buf_tcp_send_vec_.push_back(m[i]);
    }
    SendAndGetConfig(buf_tcp_send_vec_);
  }
  return Status::OK;
}

Status RobosenseHwInterface::SetNetGateWay(const uint8_t gateway[4])
{
  GetLidarInfo();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::vector<unsigned char> buf_tcp_send_vec_;
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    if (configParameter_.msopPort == 0 || configParameter_.difopPort == 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    buf_tcp_send_vec_ = FrameHeadPack(NET_CMD_WRITE_CONFIG, sizeof(configParameter_));
    memcpy(configParameter_.gatewayLocal, gateway, 4);
    unsigned char * m = reinterpret_cast<unsigned char *>(&configParameter_);
    for (unsigned int i = 0; i < sizeof(configParameter_); ++i) {
      buf_tcp_send_vec_.push_back(m[i]);
    }
    SendAndGetConfig(buf_tcp_send_vec_);
  }
  return Status::OK;
}

Status RobosenseHwInterface::SetLidarMsopPort(const uint16_t & msop_port)
{
  GetLidarInfo();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::vector<unsigned char> buf_tcp_send_vec_;
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    if (configParameter_.msopPort == 0 || configParameter_.difopPort == 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    buf_tcp_send_vec_ = FrameHeadPack(NET_CMD_WRITE_CONFIG, sizeof(configParameter_));
    configParameter_.msopPort = msop_port;
    unsigned char * m = reinterpret_cast<unsigned char *>(&configParameter_);
    for (unsigned int i = 0; i < sizeof(configParameter_); ++i) {
      buf_tcp_send_vec_.push_back(m[i]);
    }
    SendAndGetConfig(buf_tcp_send_vec_);
  }
  return Status::OK;
}

Status RobosenseHwInterface::SetLidarDifopPort(const uint16_t & difop_port)
{
  GetLidarInfo();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::vector<unsigned char> buf_tcp_send_vec_;
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    if (configParameter_.msopPort == 0 || configParameter_.difopPort == 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    buf_tcp_send_vec_ = FrameHeadPack(NET_CMD_WRITE_CONFIG, sizeof(configParameter_));
    configParameter_.difopPort = difop_port;
    unsigned char * m = reinterpret_cast<unsigned char *>(&configParameter_);
    for (unsigned int i = 0; i < sizeof(configParameter_); ++i) {
      buf_tcp_send_vec_.push_back(m[i]);
    }
    SendAndGetConfig(buf_tcp_send_vec_);
  }
  return Status::OK;
}

Status RobosenseHwInterface::SetReturnMode(const uint16_t & wave_mode)
{
  GetLidarInfo();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::vector<unsigned char> buf_tcp_send_vec_;
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    if (configParameter_.msopPort == 0 || configParameter_.difopPort == 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    if (wave_mode > 3) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    buf_tcp_send_vec_ = FrameHeadPack(NET_CMD_WRITE_CONFIG, sizeof(configParameter_));
    configParameter_.param.ldPara.waveMode = wave_mode;
    unsigned char * m = reinterpret_cast<unsigned char *>(&configParameter_);
    for (unsigned int i = 0; i < sizeof(configParameter_); ++i) {
      buf_tcp_send_vec_.push_back(m[i]);
    }
    SendAndGetConfig(buf_tcp_send_vec_);
  }
  return Status::OK;
}

Status RobosenseHwInterface::SetFovRange(const float & fov_start, const float & fov_end)
{
  GetLidarInfo();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::vector<unsigned char> buf_tcp_send_vec_;
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    if (configParameter_.msopPort == 0 || configParameter_.difopPort == 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    if (fov_start > fov_end || fov_end > 360 || fov_start < 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    buf_tcp_send_vec_ = FrameHeadPack(NET_CMD_WRITE_CONFIG, sizeof(configParameter_));
    configParameter_.param.ldPara.startFov = fov_start * 100;
    configParameter_.param.ldPara.endFov = fov_end * 100;
    unsigned char * m = reinterpret_cast<unsigned char *>(&configParameter_);
    for (unsigned int i = 0; i < sizeof(configParameter_); ++i) {
      buf_tcp_send_vec_.push_back(m[i]);
    }
    SendAndGetConfig(buf_tcp_send_vec_);
  }
  return Status::OK;
}

Status RobosenseHwInterface::SetRpmAsync(uint16_t rpm)
{
  GetLidarInfo();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::vector<unsigned char> buf_tcp_send_vec_;
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    if (configParameter_.msopPort == 0 || configParameter_.difopPort == 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    if (rpm < 300 || 1200 < rpm || rpm % 60 != 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    buf_tcp_send_vec_ = FrameHeadPack(NET_CMD_WRITE_CONFIG, sizeof(configParameter_));
    configParameter_.param.ldPara.motorSpeed = rpm;
    unsigned char * m = reinterpret_cast<unsigned char *>(&configParameter_);
    for (unsigned int i = 0; i < sizeof(configParameter_); ++i) {
      buf_tcp_send_vec_.push_back(m[i]);
    }
    SendAndGetConfig(buf_tcp_send_vec_);
  }
  return Status::OK;
}

Status RobosenseHwInterface::SetTimeSyncMode(const uint16_t & time_sync_mode)
{
  GetLidarInfo();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::vector<unsigned char> buf_tcp_send_vec_;
  if (
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
    if (configParameter_.msopPort == 0 || configParameter_.difopPort == 0) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    if (time_sync_mode > 4) {
      return Status::SENSOR_CONFIG_ERROR;
    }
    buf_tcp_send_vec_ = FrameHeadPack(NET_CMD_WRITE_CONFIG, sizeof(configParameter_));
    configParameter_.param.ldPara.timeSyncMode = time_sync_mode;
    unsigned char * m = reinterpret_cast<unsigned char *>(&configParameter_);
    for (unsigned int i = 0; i < sizeof(configParameter_); ++i) {
      buf_tcp_send_vec_.push_back(m[i]);
    }
    SendAndGetConfig(buf_tcp_send_vec_);
  }
  return Status::OK;
}

Status RobosenseHwInterface::CloudInterfaceStop()
{
  return Status::ERROR_1;
}

Status RobosenseHwInterface::GetSensorConfiguration(SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  PrintDebug(ss.str());
  return Status::ERROR_1;
}

Status RobosenseHwInterface::GetCalibrationConfiguration(
  CalibrationConfigurationBase & calibration_configuration)
{
  PrintDebug(calibration_configuration.calibration_file);
  return Status::ERROR_1;
}

Status RobosenseHwInterface::SendAndGetConfig(
  std::vector<unsigned char> buf_tcp_send_vec_, bool with_run)
{
  if (with_run) {
    if (tcp_driver_->GetIOContext()->stopped()) {
      tcp_driver_->GetIOContext()->restart();
    }
  }
  if (!is_tcp_driver_init_) {
    if (
      sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS ||
      sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS16P) {
      tcp_driver_->init_socket(
        sensor_configuration_->sensor_ip, sensor_configuration_->data_port,
        sensor_configuration_->host_ip, RobosenseTcpHostPort);
    }
    is_tcp_driver_init_ = true;
  }
  tcp_driver_->asyncSendReceive(
    buf_tcp_send_vec_,
    std::bind(&RobosenseHwInterface::ReceiveTcpPacketCallback, this, std::placeholders::_1));
  if (with_run) {
    boost::system::error_code ec = tcp_driver_->run();
    if (ec) {
      PrintError("RobosenseHwInterface::SendAndGetConfig: " + ec.message());
    }
  }
  return Status::OK;
}

void RobosenseHwInterface::IOContextRun()
{
  m_owned_ctx->run();
}

std::shared_ptr<boost::asio::io_context> RobosenseHwInterface::GetIOContext()
{
  return m_owned_ctx;
}

void RobosenseHwInterface::str_cb(const std::string & str)
{
  PrintInfo(str);
}

void RobosenseHwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger = logger;
}

void RobosenseHwInterface::PrintInfo(std::string info)
{
  if (parent_node_logger) {
    RCLCPP_INFO_STREAM((*parent_node_logger), info);
  } else {
    std::cout << info << std::endl;
  }
}

void RobosenseHwInterface::PrintError(std::string error)
{
  if (parent_node_logger) {
    RCLCPP_ERROR_STREAM((*parent_node_logger), error);
  } else {
    std::cerr << error << std::endl;
  }
}

void RobosenseHwInterface::PrintDebug(std::string debug)
{
  if (parent_node_logger) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

void RobosenseHwInterface::PrintDebug(const std::vector<uint8_t> & bytes)
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
