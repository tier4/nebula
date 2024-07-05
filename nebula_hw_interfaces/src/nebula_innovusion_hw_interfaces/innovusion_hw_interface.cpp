#include "nebula_hw_interfaces/nebula_hw_interfaces_innovusion/innovusion_hw_interface.hpp"

namespace nebula
{
namespace drivers
{
InnovusionHwInterface::InnovusionHwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  m_owned_ctx{new boost::asio::io_context(1)},
  m_owned_ctx_s{new boost::asio::io_context(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  http_client_driver_{new ::drivers::tcp_driver::HttpClientDriver(m_owned_ctx)},
  scan_cloud_ptr_{std::make_unique<innovusion_msgs::msg::InnovusionScan>()}
{
}

Status InnovusionHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  InnovusionStatus status = Status::OK;
  try {
    sensor_configuration_ = std::static_pointer_cast<InnovusionSensorConfiguration>(sensor_configuration);
  } catch (const std::exception & ex) {
    status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << status << std::endl;
    return status;
  }
  return Status::OK;
}

Status InnovusionHwInterface::CloudInterfaceStart()
{
  try {
    InitHttpClientDriver();

    std::cout << "Starting UDP server on: " << *sensor_configuration_ << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port, kInnoPktMax);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();
    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&InnovusionHwInterface::ReceiveCloudPacketCallback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status InnovusionHwInterface::RegisterScanCallback(
  std::function<void(std::unique_ptr<innovusion_msgs::msg::InnovusionScan>)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
  return Status::OK;
}

bool InnovusionHwInterface::IsPacketValid(const std::vector<uint8_t> & buffer) {
  uint32_t send_packet_size = 0;
  std::memcpy(&send_packet_size, &buffer[kInnoPktSizeSectionIndex], kInnoPktSizeSectionLength);

  if (buffer.size() < send_packet_size) {
    std::cout << "receive buffer size " << buffer.size() << " < packet size " << send_packet_size
              << std::endl;
    return false;
  }

  uint16_t magic_number = ((buffer[1] << 8) | buffer[0]);
  uint16_t packet_type = buffer[kInnoPktTypeIndex];

  // check packet is data packet
  if ((magic_number != kInnoMagicNumberDataPacket) ||
      (packet_type == 2) || (packet_type == 3)) {
    return false;
  }

  return true;
}

void InnovusionHwInterface::ProtocolCompatibility(std::vector<uint8_t> & buffer) {
  uint8_t major_version = buffer[kInnoPktMajorVersionSection];
  uint32_t* packet_size = reinterpret_cast<uint32_t *>(&buffer[kInnoPktSizeSectionIndex]);

  if (major_version == kInnoProtocolMajorV1) {
    // add 16 bytes to the headr
    *packet_size += 16;
    buffer.insert(buffer.begin() + kInnoProtocolOldHeaderLen, 16, 0);
  }
}

void InnovusionHwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer)
{
  std::vector<uint8_t> buffer_copy = buffer;
  if (!IsPacketValid(buffer_copy)) {
    return;
  }

  ProtocolCompatibility(buffer_copy);

  uint64_t packet_id = 0;
  std::memcpy(&packet_id, &buffer_copy[kInnoPktIdSection], kInnoPktIdLength);
  if (current_packet_id == 0) {
    current_packet_id = packet_id;
    std::cout << "First packet received" << std::endl;
  }

  // publish the whole frame data
  if ((scan_reception_callback_ != nullptr) && (current_packet_id != packet_id) && (scan_cloud_ptr_->size > 0)) {
    scan_cloud_ptr_->header.stamp = scan_cloud_ptr_->packets[0].stamp;
    scan_reception_callback_(std::move(scan_cloud_ptr_));
    scan_cloud_ptr_ = std::make_unique<innovusion_msgs::msg::InnovusionScan>();
    scan_cloud_ptr_->packets.clear();
    scan_cloud_ptr_->size = 0;
    current_packet_id = packet_id;
  }

  // add the point to the frame
  uint32_t buffer_size = buffer_copy.size();
  innovusion_msgs::msg::InnovusionPacket innovusion_packet;
  uint8_t *p1 = &innovusion_packet.data[0];
  memcpy(p1, &buffer_copy[0], buffer_size);
  auto now = std::chrono::system_clock::now();
  auto now_secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  auto now_nanosecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  innovusion_packet.stamp.sec = static_cast<int>(now_secs);
  innovusion_packet.stamp.nanosec =
    static_cast<int>((now_nanosecs / 1000000000. - static_cast<double>(now_secs)) * 1000000000);
  scan_cloud_ptr_->packets.emplace_back(innovusion_packet);
  ++scan_cloud_ptr_->size;
}

Status InnovusionHwInterface::CloudInterfaceStop() { return Status::ERROR_1; }

Status InnovusionHwInterface::GetSensorConfiguration(SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  PrintDebug(ss.str());
  return Status::ERROR_1;
}

Status InnovusionHwInterface::GetCalibrationConfiguration(
  CalibrationConfigurationBase & calibration_configuration)
{
  PrintDebug(calibration_configuration.calibration_file);
  return Status::ERROR_1;
}

void InnovusionHwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger = logger;
}

void InnovusionHwInterface::PrintInfo(std::string info)
{
  if (parent_node_logger) {
    RCLCPP_INFO_STREAM((*parent_node_logger), info);
  } else {
    std::cout << info << std::endl;
  }
}

void InnovusionHwInterface::PrintError(std::string error)
{
  if (parent_node_logger) {
    RCLCPP_ERROR_STREAM((*parent_node_logger), error);
  } else {
    std::cerr << error << std::endl;
  }
}

void InnovusionHwInterface::PrintDebug(std::string debug)
{
  if (parent_node_logger) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

std::string InnovusionHwInterface::GetSensorParameter(const std::string &key) {
  auto rt = http_client_driver_->get("/command/?get_" + key);
  http_client_driver_->client()->close();
  return rt;
}

std::string InnovusionHwInterface::SetSensorParameter(const std::string &key, const std::string &value) {
  auto rt = http_client_driver_->get("/command/?set_" + key + "=" + value);
  http_client_driver_->client()->close();
  return rt;
}

void InnovusionHwInterface::InitHttpClientDriver() {
  uint16_t data_port;
  uint16_t status_port;
  uint16_t message_port;
  char ip[20]{0};

  http_client_driver_->init_client(sensor_configuration_->sensor_ip, 8010);
  DisplayCommonVersion();

  std::string strUdpInfo = GetSensorParameter("udp_ports_ip");

  if (sscanf(strUdpInfo.c_str(), "%hu,%hu,%hu,%*32[^,],%32[^,], %*s", &data_port, &status_port, &message_port, ip) == 4) {
    std::cout << "udp_info:" << data_port << "," << status_port << "," << message_port << "," << ip << std::endl;

    if (IsBroadcast(ip)) {
      // broadcast ip, do nothing
      sensor_configuration_->host_ip = "0.0.0.0";
      std::cout << "broadcast ip" << std::endl;
    } else if (IsMulticast(ip)) {
      // multicast ip
      std::cout << "multicast ip" << std::endl;
      AddToMuiticastGroup(ip, data_port, sensor_configuration_->host_ip, sensor_configuration_->data_port);
    } else {
      // unicast ip
      std::cout << "unicast ip" << std::endl;
      UpdateUnicastIpPort(ip, data_port, sensor_configuration_->host_ip, sensor_configuration_->data_port);
    }
  }
}

void InnovusionHwInterface::DisplayCommonVersion() {
  std::cout<<"*************** Innovusion Lidar Version Info ***************"<<std::endl;
  std::cout<<"sw_version:"<<GetSensorParameter("sw_version")<<std::endl;
  std::cout<<"sdk_version:"<<GetSensorParameter("sdk_version")<<std::endl;
  std::cout<<"lidar_id:"<<GetSensorParameter("lidar_id")<<std::endl;
  std::cout<<"fw_version:"<<GetSensorParameter("fw_version")<<std::endl;
  std::cout<<"sn:"<<GetSensorParameter("sn")<<std::endl;
  std::cout<<"*************************************************************"<<std::endl;
}

bool InnovusionHwInterface::IsBroadcast(std::string strIp) {
  bool bRet = false;
  if (strIp.size() > 4 && strcmp(strIp.c_str() + strIp.size() - 4, ".255") == 0) {
    bRet = true;
  }
  return bRet;
}

bool InnovusionHwInterface::IsMulticast(std::string strIp) {
  bool bRet = false;
  uint16_t uHead = 0;
  if (sscanf(strIp.c_str(), "%hu", &uHead) == 1) {
    if (uHead >= 224 && uHead <= 239) {
      bRet = true;
    }
  }
  return bRet;
}

void InnovusionHwInterface::AddToMuiticastGroup(const std::string &strMuiticastIp, const uint16_t &uMulticastPort,
                                                const std::string &strLocalIp, const uint16_t &uLocalPort) {
  //TODO: multicast access to point cloud data is not supported
  std::cout<<"Attention!!!!!!!!! Currently, multicast access to point cloud data is not supported! We shall change to unicast!"<<std::endl;
  UpdateUnicastIpPort(strMuiticastIp, uMulticastPort, strLocalIp, uLocalPort);
}

void InnovusionHwInterface::UpdateUnicastIpPort(const std::string &strUnicastIp, const uint16_t &uUnicastPort,
                                                const std::string &strLocalIp, const uint16_t &uLocalPort) {
  if(strUnicastIp == strLocalIp && uUnicastPort == uLocalPort) {
    std::cout << "UpdateUnicastIpPort, same ip and port" << std::endl;
    return;
  }

  if(sensor_configuration_->sensor_model == nebula::drivers::SensorModel::INNOVUSION_ROBIN ||
     sensor_configuration_->sensor_model == nebula::drivers::SensorModel::INNOVUSION_FALCON){
    std::string strSendInfo = std::to_string(uLocalPort) + "," + std::to_string(uLocalPort)
                              + "," + std::to_string(uLocalPort);
    std::cout << "UpdateFalconUnicastIpPort, strSendInfo:" << strSendInfo << std::endl;

    SetSensorParameter("udp_ports_ip", strSendInfo);
  } else {
    std::cout << "UpdateUnicastIpPort, not support sensor model" << std::endl;
  }
}

void InnovusionHwInterface::RobinSetUdpPortIP(std::string strUdpIp, uint16_t uUdpPort)
{
  std::string strSendInfo = "{\"ip\":\"" + strUdpIp + "\",\"port_data\":" + std::to_string(uUdpPort)
    + ",\"port_message\":" + std::to_string(uUdpPort) + ",\"port_status\":" + std::to_string(uUdpPort) + "}";
  std::cout << "UpdateFalconUnicastIpPort, strSendInfo:" << strSendInfo << std::endl;

  auto rt = http_client_driver_->post("/v1/lidar/udp_ports_ip", strSendInfo);
  http_client_driver_->client()->close();
  std::cout << "UpdateRobinUnicastIpPort, rt:" << rt << std::endl;
}

boost::property_tree::ptree InnovusionHwInterface::ParseJson(const std::string &str) {
  boost::property_tree::ptree tree;
  try {
    std::stringstream ss;
    ss << str;
    boost::property_tree::read_json(ss, tree);
  } catch (boost::property_tree::json_parser_error & e) {
    std::cerr << "Error on ParseJson:" << e.what() << std::endl;
  }
  return tree;
}

Status InnovusionHwInterface::GetHttpClientDriverOnce(
  std::shared_ptr<boost::asio::io_context> ctx,
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd)
{
  hcd = std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>(
    new ::drivers::tcp_driver::HttpClientDriver(ctx));
  try {
    hcd->init_client(sensor_configuration_->sensor_ip, 8088);
  } catch (const std::exception & ex) {
    Status status = Status::HTTP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << "," << 8088 << std::endl;
    return Status::HTTP_CONNECTION_ERROR;
  }
  return Status::OK;
}

Status InnovusionHwInterface::GetSnapshotAsync(
  std::function<void(const std::string & str)> str_callback, const std::string & snapshot)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if (st != Status::OK) {
    return st;
  }
  hcd->asyncGet(str_callback, snapshot);
  ctx->run();
  return Status::OK;
}

}  // namespace drivers
}  // namespace nebula
