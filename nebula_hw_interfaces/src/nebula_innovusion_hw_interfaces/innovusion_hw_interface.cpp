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
  tcp_driver_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx)},
  tcp_driver_s_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx_s)},
  scan_cloud_ptr_{std::make_unique<innovusion_msgs::msg::InnovusionScan>()}
{
}

Status InnovusionHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  InnovusionStatus status = Status::OK;
  try {
    sensor_configuration_ =
      std::static_pointer_cast<InnovusionSensorConfiguration>(sensor_configuration);
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

boost::property_tree::ptree InnovusionHwInterface::ParseJson(const std::string & str)
{
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
