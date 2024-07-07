#include "nebula_hw_interfaces/nebula_hw_interfaces_seyond/seyond_hw_interface.hpp"

#include <sstream>

namespace nebula
{
namespace drivers
{
SeyondHwInterface::SeyondHwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  sensor_configuration_(std::make_shared<const SeyondSensorConfiguration>()),
  cloud_packet_callback_(nullptr),
  m_owned_ctx_{new boost::asio::io_context(1)},
  command_tcp_driver_{new ::drivers::tcp_driver::TcpDriver(m_owned_ctx_)}
{
}

SeyondHwInterface::~SeyondHwInterface()
{
}

Status SeyondHwInterface::InitializeTcpDriver()
{
  // Initialize command sender which communicate with the device via TCP
  try {
    command_tcp_driver_->init_socket(
      sensor_configuration_->sensor_ip, SeyondTcpCommandPort, sensor_configuration_->host_ip,
      SeyondTcpCommandPort);
  } catch (const std::exception & ex) {
    std::stringstream ss;
    ss << "SeyondHwInterface::SensorInterfaceStart (init TCP): " << Status::ERROR_1 << std::endl;
    PrintError(ss.str());
    command_tcp_driver_->closeSync();
    return Status::ERROR_1;
  }

  DisplayCommonVersion();
  return Status::OK;
}

void SeyondHwInterface::ReceiveSensorPacketCallback(std::vector<uint8_t> & buffer)
{
  cloud_packet_callback_(buffer);
}

Status SeyondHwInterface::SensorInterfaceStart()
{
  StartUdpStreaming();

  // Initialize stream receiver which communicate with the device via UDP
  try {
    std::cout << "Starting UDP server on: " << *sensor_configuration_ << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port, kSeyondPktMax);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();
    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&SeyondHwInterface::ReceiveSensorPacketCallback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ", "
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status SeyondHwInterface::SensorInterfaceStop()
{
  return Status::ERROR_1;
}

Status SeyondHwInterface::SetSensorConfiguration(
  const std::shared_ptr<const SensorConfigurationBase> & sensor_configuration)
{
  if (!(sensor_configuration->sensor_model == SensorModel::SEYOND_FALCON_KINETIC ||
        sensor_configuration->sensor_model == SensorModel::SEYOND_ROBIN_W)) {
    return Status::INVALID_SENSOR_MODEL;
  }

  sensor_configuration_ =
    std::static_pointer_cast<const SeyondSensorConfiguration>(sensor_configuration);

  return Status::OK;
}

Status SeyondHwInterface::GetSensorConfiguration(
  const SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  PrintDebug(ss.str());
  return Status::ERROR_1;
}

Status SeyondHwInterface::GetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  PrintDebug(calibration_configuration.calibration_file);
  return Status::ERROR_1;
}

Status SeyondHwInterface::RegisterScanCallback(
  std::function<void(std::vector<uint8_t> &)> scan_callback)
{
  cloud_packet_callback_ = std::move(scan_callback);
  return Status::OK;
}

// void SeyondHwInterface::SetTargetModel(nebula::drivers::SensorModel model)
// {
//   target_model_no = static_cast<int>(model);
// }

void SeyondHwInterface::PrintError(std::string error)
{
  if (parent_node_logger_) {
    RCLCPP_ERROR_STREAM((*parent_node_logger_), error);
  } else {
    std::cerr << error << std::endl;
  }
}

void SeyondHwInterface::PrintDebug(std::string debug)
{
  if (parent_node_logger_) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger_), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

void SeyondHwInterface::PrintInfo(std::string info)
{
  if (parent_node_logger_) {
    RCLCPP_INFO_STREAM((*parent_node_logger_), info);
  } else {
    std::cout << info << std::endl;
  }
}

void SeyondHwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger_ = logger;
}

void SeyondHwInterface::DisplayCommonVersion()
{
  std::stringstream info;
  info << "*************** Innovusion Lidar Version Info ***************" << std::endl;
  info << "sw_version:" << SendCommand("get_version") << std::endl;
  info << "sn:" << SendCommand("get_sn") << std::endl;
  info << "framerate:" << SendCommand("get_i_config motor galvo_framerate") << std::endl;
  info << "*************************************************************" << std::endl;
  PrintDebug(info.str());
}

std::string SeyondHwInterface::SendCommand(std::string command)
{
  if (command_tcp_driver_->GetIOContext()->stopped()) {
    command_tcp_driver_->GetIOContext()->restart();
  }

  std::vector<unsigned char> payload;
  std::copy(command.begin(), command.end(), std::back_inserter(payload));
  payload.emplace_back('\n');
  payload.emplace_back(0);

  command_tcp_driver_->asyncSend(payload);
  command_tcp_driver_->run();

  boost::asio::streambuf rec_buf;
  command_tcp_driver_->socket()->receive(rec_buf);
  std::string rec_str = boost::asio::buffer_cast<const char *>(rec_buf.data());
  command_tcp_driver_->closeSync();
  return rec_str;
}

Status SeyondHwInterface::StartUdpStreaming()
{
  // XXX: This streaming-starting command need to be sent via HTTP as of now
  auto stream_starter_ctx = std::make_shared<boost::asio::io_context>(1);
  auto stream_starter_http_driver = std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>(
    new ::drivers::tcp_driver::HttpClientDriver(stream_starter_ctx));

  // Initialize stream starter which communicate with the device via HTTP
  try {
    stream_starter_http_driver->init_client(
      sensor_configuration_->sensor_ip, SeyondHttpCommandPort);
  } catch (const std::exception & ex) {
    std::stringstream ss;
    ss << "SeyondHwInterface::SensorInterfaceStart (init HTTP): " << Status::HTTP_CONNECTION_ERROR
       << std::endl;
    PrintError(ss.str());
    return Status::HTTP_CONNECTION_ERROR;
  }

  // XXX: Currently, one port is used to receive data, status, and message
  uint16_t data_port = sensor_configuration_->data_port;
  uint16_t status_port = sensor_configuration_->data_port;
  uint16_t message_port = sensor_configuration_->data_port;
  Status stat;
  switch (sensor_configuration_->sensor_model) {
    case SensorModel::SEYOND_FALCON_KINETIC: {
      std::string payload = std::to_string(data_port) + "," + std::to_string(message_port) + "," +
                            std::to_string(status_port);
      std::string http_command = "/command/?set_udp_ports_ip=" + payload;
      auto response = stream_starter_http_driver->get(http_command);
      stat = Status::OK;
      break;
    }
    case SensorModel::SEYOND_ROBIN_W: {
      std::string payload = std::to_string(data_port) + "," + std::to_string(message_port) + "," +
                            std::to_string(status_port);
      std::string http_command = "/command/?set_udp_ports_ip=" + payload;
      auto response = stream_starter_http_driver->get(http_command);
      stat = Status::OK;
      break;
    }
    default: {
      PrintError("Invalid sensor model was specified");
      stat = Status::INVALID_SENSOR_MODEL;
    }
  }
  stream_starter_http_driver->client()->close();

  return stat;
}

Status SeyondHwInterface::SetReturnMode(int return_mode)
{
  std::string command =
    "set_i_config manufacture multiple_return_mode " + std::to_string(return_mode);
  SendCommand(command);
  return Status::OK;
}

Status SeyondHwInterface::SetPtpMode(PtpProfile profile)
{
  std::string command = "set_i_config time ntp_en 0";  // Disable NTP first just in case
  SendCommand(command);

  command = "set_i_config time ptp_en 1";
  SendCommand(command);

  // Show messages regarding support status
  if (profile != PtpProfile::IEEE_1588v2 && profile != PtpProfile::IEEE_802_1AS_AUTO) {
    PrintInfo("Unsupported PTP profile was selected. Falling back to IEEE 1588v2");
  } else if (profile == PtpProfile::IEEE_802_1AS_AUTO) {
    PrintInfo(
      "\"automotive\" was specified as PTP profile. "
      "Currently, (PTP domain | PTP transport type | PTP switch type) "
      "specification is not supported.");
  }

  int is_gptp = (profile == PtpProfile::IEEE_802_1AS_AUTO);
  command = "set_i_config time ptp_automotive " + std::to_string(is_gptp);
  SendCommand(command);

  return Status::OK;
}

Status SeyondHwInterface::CheckAndSetConfig()
{
  Status ret = Status::ERROR_1;
  // Set return mode
  auto return_mode_int = nebula::drivers::IntFromReturnModeSeyond(
    sensor_configuration_->return_mode, sensor_configuration_->sensor_model);
  if (return_mode_int < 0) {
    PrintError(
      "Invalid Return Mode for this sensor. Please check your settings. Falling back to Dual "
      "mode.");
    return_mode_int = nebula::drivers::IntFromReturnModeSeyond(
      ReturnMode::DUAL, sensor_configuration_->sensor_model);
  }
  ret = SetReturnMode(return_mode_int);
  if (ret != Status::OK) {
    return ret;
  }

  // Set PTP mode
  ret = SetPtpMode(sensor_configuration_->ptp_profile);
  if (ret != Status::OK) {
    return ret;
  }

  return Status::OK;
}

}  // namespace drivers
}  // namespace nebula
