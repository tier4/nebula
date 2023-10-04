#include "nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp"
namespace nebula
{
namespace drivers
{
RobosenseHwInterface::RobosenseHwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  info_io_context_{new ::drivers::common::IoContext(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  info_udp_driver_{new ::drivers::udp_driver::UdpDriver(*info_io_context_)},
  scan_cloud_ptr_{std::make_unique<robosense_msgs::msg::RobosenseScan>()}
{
}

void RobosenseHwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer)
{
  int scan_phase = static_cast<int>(sensor_configuration_->scan_phase * 100.0);
  if (!is_valid_packet_(buffer.size())) {
    PrintDebug("Invalid Packet: " + std::to_string(buffer.size()));
    return;
  }
  uint32_t buffer_size = buffer.size();
  std::array<uint8_t, MTU_SIZE> packet_data{};
  std::copy_n(std::make_move_iterator(buffer.begin()), buffer_size, packet_data.begin());
  robosense_msgs::msg::MsopPacket msop_packet;
  msop_packet.data = packet_data;

  const auto now = std::chrono::system_clock::now();
  const auto timestamp_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  constexpr int nanosec_per_sec = 1000000000;
  msop_packet.stamp.sec = static_cast<int>(timestamp_ns / nanosec_per_sec);
  msop_packet.stamp.nanosec = static_cast<int>(timestamp_ns % nanosec_per_sec);

  scan_cloud_ptr_->packets.emplace_back(msop_packet);

  int current_phase{};
  bool comp_flg = false;

  const auto & data = scan_cloud_ptr_->packets.back().data;
  current_phase = (data[azimuth_index_ + 1] & 0xff) + ((data[azimuth_index_] & 0xff) << 8);

  current_phase = (static_cast<int>(current_phase) + 36000 - scan_phase) % 36000;

  if (current_phase >= prev_phase_ || scan_cloud_ptr_->packets.size() < 2) {
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

void RobosenseHwInterface::ReceiveInfoPacketCallback(const std::vector<uint8_t> & buffer)
{
  if (!is_valid_info_packet_(buffer.size())) {
    PrintDebug("Invalid Packet: " + std::to_string(buffer.size()));
    return;
  }

  info_buffer_.emplace(buffer);  //////
  is_info_received = true;       ////////

  if (info_reception_callback_) {
    std::unique_ptr<robosense_msgs::msg::DifopPacket> difop_packet =
      std::make_unique<robosense_msgs::msg::DifopPacket>();
    std::copy_n(std::make_move_iterator(buffer.begin()), buffer.size(), difop_packet->data.begin());

    info_reception_callback_(std::move(difop_packet));
  }
}

Status RobosenseHwInterface::WaitForSensorInfo(const std::chrono::milliseconds & timeout) const
{
  const auto start = std::chrono::system_clock::now();
  while (rclcpp::ok() && !is_info_received) {
    const auto now = std::chrono::system_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
    if (elapsed > timeout) {
      return Status::ERROR_1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return Status::OK;
}

Status RobosenseHwInterface::CloudInterfaceStart()
{
  try {
    std::cout << "Starting UDP server for data packets on: " << *sensor_configuration_ << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();

    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&RobosenseHwInterface::ReceiveCloudPacketCallback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status RobosenseHwInterface::InfoInterfaceStart()
{
  try {
    std::cout << "Starting UDP server for info packets on: " << *sensor_configuration_ << std::endl;
    PrintInfo(
      "Starting UDP server for info packets on: " + sensor_configuration_->sensor_ip + ":" +
      std::to_string(sensor_configuration_->gnss_port));
    info_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->gnss_port);
    info_udp_driver_->receiver()->open();
    info_udp_driver_->receiver()->bind();

    info_udp_driver_->receiver()->asyncReceive(
      std::bind(&RobosenseHwInterface::ReceiveInfoPacketCallback, this, std::placeholders::_1));

  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->gnss_port << std::endl;
    return status;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  return Status::OK;
}

Status RobosenseHwInterface::InfoInterfaceStop()
{
  info_udp_driver_->receiver()->close();
  return Status::OK;
}

Status RobosenseHwInterface::CloudInterfaceStop()
{
  return Status::ERROR_1;
}

Status RobosenseHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  Status status = Status::OK;

  try {
    sensor_configuration_ =
      std::static_pointer_cast<RobosenseSensorConfiguration>(sensor_configuration);

    if (sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_BPEARL) {
      azimuth_index_ = 44;
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == BPEARL_PACKET_SIZE); };
      is_valid_info_packet_ = [](size_t packet_size) {
        return (packet_size == BPEARL_INFO_PACKET_SIZE);
      };
    } else if (sensor_configuration->sensor_model == SensorModel::ROBOSENSE_HELIOS_5515) {
      azimuth_index_ = 44;
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == HELIOS5515_PACKET_SIZE); };
      is_valid_info_packet_ = [](size_t packet_size) {
        return (packet_size == HELIOS5515_INFO_PACKET_SIZE);
      };
    } else {
      status = Status::INVALID_SENSOR_MODEL;
    }
  } catch (const std::exception & ex) {
    status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << status << std::endl;
    return status;
  }
  return status;
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

Status RobosenseHwInterface::GetLidarCalibrationFromSensor(
  const std::function<void(
    const std::string & calibration_received, const ReturnMode & return_mode_received)> & callback)
{
  if (!info_buffer_.has_value()) {
    PrintInfo("Info packet is not received yet.");
    return Status::ERROR_1;
  }

  std::stringstream calibration;
  calibration << "Laser ID,Elevation,Azimuth\n";

  size_t channel_num = GetChannelSize(sensor_configuration_->sensor_model);
  size_t vertical_data_offset{};
  size_t horizontal_data_offset{};
  ReturnMode return_mode = ReturnMode::UNKNOWN;

  if (sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS_5515) {
    vertical_data_offset = HELIOS5515_CORRECTED_VERTICAL_ANGLE_OFFSET;
    horizontal_data_offset = HELIOS5515_CORRECTED_HORIZONTAL_ANGLE_OFFSET;

    uint8_t return_mode_data = info_buffer_.value()[HELIOS5515_RETURN_MODE_OFFSET];
    if (return_mode_data == 0x00) {
      return_mode = ReturnMode::DUAL;
    } else {
      return_mode = ReturnMode::SINGLE;
    }

  } else if (sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_BPEARL) {
    vertical_data_offset = BPEARL_CORRECTED_VERTICAL_ANGLE_OFFSET;
    horizontal_data_offset = BPEARL_CORRECTED_HORIZONTAL_ANGLE_OFFSET;

    uint8_t return_mode_data = info_buffer_.value()[BPEARL_RETURN_MODE_OFFSET];
    if (return_mode_data == 0x00) {
      return_mode = ReturnMode::DUAL;
    } else {
      return_mode = ReturnMode::SINGLE;
    }

  } else {
    return Status::INVALID_SENSOR_MODEL;
  }

  for (size_t channel = 0; channel < channel_num; ++channel) {
    const auto vertical_offset = (channel * 3) + vertical_data_offset;
    const auto horizontal_offset = (channel * 3) + horizontal_data_offset;

    auto & info_buffer = info_buffer_.value();

    uint16_t vertical_angle_data =
      (info_buffer[vertical_offset + 2] & 0xff) + ((info_buffer[vertical_offset + 1] & 0xff) << 8);
    double vertical_angle = static_cast<double>(vertical_angle_data) / 100.0;
    if (info_buffer[vertical_offset] == 0x01) vertical_angle = -vertical_angle;

    uint16_t horizontal_angle_data = (info_buffer[horizontal_offset + 2] & 0xff) +
                                     ((info_buffer[horizontal_offset + 1] & 0xff) << 8);
    double horizontal_angle = static_cast<double>(horizontal_angle_data) / 100.0;
    if (info_buffer[horizontal_offset] == 0x01) horizontal_angle = -horizontal_angle;

    calibration << channel + 1 << "," << vertical_angle << "," << horizontal_angle << std::endl;
  }
  callback(calibration.str(), return_mode);
  return Status::OK;
}

std::vector<uint8_t> RobosenseHwInterface::GetInfoPacketFromSensor()
{
  return *info_buffer_;
}

Status RobosenseHwInterface::RegisterScanCallback(
  std::function<void(std::unique_ptr<robosense_msgs::msg::RobosenseScan>)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
  return Status::OK;
}

Status RobosenseHwInterface::RegisterInfoCallback(
  std::function<void(std::unique_ptr<robosense_msgs::msg::DifopPacket>)> info_callback)
{
  info_reception_callback_ = std::move(info_callback);
  return Status::OK;
}

void RobosenseHwInterface::PrintDebug(std::string debug)
{
  if (parent_node_logger_) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger_), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

void RobosenseHwInterface::PrintInfo(std::string info)
{
  if (parent_node_logger_) {
    RCLCPP_INFO_STREAM((*parent_node_logger_), info);
  } else {
    std::cout << info << std::endl;
  }
}

void RobosenseHwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger_ = logger;
}

}  // namespace drivers
}  // namespace nebula