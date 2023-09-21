#include "nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp"
namespace nebula
{
namespace drivers
{
RobosenseHwInterface::RobosenseHwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  cloud_io_context_info_{new ::drivers::common::IoContext(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  cloud_udp_driver_info{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_info_)},
  scan_cloud_ptr_{std::make_unique<pandar_msgs::msg::PandarScan>()}
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
  current_phase = (data[azimuth_index_ + 1] & 0xff) + ((data[azimuth_index_] & 0xff) << 8);

  //    PrintInfo("Current phase: " + std::to_string(current_phase));  ///////

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
      scan_cloud_ptr_ = std::make_unique<pandar_msgs::msg::PandarScan>();
    }
  }
}

void RobosenseHwInterface::ReceiveInfoPacketCallback(const std::vector<uint8_t> & buffer)
{
  PrintInfo("Info Packet Received");

  if (!is_valid_info_packet_(buffer.size())) {
    PrintDebug("Invalid Packet: " + std::to_string(buffer.size()));
    return;
  }

  info_buffer_.emplace(buffer);
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
    cloud_udp_driver_info->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->gnss_port);
    cloud_udp_driver_info->receiver()->open();
    cloud_udp_driver_info->receiver()->bind();

    cloud_udp_driver_info->receiver()->asyncReceive(
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

Status RobosenseHwInterface::CloudInterfaceStop()
{
  return Status::ERROR_1;
}

Status RobosenseHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  sensor_configuration_ =
    std::static_pointer_cast<RobosenseSensorConfiguration>(sensor_configuration);

  is_valid_packet_ = [](size_t packet_size) { return (packet_size == HELIOS5515_PACKET_SIZE); };
  is_valid_info_packet_ = [](size_t packet_size) {
    return (packet_size == HELIOS5515_INFO_PACKET_SIZE);
  };

  return Status::OK;
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
  const std::function<void(const std::string & received_string)> & string_callback)
{
  std::stringstream calibration;
  calibration << "Laser ID,Elevation,Azimuth\n";

  size_t channel_num = 0;
  size_t vertical_data_offset = 0;
  size_t horizontal_data_offset = 0;

  if (sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_HELIOS_5515) {
    channel_num = 32;
    vertical_data_offset = HELIOS5515_CORRECTED_VERTICAL_ANGLE_OFFSET;
    horizontal_data_offset = HELIOS5515_CORRECTED_HORIZONTAL_ANGLE_OFFSET;
  } else if (sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_BPEARL) {
    channel_num = 32;
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

    calibration << channel << "," << vertical_angle << "," << horizontal_angle << std::endl;
  }

  string_callback(calibration.str());
  return Status::OK;
}

Status RobosenseHwInterface::RegisterScanCallback(
  std::function<void(std::unique_ptr<pandar_msgs::msg::PandarScan>)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
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