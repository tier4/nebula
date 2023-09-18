#include "nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp"
namespace nebula
{
namespace drivers
{
RobosenseHwInterface::RobosenseHwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
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

Status RobosenseHwInterface::CloudInterfaceStart()
{
  try {
    std::cout << "Starting UDP server on: " << *sensor_configuration_ << std::endl;
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