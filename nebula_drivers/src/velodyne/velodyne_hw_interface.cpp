#include "velodyne/velodyne_hw_interface.hpp"

#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <memory>

namespace nebula
{
namespace drivers
{
VelodyneHwInterface::VelodyneHwInterface()
: cloud_io_context_{new IoContext(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  scan_cloud_ptr_{std::make_unique<velodyne_msgs::msg::VelodyneScan>()}
{
}

Status VelodyneHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  sensor_configuration_ = std::static_pointer_cast<VelodyneSensorConfiguration>(sensor_configuration);
  phase_ = (uint16_t)round(sensor_configuration_->scan_phase*100);
  Status status = Status::OK;
  return status;
}

Status VelodyneHwInterface::CloudInterfaceStart()
{
  try {
//    std::cout << "Starting UDP server on: " << sensor_configuration_->host_ip << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();
    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&VelodyneHwInterface::ReceiveCloudPacketCallback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status VelodyneHwInterface::RegisterScanCallback(
  std::function<void(std::unique_ptr<velodyne_msgs::msg::VelodyneScan>)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
  return Status::OK;
}

void VelodyneHwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer)
{
  // Process current packet
  uint32_t buffer_size = buffer.size();
  std::array<uint8_t, 1206> packet_data{};
  std::copy_n(std::make_move_iterator(buffer.begin()), buffer_size, packet_data.begin());
  velodyne_msgs::msg::VelodynePacket velodyne_packet;
  auto now = std::chrono::system_clock::now();
  auto now_secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  auto now_nanosecs = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  velodyne_packet.data = packet_data;
  velodyne_packet.stamp.sec = static_cast<int>(now_secs);
  velodyne_packet.stamp.nanosec = static_cast<int>((now_nanosecs/1000000000. - static_cast<double>(now_secs))*1000000000);
  scan_cloud_ptr_->packets.emplace_back(velodyne_packet);
  processed_packets_++;

  // Check if scan is complete
  packet_first_azm_  = scan_cloud_ptr_->packets.back().data[2]; // lower word of azimuth block 0
  packet_first_azm_ |= scan_cloud_ptr_->packets.back().data[3] << 8; // higher word of azimuth block 0

  packet_last_azm_ = scan_cloud_ptr_->packets.back().data[1102];
  packet_last_azm_ |= scan_cloud_ptr_->packets.back().data[1103] << 8;

  packet_first_azm_phased_ = (36000 + packet_first_azm_ - phase_) % 36000;
  packet_last_azm_phased_ = (36000 + packet_last_azm_ - phase_) % 36000;
//  std::cout << "first:" << packet_first_azm_phased_ << std::endl;
//  std::cout << "last" << packet_last_azm_phased_ << std::endl;

  if (processed_packets_ > 1) {
    if (
      packet_last_azm_phased_ < packet_first_azm_phased_ ||
      packet_first_azm_phased_ < prev_packet_first_azm_phased_) {
        // Callback
        scan_reception_callback_(std::move(scan_cloud_ptr_));
        scan_cloud_ptr_ = std::make_unique<velodyne_msgs::msg::VelodyneScan>();
        processed_packets_ = 0;
    }
  }
  prev_packet_first_azm_phased_ = packet_first_azm_phased_;
}
Status VelodyneHwInterface::CloudInterfaceStop() { return Status::ERROR_1; }

Status VelodyneHwInterface::GetSensorConfiguration(SensorConfigurationBase & sensor_configuration)
{
  return Status::ERROR_1;
}

Status VelodyneHwInterface::GetCalibrationConfiguration(
  CalibrationConfigurationBase & calibration_configuration)
{
  return Status::ERROR_1;
}

}  // namespace drivers
}  // namespace nebula
