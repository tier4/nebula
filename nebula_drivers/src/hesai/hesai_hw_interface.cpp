#include "hesai/hesai_hw_interface.hpp"

namespace nebula
{
namespace drivers
{
HesaiHwInterface::HesaiHwInterface()
: cloud_io_context_{new IoContext(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)}
{
}

Status HesaiHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  try {
    sensor_configuration_ =
      std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration);
  } catch (const std::exception & ex) {
    Status status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << NebulaStatusToString(status) << std::endl;
    return status;
  }
  return Status::OK;
}

Status HesaiHwInterface::CloudInterfaceStart()
{
  try {
    std::cout << "Starting UDP server on: " << sensor_configuration_->host_ip << "."
              << sensor_configuration_->data_port << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();
    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&HesaiHwInterface::ReceiveCloudPacketCallback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << NebulaStatusToString(status) << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
    }
  return Status::OK;
}

void HesaiHwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer)
{
  std::cout << "New packet of size:" << buffer.size() << std::endl;
  size_t packet_size = buffer.size();
  if (packet_size == 1194 || packet_size == 1198) {     // todo: this is for pandar, implement universal _is_valid function
    cur_pkt_ = buffer;
    // send one level higher and publish
  }

}
Status HesaiHwInterface::CloudInterfaceStop()
{
  return Status::ERROR_1;
}

Status HesaiHwInterface::GetSensorConfiguration(SensorConfigurationBase & sensor_configuration)
{
  return Status::ERROR_1;
}

Status HesaiHwInterface::GetCalibrationConfiguration(
  CalibrationConfigurationBase & calibration_configuration)
{
  return Status::ERROR_1;
}

}  // namespace drivers
}  // namespace nebula
