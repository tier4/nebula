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

HesaiHwInterface::HesaiHwInterface(
  std::shared_ptr<SensorConfigurationBase> & sensor_configuration,
  CalibrationConfigurationBase & calibration_configuration)
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
    return Status::OK;
  } catch (const std::exception & ex) {
    Status status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << NebulaStatusToString(status) << std::endl;
    return status;
  }
}

Status HesaiHwInterface::CloudInterfaceStart()
{
  try {
    std::cout << "Starting UDP server on: " << sensor_configuration_->sensor_ip << "."
              << sensor_configuration_->data_port << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->sensor_ip, sensor_configuration_->data_port);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();
    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&HesaiHwInterface::ReceiveCloudPacketCallback, this, std::placeholders::_1));
    std::cout << "After callback" << std::endl;
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << NebulaStatusToString(status) << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
  }
  return Status::OK;
}

void HesaiHwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer)
{
  std::cout << "Got a Packet. Form the scan here" << std::endl;
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
