#include "nebula_hw_interfaces/nebula_hw_interfaces_seyond/seyond_hw_interface.hpp"
namespace nebula
{
namespace drivers
{
SeyondHwInterface::SeyondHwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  m_owned_ctx{new boost::asio::io_context(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)}
{
}

void SeyondHwInterface::ReceiveSensorPacketCallback(std::vector<uint8_t> & buffer)
{

}

Status SeyondHwInterface::SensorInterfaceStart()
{
  return Status::OK;
}

Status SeyondHwInterface::SensorInterfaceStop()
{
  return Status::ERROR_1;
}

Status SeyondHwInterface::SetSensorConfiguration(
  std::shared_ptr<const SensorConfigurationBase> sensor_configuration)
{
  return Status::OK;
}

Status SeyondHwInterface::GetSensorConfiguration(const SensorConfigurationBase & sensor_configuration)
{
  return Status::ERROR_1;
}

Status SeyondHwInterface::GetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  return Status::ERROR_1;
}

Status SeyondHwInterface::RegisterScanCallback(
  std::function<void(std::vector<uint8_t> &)> scan_callback)
{
  return Status::OK;
}

void SeyondHwInterface::SetTargetModel(int model)
{
  target_model_no = model;
}

void SeyondHwInterface::PrintError(std::string error)
{

}

void SeyondHwInterface::PrintDebug(std::string debug)
{

}

void SeyondHwInterface::PrintInfo(std::string info)
{

}

void SeyondHwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger_ = logger;
}

}  // namespace drivers
}  // namespace nebula
