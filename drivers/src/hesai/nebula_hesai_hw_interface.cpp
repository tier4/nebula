#include "hesai/nebula_hesai_hw_interface.hpp"

namespace nebula
{
namespace drivers
{
NebulaHesaiHwInterface::NebulaHesaiHwInterface() : io_context_(), cloud_udp_driver_(io_context_) {}

NebulaHesaiHwInterface::NebulaHesaiHwInterface(
  SensorConfigurationBase & sensor_configuration,
  CalibrationConfigurationBase & calibration_configuration)
: NebulaHwInterfaceBase(sensor_configuration, calibration_configuration),
  io_context_(),
  cloud_udp_driver_(io_context_)
{
  CloudInterfaceStart(sensor_configuration_);
}
Status NebulaHesaiHwInterface::CloudInterfaceStart(
  const SensorConfigurationBase & sensor_configuration)
{
  return Status::ERROR_1;
}

}  // namespace drivers
}  // namespace nebula
