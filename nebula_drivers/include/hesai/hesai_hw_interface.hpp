#ifndef NEBULA_HESAI_HW_INTERFACE_H
#define NEBULA_HESAI_HW_INTERFACE_H

#include "common/nebula_hw_interface_base.hpp"
#include "hesai/hesai_common.hpp"
#include "udp_driver/udp_driver.hpp"

namespace nebula
{
namespace drivers
{

class HesaiHwInterface : NebulaHwInterfaceBase
{
private:
  IoContext cloud_io_context_;
  IoContext gnss_io_context_;
  ::drivers::udp_driver::UdpDriver cloud_udp_driver_;
  ::drivers::udp_driver::UdpDriver gnss_udp_driver_;
  std::shared_ptr<HesaiSensorConfiguration> sensor_configuration_;
  std::shared_ptr<HesaiCalibrationConfiguration> calibration_configuration_;

public:
  HesaiHwInterface();
  HesaiHwInterface(std::shared_ptr<SensorConfigurationBase>& sensor_configuration,
                         CalibrationConfigurationBase & calibration_configuration);

  Status ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) final;
  Status CloudInterfaceStart() final;
  Status CloudInterfaceStop() final;
  Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration) final;
  Status GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration) final;
  Status SetSensorConfiguration(std::shared_ptr<SensorConfigurationBase> sensor_configuration) final;

};

}//drivers
}//nebula

#endif  //NEBULA_HESAI_HW_INTERFACE_H
