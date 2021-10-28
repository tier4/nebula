#ifndef NEBULA_DRIVERS_NEBULA_HESAI_H
#define NEBULA_DRIVERS_NEBULA_HESAI_H

#include "common/nebula_hw_interface_base.hpp"
#include "udp_driver/udp_driver.hpp"

namespace nebula
{
namespace drivers
{

class NebulaHesaiHwInterface : NebulaHwInterfaceBase
{
private:
  IoContext io_context_;
  ::drivers::udp_driver::UdpDriver cloud_udp_driver_;

public:
  NebulaHesaiHwInterface();
  NebulaHesaiHwInterface(SensorConfigurationBase & sensor_configuration,
                         CalibrationConfigurationBase & calibration_configuration);

  Status CloudInterfaceStart(const SensorConfigurationBase &sensor_configuration) override;

};

}
}

#endif  //NEBULA_DRIVERS_NEBULA_HESAI_H
