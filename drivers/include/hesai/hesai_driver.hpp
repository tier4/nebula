#ifndef NEBULA_DRIVERS_NEBULA_HESAI_H
#define NEBULA_DRIVERS_NEBULA_HESAI_H

#include <iostream>
#include <stdexcept>
#include <string>
#include "common/nebula_common.hpp"
#include "common/nebula_driver_base.hpp"
#include "common/nebula_status.hpp"
#include "hesai/hesai_common.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace drivers
{
class HesaiDriver : NebulaDriverBase
{
private:
public:
  HesaiDriver();
  HesaiDriver(const CalibrationConfigurationBase & calibration_configuration,
                    const CloudConfigurationBase & cloud_configuration);
  Status SetCalibrationConfiguration(
    const CalibrationConfigurationBase & calibration_configuration) override;
  Status SetCloudConfiguration(const CloudConfigurationBase & cloud_configuration) override;

  std::shared_ptr<sensor_msgs::msg::PointCloud2> ParsePacketToPointcloud(
    std::vector<uint8_t> & packet) override;
};

}  // namespace drivers
}  // namespace nebula

#endif  //NEBULA_DRIVERS_NEBULA_HESAI_H
