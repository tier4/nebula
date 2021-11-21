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
  std::unique_ptr<IoContext> cloud_io_context_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::shared_ptr<HesaiSensorConfiguration> sensor_configuration_;
  std::shared_ptr<HesaiCalibrationConfiguration> calibration_configuration_;
  size_t azimuth_index_{};
  size_t mtu_size_;
  std::function<bool(size_t)> is_valid_packet_; /*Lambda Function Array to verify proper packet size*/
  std::function<void (std::unique_ptr<std::vector<std::vector<uint8_t>>> buffer)> scan_reception_callback_;/**This function pointer is called when the scan is complete*/
public:
  HesaiHwInterface();

  void ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) final;
  Status CloudInterfaceStart() final;
  Status CloudInterfaceStop() final;
  Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration) final;
  Status GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration) final;
  Status SetSensorConfiguration(std::shared_ptr<SensorConfigurationBase> sensor_configuration) final;
  Status RegisterScanCallback(std::function<void (std::unique_ptr<std::vector<std::vector<uint8_t>>>)> scan_callback) final;
};

}//drivers
}//nebula

#endif  //NEBULA_HESAI_HW_INTERFACE_H
