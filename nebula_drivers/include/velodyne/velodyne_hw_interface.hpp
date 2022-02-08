#ifndef NEBULA_VELODYNE_HW_INTERFACE_H
#define NEBULA_VELODYNE_HW_INTERFACE_H

#include "common/nebula_hw_interface_base.hpp"
#include "velodyne/velodyne_common.hpp"
#include "udp_driver/udp_driver.hpp"

#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

namespace nebula
{
namespace drivers
{
class VelodyneHwInterface : NebulaHwInterfaceBase
{
private:
  std::unique_ptr<IoContext> cloud_io_context_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::shared_ptr<VelodyneSensorConfiguration> sensor_configuration_;
  std::shared_ptr<VelodyneCalibrationConfiguration> calibration_configuration_;
  size_t azimuth_index_{};
  size_t mtu_size_{};
  std::unique_ptr<velodyne_msgs::msg::VelodyneScan> scan_cloud_ptr_;
  std::function<bool(size_t)>
    is_valid_packet_; /*Lambda Function Array to verify proper packet size*/
  std::function<void(std::unique_ptr<velodyne_msgs::msg::VelodyneScan> buffer)>
    scan_reception_callback_; /**This function pointer is called when the scan is complete*/

  uint16_t packet_first_azm_ = 0;
  uint16_t packet_first_azm_phased_ = 0;
  uint16_t packet_last_azm_ = 0;
  uint16_t packet_last_azm_phased_ = 0;
  uint16_t prev_packet_first_azm_phased_ = 0;
  uint16_t phase_ = 0;
  uint processed_packets_ = 0;

public:
  VelodyneHwInterface();

  void ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) final;
  Status CloudInterfaceStart() final;
  Status CloudInterfaceStop() final;
  Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration) final;
  Status GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration) final;
  Status SetSensorConfiguration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration) final;
  Status RegisterScanCallback(
    std::function<void(std::unique_ptr<velodyne_msgs::msg::VelodyneScan>)> scan_callback);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_VELODYNE_HW_INTERFACE_H
