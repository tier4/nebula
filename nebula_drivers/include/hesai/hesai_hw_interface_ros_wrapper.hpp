#ifndef NEBULA_HesaiHwInterfaceRosWrapper_H
#define NEBULA_HesaiHwInterfaceRosWrapper_H

#include "common/nebula_common.hpp"
#include "common/nebula_hw_interface_ros_wrapper_base.hpp"
#include "hesai/hesai_common.hpp"
#include "hesai/hesai_hw_interface.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pandar_msgs/msg/pandar_jumbo_packet.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace ros
{
class HesaiHwInterfaceRosWrapper final : public rclcpp::Node, NebulaHwInterfaceWrapperBase
{
  drivers::HesaiHwInterface hw_interface_;
  Status interface_status_;

  drivers::HesaiSensorConfiguration sensor_configuration_;
  drivers::HesaiCloudConfiguration cloud_configuration_;
  drivers::HesaiCalibrationConfiguration calibration_configuration_;

  rclcpp::Publisher<pandar_msgs::msg::PandarScan>::SharedPtr pandar_scan_pub_;

  Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) override;
  void ReceiveScanDataCallback(std::unique_ptr<pandar_msgs::msg::PandarScan> scan_buffer);

public:
  explicit HesaiHwInterfaceRosWrapper(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  Status StreamStart() override;
  Status StreamStop() override;
  Status Shutdown() override;
  Status GetParameters(
    drivers::HesaiSensorConfiguration & sensor_configuration,
    drivers::HesaiCalibrationConfiguration & calibration_configuration,
    drivers::HesaiCloudConfiguration & cloud_configuration);
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_HesaiHwInterfaceRosWrapper_H