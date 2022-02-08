#ifndef NEBULA_VelodyneHwInterfaceRosWrapper_H
#define NEBULA_VelodyneHwInterfaceRosWrapper_H

#include "common/nebula_common.hpp"
#include "common/nebula_hw_interface_ros_wrapper_base.hpp"
#include "velodyne/velodyne_common.hpp"
#include "velodyne/velodyne_hw_interface.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

namespace nebula
{
namespace ros
{
class VelodyneHwInterfaceRosWrapper final : public rclcpp::Node, NebulaHwInterfaceWrapperBase
{
  drivers::VelodyneHwInterface hw_interface_;
  Status interface_status_;

  drivers::VelodyneSensorConfiguration sensor_configuration_;
  drivers::VelodyneCalibrationConfiguration calibration_configuration_;

  rclcpp::Publisher<velodyne_msgs::msg::VelodyneScan>::SharedPtr velodyne_scan_pub_;

  Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) override;
  void ReceiveScanDataCallback(std::unique_ptr<velodyne_msgs::msg::VelodyneScan> scan_buffer);

public:
  explicit VelodyneHwInterfaceRosWrapper(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  Status StreamStart() override;
  Status StreamStop() override;
  Status Shutdown() override;
  Status GetParameters(
    drivers::VelodyneSensorConfiguration & sensor_configuration);
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_VelodyneHwInterfaceRosWrapper_H