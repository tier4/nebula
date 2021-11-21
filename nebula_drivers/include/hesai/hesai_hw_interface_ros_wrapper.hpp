#ifndef NEBULA_HesaiHwInterfaceWrapper_H
#define NEBULA_HesaiHwInterfaceWrapper_H

#include "common/nebula_common.hpp"
#include "common/nebula_hw_interface_ros_wrapper_base.hpp"
#include "hesai/hesai_common.hpp"
#include "hesai/hesai_hw_interface.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pandar_msgs/msg/pandar_jumbo_packet.h"
#include "pandar_msgs/msg/pandar_packet.h"

namespace nebula
{
namespace ros
{
class HesaiHwInterfaceWrapper final : public rclcpp::Node, NebulaHwInterfaceWrapperBase
{
  drivers::HesaiHwInterface hw_interface_;

  // ROS
  std::string sensor_model_;
  std::string echo_mode_;
  std::string host_ip_;
  std::string sensor_ip_;
  uint16_t data_port_;
  uint16_t gnss_port_;
  uint16_t frequency_ms_;
  drivers::HesaiSensorConfiguration sensor_configuration_;

  Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) override;
  void ReceiveScanDataCallback(
    std::unique_ptr<std::vector<std::vector<uint8_t>>> scan_buffer) override;

public:
  explicit HesaiHwInterfaceWrapper(
    const rclcpp::NodeOptions & options, const std::string & node_name);
  Status StreamStart() override;
  Status StreamStop() override;
  Status Shutdown() override;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_HesaiHwInterfaceWrapper_H