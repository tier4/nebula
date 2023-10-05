#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp"
#include "nebula_ros/common/nebula_hw_interface_ros_wrapper_base.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "robosense_msgs/msg/robosense_packet.hpp"
#include "robosense_msgs/msg/robosense_scan.hpp"

namespace nebula
{
namespace ros
{
/// @brief Get parameter from rclcpp::Parameter
/// @tparam T
/// @param p Parameter from rclcpp parameter callback
/// @param name Target parameter name
/// @param value Corresponding value
/// @return Whether the target name existed
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

/// @brief Hardware interface ros wrapper of Robosense driver
class RobosenseHwInterfaceRosWrapper final : public rclcpp::Node, NebulaHwInterfaceWrapperBase
{
public:
  explicit RobosenseHwInterfaceRosWrapper(const rclcpp::NodeOptions & options);

  /// @brief Start point cloud streaming (Call CloudInterfaceStart of HwInterface)
  /// @return Resulting status
  Status StreamStart() override;

  /// @brief Stop point cloud streaming (not used)
  /// @return Resulting status
  Status StreamStop() override;

  /// @brief Shutdown (not used)
  /// @return Resulting status
  Status Shutdown() override;

  /// @brief Get configurations from ros parameters
  /// @param sensor_configuration Output of SensorConfiguration
  /// @return Resulting status
  Status GetParameters(drivers::RobosenseSensorConfiguration & sensor_configuration);

private:
  drivers::RobosenseHwInterface hw_interface_;
  drivers::RobosenseSensorConfiguration sensor_configuration_;
  Status interface_status_;

  /// @brief Received Robosense Scan message publisher
  rclcpp::Publisher<robosense_msgs::msg::RobosenseScan>::SharedPtr robosense_scan_pub_;

  /// @brief Received Robosense Difop message publisher
  rclcpp::Publisher<robosense_msgs::msg::RobosensePacket>::SharedPtr robosense_difop_pub_;

  /// @brief Initializing hardware interface ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) override;

  /// @brief Callback for receiving RobosenseScan
  /// @param scan_buffer Received RobosenseScan
  void ReceiveScanDataCallback(std::unique_ptr<robosense_msgs::msg::RobosenseScan> scan_buffer);

  /// @brief Callback for receiving RobosensePacket
  /// @param difop_buffer Received DIFOP packet
  void ReceiveInfoDataCallback(std::unique_ptr<robosense_msgs::msg::RobosensePacket> difop_buffer);
};

}  // namespace ros
}  // namespace nebula
