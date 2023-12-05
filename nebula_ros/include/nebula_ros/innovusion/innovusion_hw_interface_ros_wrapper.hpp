#ifndef NEBULA_InnovusionHwInterfaceRosWrapper_H
#define NEBULA_InnovusionHwInterfaceRosWrapper_H

#include "nebula_common/innovusion/innovusion_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_innovusion/innovusion_hw_interface.hpp"
#include "nebula_ros/common/nebula_hw_interface_ros_wrapper_base.hpp"
#include "boost_tcp_driver/tcp_driver.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "innovusion_msgs/msg/innovusion_packet.hpp"
#include "innovusion_msgs/msg/innovusion_scan.hpp"

#include <boost/asio.hpp>

#include <mutex>
#include <thread>

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

/// @brief Hardware interface ros wrapper of Innovusion driver
class InnovusionHwInterfaceRosWrapper final : public rclcpp::Node, NebulaHwInterfaceWrapperBase
{
  drivers::InnovusionHwInterface hw_interface_;
  Status interface_status_;

  drivers::InnovusionSensorConfiguration sensor_configuration_;

  /// @brief Received Innovusion message publisher
  rclcpp::Publisher<innovusion_msgs::msg::InnovusionScan>::SharedPtr innovusion_scan_pub_;

  /// @brief Initializing hardware interface ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) override;
  /// @brief Callback for receiving InnovusionScan
  /// @param scan_buffer Received InnovusionScan
  void ReceiveScanDataCallback(std::unique_ptr<innovusion_msgs::msg::InnovusionScan> scan_buffer);

public:
  explicit InnovusionHwInterfaceRosWrapper(const rclcpp::NodeOptions & options);
  ~InnovusionHwInterfaceRosWrapper() noexcept override;
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
  Status GetParameters(drivers::InnovusionSensorConfiguration & sensor_configuration);

private:
  std::mutex mtx_config_;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_InnovusionHwInterfaceRosWrapper_H
