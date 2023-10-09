#pragma once

#include "boost_tcp_driver/tcp_driver.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/robosense_info_driver.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp"
#include "nebula_ros/common/nebula_hw_monitor_ros_wrapper_base.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "robosense_msgs/msg/robosense_info_packet.hpp"

#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

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

/// @brief Hardware monitor ros wrapper of robosense driver
class RobosenseHwMonitorRosWrapper final : public rclcpp::Node, NebulaHwMonitorWrapperBase
{
  drivers::RobosenseHwInterface hw_interface_;
  Status interface_status_;
  std::unique_ptr<drivers::RobosenseInfoDriver> info_driver_;
  std::vector<uint8_t> info_packet_buffer_;

  drivers::RobosenseSensorConfiguration sensor_configuration_;
  std::shared_ptr<nebula::drivers::RobosenseCalibrationConfiguration> calibration_configuration_;

  rclcpp::Subscription<robosense_msgs::msg::RobosenseInfoPacket>::SharedPtr robosense_info_sub_;

  /// @brief Initializing hardware monitor ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  Status InitializeHwMonitor(
    const drivers::SensorConfigurationBase & sensor_configuration) override;

public:
  explicit RobosenseHwMonitorRosWrapper(const rclcpp::NodeOptions & options);
  /// @brief Not used
  /// @return Current status
  Status MonitorStart() override;
  /// @brief Not used
  /// @return Status::OK
  Status MonitorStop() override;
  /// @brief Not used
  /// @return Status::OK
  Status Shutdown() override;
  /// @brief Get configurations from ros parameters
  /// @param sensor_configuration Output of SensorConfiguration
  /// @return Resulting status
  Status GetParameters(drivers::RobosenseSensorConfiguration & sensor_configuration);

private:
  diagnostic_updater::Updater diagnostics_updater_;
  /// @brief Initializing diagnostics
  void InitializeRobosenseDiagnostics();
  /// @brief Callback of the timer for getting the current lidar status
  void OnRobosenseStatusTimer();

  /// @brief Check status information from RobosenseLidarStatus for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  /// @brief Callback for receiving DIFOP packet
  /// @param info_msg Received DIFOP packet
  void ReceiveInfoMsgCallback(const robosense_msgs::msg::RobosenseInfoPacket::SharedPtr info_msg);

  /// @brief rclcpp parameter callback
  /// @param parameters Received parameters
  /// @return SetParametersResult
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rclcpp::TimerBase::SharedPtr diagnostics_status_timer_;
  std::map<std::string, std::string> current_sensor_info_;

  std::unique_ptr<rclcpp::Time> current_info_time_;
  uint16_t diag_span_{1000};
  std::optional<std::string> hardware_id_;
};

}  // namespace ros
}  // namespace nebula
