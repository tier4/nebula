#pragma once

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <boost_tcp_driver/tcp_driver.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/robosense/robosense_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

#include <mutex>
#include <thread>

namespace nebula
{
namespace ros
{

/// @brief Hardware monitor ros wrapper of robosense driver
class RobosenseHwMonitorWrapper
{
public:
  explicit RobosenseHwMonitorWrapper(rclcpp::Node* const parent_node,
                                     std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration>& config);
  
  void OnConfigChange(const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & new_config);

  /// @brief Callback for receiving DIFOP packet
  /// @param info_msg Received DIFOP packet
  void DiagnosticsCallback(const std::map<std::string, std::string> & diag_info);

private:
  /// @brief Initializing diagnostics
  void InitializeRobosenseDiagnostics();
  /// @brief Callback of the timer for getting the current lidar status
  void OnRobosenseStatusTimer();

  /// @brief Check status information from RobosenseLidarStatus for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  rclcpp::Node * parent_;
  rclcpp::Logger logger_;
  std::optional<diagnostic_updater::Updater> diagnostics_updater_;
  nebula::Status status_;

  std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> sensor_cfg_ptr_;
  std::mutex mtx_config_;

  rclcpp::TimerBase::SharedPtr diagnostics_status_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_update_timer_;
  std::map<std::string, std::string> current_sensor_info_;
  std::mutex mtx_current_sensor_info_;

  rclcpp::Time current_info_time_;
  uint16_t diag_span_{1000};
};

}  // namespace ros
}  // namespace nebula
