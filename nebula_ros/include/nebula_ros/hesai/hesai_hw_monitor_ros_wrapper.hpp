#ifndef NEBULA_HesaiHwMonitorRosWrapper_H
#define NEBULA_HesaiHwMonitorRosWrapper_H

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp"
#include "nebula_ros/common/nebula_hw_monitor_ros_wrapper_base.hpp"
#include "boost_tcp_driver/tcp_driver.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <boost/asio.hpp>

#include <mutex>
#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

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

/// @brief Hardware monitor ros wrapper of hesai driver
class HesaiHwMonitorRosWrapper final : public rclcpp::Node, NebulaHwMonitorWrapperBase
{
  drivers::HesaiHwInterface hw_interface_;
  Status interface_status_;

  drivers::HesaiSensorConfiguration sensor_configuration_;

  /// @brief Initializing hardware monitor ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  Status InitializeHwMonitor(
    const drivers::SensorConfigurationBase & sensor_configuration) override;

public:
  explicit HesaiHwMonitorRosWrapper(const rclcpp::NodeOptions & options);
  ~HesaiHwMonitorRosWrapper() noexcept override;
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
  Status GetParameters(drivers::HesaiSensorConfiguration & sensor_configuration);

private:
  diagnostic_updater::Updater diagnostics_updater_;
  /// @brief Initializing diagnostics
  void InitializeHesaiDiagnostics();
  /// @brief Callback of the timer for getting the current lidar status
  void OnHesaiStatusTimer();
  /// @brief Callback of the timer for getting the current lidar monitor via http
  void OnHesaiLidarMonitorTimerHttp();
  /// @brief Callback of the timer for getting the current lidar monitor via tcp
  void OnHesaiLidarMonitorTimer();
  //  void OnHesaiDiagnosticsTimer();
  //  void OnHesaiStatusTimer();

  /// @brief Check status information from HesaiLidarStatus for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check ptp information from HesaiLidarStatus for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckPtp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check temperature information from HesaiLidarStatus for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check rpm information from HesaiLidarStatus for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckRpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check voltage information from HesaiLidarStatus for diagnostic_updater via http
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckVoltageHttp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check voltage information from HesaiLidarStatus for diagnostic_updater via tcp
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckVoltage(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  rclcpp::TimerBase::SharedPtr diagnostics_update_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_update_monitor_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_status_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_lidar_monitor_timer_;
  std::unique_ptr<HesaiLidarStatus> current_status;
  std::unique_ptr<HesaiLidarMonitor> current_monitor;
  std::unique_ptr<HesaiConfig> current_config;
  std::unique_ptr<HesaiInventory> current_inventory;
  std::unique_ptr<boost::property_tree::ptree> current_lidar_monitor_tree;
  std::unique_ptr<rclcpp::Time> current_status_time;
  std::unique_ptr<rclcpp::Time> current_config_time;
  std::unique_ptr<rclcpp::Time> current_inventory_time;
  std::unique_ptr<rclcpp::Time> current_lidar_monitor_time;
  uint8_t current_diag_status;
  uint8_t current_monitor_status;

  uint16_t diag_span_;
  std::mutex mtx_diag;
  std::mutex mtx_status;
  std::mutex mtx_lidar_monitor;
  //  std::timed_mutex mtx_lidar_monitor;
  std::mutex mtx_config_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  /// @brief rclcpp parameter callback
  /// @param parameters Received parameters
  /// @return SetParametersResult
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  /// @brief Get value from property_tree
  /// @param pt property_tree
  /// @param key Pey string
  /// @return Value
  std::string GetPtreeValue(boost::property_tree::ptree * pt, const std::string & key);
  /// @brief Making fixed precision string
  /// @param val Target value
  /// @param pre Precision
  /// @return Created string
  std::string GetFixedPrecisionString(double val, int pre = 2);

  std::string info_model;
  std::string info_serial;
  rclcpp::CallbackGroup::SharedPtr cbg_r_;
  rclcpp::CallbackGroup::SharedPtr cbg_m_;
  rclcpp::CallbackGroup::SharedPtr cbg_m2_;

  const char * not_supported_message;
  const char * error_message;
  std::string message_sep;

  std::vector<std::string> temperature_names;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_HesaiHwMonitorRosWrapper_H
