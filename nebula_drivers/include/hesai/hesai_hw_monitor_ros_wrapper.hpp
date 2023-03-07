#ifndef NEBULA_HesaiHwMonitorRosWrapper_H
#define NEBULA_HesaiHwMonitorRosWrapper_H

#include "common/nebula_common.hpp"
#include "common/nebula_hw_monitor_ros_wrapper_base.hpp"
#include "hesai/hesai_common.hpp"
#include "hesai/hesai_hw_interface.hpp"
#include "tcp_driver/tcp_driver.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <boost/asio.hpp>

#include <mutex>

namespace nebula
{
namespace ros
{
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

class HesaiHwMonitorRosWrapper final : public rclcpp::Node, NebulaHwMonitorWrapperBase
{
  drivers::HesaiHwInterface hw_interface_;
  Status interface_status_;

  drivers::HesaiSensorConfiguration sensor_configuration_;

  Status InitializeHwMonitor(
    const drivers::SensorConfigurationBase & sensor_configuration) override;

public:
  explicit HesaiHwMonitorRosWrapper(const rclcpp::NodeOptions & options);

  Status MonitorStart() override;
  Status MonitorStop() override;
  Status Shutdown() override;
  Status GetParameters(drivers::HesaiSensorConfiguration & sensor_configuration);

private:
  diagnostic_updater::Updater diagnostics_updater_;
  void InitializeHesaiDiagnostics();
  void OnHesaiStatusTimer();
  void OnHesaiLidarMonitorTimerHttp();
  void OnHesaiLidarMonitorTimer();
  //  void OnHesaiDiagnosticsTimer();
  //  void OnHesaiStatusTimer();
  void HesaiCheckStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void HesaiCheckPtp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void HesaiCheckTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void HesaiCheckRpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void HesaiCheckVoltageHttp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
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
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  std::string GetPtreeValue(boost::property_tree::ptree * pt, const std::string & key);
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
