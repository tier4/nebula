#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <array>
#include <memory>

namespace nebula
{
namespace ros
{
class HesaiHwMonitorWrapper
{
public:
  HesaiHwMonitorWrapper(rclcpp::Node* const parent_node,
                        const std::shared_ptr<nebula::drivers::HesaiHwInterface>& hw_interface,
                        std::shared_ptr<nebula::drivers::HesaiSensorConfiguration>& config);

  nebula::Status InitializeHwMonitor();

  void InitializeHesaiDiagnostics();

  std::string GetPtreeValue(boost::property_tree::ptree* pt, const std::string& key);

  std::string GetFixedPrecisionString(double val, int pre);

  void OnHesaiStatusTimer();

  void OnHesaiLidarMonitorTimerHttp();

  void OnHesaiLidarMonitorTimer();

  void HesaiCheckStatus(diagnostic_updater::DiagnosticStatusWrapper& diagnostics);

  void HesaiCheckPtp(diagnostic_updater::DiagnosticStatusWrapper& diagnostics);

  void HesaiCheckTemperature(diagnostic_updater::DiagnosticStatusWrapper& diagnostics);

  void HesaiCheckRpm(diagnostic_updater::DiagnosticStatusWrapper& diagnostics);

  void HesaiCheckVoltageHttp(diagnostic_updater::DiagnosticStatusWrapper& diagnostics);

  void HesaiCheckVoltage(diagnostic_updater::DiagnosticStatusWrapper& diagnostics);

  nebula::Status Status();

private:
  rclcpp::Logger logger_;
  diagnostic_updater::Updater diagnostics_updater_;
  nebula::Status status_;

  const std::shared_ptr<nebula::drivers::HesaiHwInterface> hw_interface_;
  rclcpp::Node* const parent_node_;

  uint16_t diag_span_;
  rclcpp::TimerBase::SharedPtr diagnostics_update_timer_{};
  rclcpp::TimerBase::SharedPtr fetch_diagnostics_timer_{};

  std::unique_ptr<HesaiLidarStatus> current_status_{};
  std::unique_ptr<HesaiLidarMonitor> current_monitor_{};
  std::unique_ptr<HesaiConfig> current_config_{};
  std::unique_ptr<HesaiInventory> current_inventory_{};
  std::unique_ptr<boost::property_tree::ptree> current_lidar_monitor_tree_{};

  std::unique_ptr<rclcpp::Time> current_status_time_{};
  std::unique_ptr<rclcpp::Time> current_config_time_{};
  std::unique_ptr<rclcpp::Time> current_inventory_time_{};
  std::unique_ptr<rclcpp::Time> current_lidar_monitor_time_{};

  uint8_t current_diag_status_;
  uint8_t current_monitor_status_;

  std::mutex mtx_lidar_status_;
  std::mutex mtx_lidar_monitor_;

  std::string info_model_;
  std::string info_serial_;

  std::vector<std::string> temperature_names_;

  bool setup_sensor;
  const std::string MSG_NOT_SUPPORTED = "Not supported";
  const std::string MSG_ERROR = "Error";
  const std::string MSG_SEP = ": ";
};
}  // namespace ros
}  // namespace nebula