#ifndef NEBULA_InnovusionHwMonitorRosWrapper_H
#define NEBULA_InnovusionHwMonitorRosWrapper_H

#include "nebula_common/innovusion/innovusion_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_innovusion/innovusion_hw_interface.hpp"
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

/// @brief Hardware monitor ros wrapper of Innovusion driver
class InnovusionHwMonitorRosWrapper final : public rclcpp::Node, NebulaHwMonitorWrapperBase
{
private:
  drivers::InnovusionHwInterface hw_interface_;
  diagnostic_updater::Updater diagnostics_updater_;
  Status interface_status_;
  drivers::InnovusionSensorConfiguration sensor_configuration_;
  std::shared_ptr<boost::property_tree::ptree> current_snapshot_;
  std::shared_ptr<rclcpp::Time> current_snapshot_time_;
  rclcpp::TimerBase::SharedPtr diagnostics_snapshot_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_update_timer_;
  uint16_t diag_span_;
  rclcpp::CallbackGroup::SharedPtr cbg_r_;
  rclcpp::CallbackGroup::SharedPtr cbg_m_;
  uint8_t current_diag_status_;

  std::string key_lidar_snapshot_;
  std::string key_lidar_rpm_;
  std::string key_laser_voltage_;
  std::string key_lidar_up_time_;
  std::string key_det_temp_;
  std::string key_laser_temp_;

  /// @brief Initializing hardware monitor ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  Status InitializeHwMonitor(
    const drivers::SensorConfigurationBase & sensor_configuration) override;

  /// @brief Check the current motor rpm for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void InnovusionCheckMotorRpm(
      diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the laser voltage for diagnostic_updater
  /// @param diagnostics 
  void InnovusionCheckLaserVoltage(
      diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the uptime for diagnostic_updater
  void InnovusionCheckUptime(
      diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the det temperature for diagnostic_updater
  void InnovusionCheckDetTemp(
      diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the laser temperature for diagnostic_updater
  void InnovusionCheckLaserTemp(
      diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Get value from property_tree
  /// @param pt property_tree
  /// @param key Pey string
  /// @return Value
  std::tuple<bool, uint8_t, std::string, std::string> InnovusionCommonGet(
    std::shared_ptr<boost::property_tree::ptree> pt,std::string key);
  /// @brief Get value from property_tree
  /// @param pt property_tree
  /// @param key Pey string
  /// @return Value
  std::string GetPtreeValue(
       std::shared_ptr<boost::property_tree::ptree> pt, const std::string & key);

public:
  explicit InnovusionHwMonitorRosWrapper(const rclcpp::NodeOptions & options);
  ~InnovusionHwMonitorRosWrapper() noexcept override;
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
  Status GetParameters(drivers::InnovusionSensorConfiguration & sensor_configuration);
  /// @brief Initializing diagnostics
  void InitializeInnovusionDiagnostics();
  /// @brief Callback of the timer for getting the current lidar snapshot
  void OnInnovusionSnapshotTimer();
  /// @brief Get the key of the lidar snapshot
  const std::string & GetKeyLidarSnapshot() const { return key_lidar_snapshot_; }
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_InnovusionHwMonitorRosWrapper_H
