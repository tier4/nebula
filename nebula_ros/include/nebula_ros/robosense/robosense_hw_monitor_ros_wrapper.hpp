#ifndef NEBULA_RobosenseHwMonitorRosWrapper_H
#define NEBULA_RobosenseHwMonitorRosWrapper_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp"
#include "nebula_ros/common/nebula_hw_monitor_ros_wrapper_base.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "robosense_msgs/msg/robosense_bp_ruby_monitor_info.hpp"
#include "robosense_msgs/msg/robosense_flash_monitor_info.hpp"
#include "robosense_msgs/msg/robosense_helios_monitor_info.hpp"
#include "robosense_msgs/msg/robosense_mems_monitor_info.hpp"

#include <mutex>

namespace nebula
{
namespace ros
{
//@brief uint8_t to string in hex
std::string to_hex(const uint8_t * values, size_t val_len)
{
  std::string code_str;
  for (unsigned int i = 0; i < val_len; i++) {
    char s1 = char(values[i] >> 4);
    char s2 = char(values[i] & 0xf);
    s1 > 9 ? s1 += 87 : s1 += 48;
    s2 > 9 ? s2 += 87 : s2 += 48;
    code_str.append(1, s1);
    code_str.append(1, s2);
  }
  return code_str;
}
/// @brief Hardware monitor ros wrapper of robosense driver
class RobosenseHwMonitorRosWrapper final : public rclcpp::Node, NebulaHwMonitorWrapperBase
{
  drivers::RobosenseHwInterface hw_interface_;
  Status interface_status_;

  drivers::RobosenseSensorConfiguration sensor_configuration_;

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

private:  // ROS Diagnostics
  diagnostic_updater::Updater diagnostics_updater_;
  /// @brief Initializing diagnostics
  void InitializeRobosenseDiagnostics();

  /// @brief Getting motor:rpm from the current property_tree
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetMotorRpm();
  /// @brief Getting  bot_fpga_temperature from the current RobosenseHeliosMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetBotFpgaTemperature();
  /// @brief Getting  recv_A_temperature from the current RobosenseHeliosMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetRecvATemperature();
  /// @brief Getting  recv_B_temperature from the current RobosenseHeliosMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetRecvBTemperature();
  /// @brief Getting  main_fpga_temperature from the current RobosenseHeliosMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetMainFpgaTemperature();
  /// @brief Getting  main_fpga_core_temperature from the current RobosenseHeliosMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetMainCoreFpgaTemperature();
  /// @brief Getting  apd_temperature from the current RobosenseBpRubyMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetApdTemperature();
  /// @brief Getting  main_bot_temperature from the current RobosenseBpRubyMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetMainBotTemperature();
  /// @brief Getting  battery_volt from the current RobosenseM1PlusMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetBatteryVolt();
  /// @brief Getting lane up from the current RobosenseHeliosMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetLaneUp();
  /// @brief Getting lane up cnt from the current RobosenseHeliosMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetLaneUpCnt();
  /// @brief Getting gps status from the current RobosenseHeliosMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetMainStatus();
  /// @brief Getting gps status from the current RobosenseHeliosMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetGpsStatus();
  /// @brief Getting lidar fault status from the current RobosenseM1PlusMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetLidarFaultStatus();
  /// @brief Getting lidar roi status from the current RobosenseM1PlusMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetLidarRoiStatus();
  /// @brief Getting lidar pl_vmon_vin_p from the current RobosenseFlashMonitorInfo msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetPlVmonVinP();
  /// @brief Getting lidar fpga_pl_kernal_temperature from the current RobosenseFlashMonitorInfo
  /// msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetFpgaPlKernalTemperature();
  /// @brief Getting lidar fpga_ps_kernal_temperature from the current RobosenseFlashMonitorInfo
  /// msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetFpgaPsKernalTemperature();
  /// @brief Getting lidar window_temperature from the current RobosenseFlashMonitorInfo
  /// msg
  /// @return tuple<Got execption, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> RobosenseGetWindowTemperature();
  /// @brief Get SensorModel description
  /// @param sensor_model SensorModel
  std::string GetInfoModel(nebula::drivers::SensorModel sensor_model);

  /// @brief Check the current rpm for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckRpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current bot_fpga_temperature for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckBotFpgaTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current recv_A_temperature for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckRecvATemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current recv_B_temperature for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckRecvBTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current main_fpga_temperature for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckMainFpgaTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current main_fpga_core_temperature for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckMainFpgaCoreTemperature(
    diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current apd_temperature for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckApdTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current main_bot_temperature for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckMainBotTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current main_battery voltage for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckBatteryVolt(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current lane_up for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckLaneUp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current lane_up for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckLaneUpCnt(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current lane_up for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckMainStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current lane_up for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckGpsStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current lidar_fault_status for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckLidarFaultStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current lidar_roi_status for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckLidarRoiStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current pl_vmon vin_p for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckPlVmonVinP(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current fpga_pl_kernal_temperature for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckFpgaPlKernalTemperature(
    diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current fpga_ps_kernal_temperature for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckFpgaPsKernalTemperature(
    diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current window_temperature for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void RobosenseCheckWindowTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  uint8_t current_diag_status;

  std::mutex mtx_config_;

  std::string info_model;
  std::string info_serial;
  bool is_get_info_serial_{false};
  drivers::AllDeviceCalculateParamsInfo device_param_val;
  void ReceiveHeliosMonitorInfoCallback(
    const robosense_msgs::msg::RobosenseHeliosMonitorInfo::SharedPtr monitor_info);
  void ReceiveBpRubyMonitorInfoCallback(
    const robosense_msgs::msg::RobosenseBpRubyMonitorInfo::SharedPtr monitor_info);
  void ReceiveMemsMonitorInfoCallback(
    const robosense_msgs::msg::RobosenseMemsMonitorInfo::SharedPtr monitor_info);
  void ReceiveFlashMonitorInfoCallback(
    const robosense_msgs::msg::RobosenseFlashMonitorInfo::SharedPtr monitor_info);
  rclcpp::Subscription<robosense_msgs::msg::RobosenseBpRubyMonitorInfo>::SharedPtr
    robosense_bp_ruby_monitor_info_sub_;
  rclcpp::Subscription<robosense_msgs::msg::RobosenseHeliosMonitorInfo>::SharedPtr
    robosense_helios_monitor_info_sub_;
  rclcpp::Subscription<robosense_msgs::msg::RobosenseMemsMonitorInfo>::SharedPtr
    robosense_mems_monitor_info_sub_;
  rclcpp::Subscription<robosense_msgs::msg::RobosenseFlashMonitorInfo>::SharedPtr
    robosense_flash_monitor_info_sub_;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_RobosenseHwMonitorRosWrapper_H
