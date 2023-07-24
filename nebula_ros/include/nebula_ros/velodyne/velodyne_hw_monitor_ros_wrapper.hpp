#ifndef NEBULA_VelodyneHwMonitorRosWrapper_H
#define NEBULA_VelodyneHwMonitorRosWrapper_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/velodyne/velodyne_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_velodyne/velodyne_hw_interface.hpp"
#include "nebula_ros/common/nebula_hw_monitor_ros_wrapper_base.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <mutex>

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

/// @brief Hardware monitor ros wrapper of velodyne driver
class VelodyneHwMonitorRosWrapper final : public rclcpp::Node, NebulaHwMonitorWrapperBase
{
  drivers::VelodyneHwInterface hw_interface_;
  Status interface_status_;

  drivers::VelodyneSensorConfiguration sensor_configuration_;
  drivers::VelodyneCalibrationConfiguration calibration_configuration_;

  /// @brief Initializing hardware monitor ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  Status InitializeHwMonitor(
    const drivers::SensorConfigurationBase & sensor_configuration) override;

public:
  explicit VelodyneHwMonitorRosWrapper(const rclcpp::NodeOptions & options);

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
  Status GetParameters(drivers::VelodyneSensorConfiguration & sensor_configuration);

private:  // ROS Diagnostics
  diagnostic_updater::Updater diagnostics_updater_;
  /// @brief Initializing diagnostics
  void InitializeVelodyneDiagnostics();
  /// @brief Get value from property_tree
  /// @param pt property_tree
  /// @param key Pey string
  /// @return Value
  std::string GetPtreeValue(
    std::shared_ptr<boost::property_tree::ptree> pt, const std::string & key);
  /// @brief Making fixed precision string
  /// @param val Target value
  /// @param pre Precision
  /// @return Created string
  std::string GetFixedPrecisionString(double val, int pre = 2);
  rclcpp::TimerBase::SharedPtr diagnostics_diag_timer_;
  std::shared_ptr<boost::property_tree::ptree> current_diag_tree;
  /// @brief Callback of the timer for getting the current lidar status & updating the diagnostics
  void OnVelodyneDiagnosticsTimer();

  /// @brief Getting top:hv from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopHv();
  /// @brief Getting top:ad_temp from the current property_tree (only VLP32)
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopAdTemp();  // only32
  /// @brief Getting top:lm20_temp from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopLm20Temp();
  /// @brief Getting top:pwr_5v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwr5v();
  /// @brief Getting top:pwr_2_5v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwr25v();
  /// @brief Getting top:pwr_3_3v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwr33v();
  /// @brief Getting top:pwr_5v_raw from the current property_tree (only VLP16)
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwr5vRaw();  // only16
  /// @brief Getting top:pwr_raw from the current property_tree (only VLP32)
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwrRaw();  // only32
  /// @brief Getting top:pwr_vccint from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwrVccint();
  /// @brief Getting bot:i_out from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotIOut();
  /// @brief Getting bot:pwr_1_2v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwr12v();
  /// @brief Getting bot:lm20_temp from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotLm20Temp();
  /// @brief Getting bot:pwr_5v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwr5v();
  /// @brief Getting bot:pwr_2_5v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwr25v();
  /// @brief Getting bot:pwr_3_3v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwr33v();
  /// @brief Getting bot:pwr_v_in from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwrVIn();
  /// @brief Getting bot:pwr_1_25v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwr125v();
  /// @brief Getting vhv from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetVhv();
  /// @brief Getting adc_nf from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetAdcNf();
  /// @brief Getting adc_stats from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetAdcStats();
  /// @brief Getting ixe from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetIxe();
  /// @brief Getting adctp_stat from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetAdctpStat();

  /// @brief Getting gps:pps_state from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetGpsPpsState();
  /// @brief Getting gps:position from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetGpsPosition();
  /// @brief Getting motor:state from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetMotorState();
  /// @brief Getting motor:rpm from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetMotorRpm();
  /// @brief Getting motor:lock from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetMotorLock();
  /// @brief Getting motor:phase from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetMotorPhase();
  /// @brief Getting laser:state from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetLaserState();

  /// @brief Check top:hv from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckTopHv(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:ad_temp from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckTopAdTemp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:lm20_temp from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckTopLm20Temp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:pwr_5v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckTopPwr5v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:pwr_2_5v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckTopPwr25v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:pwr_3_3v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckTopPwr33v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:pwr_raw from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckTopPwrRaw(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:pwr_vccint from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckTopPwrVccint(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:i_out from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckBotIOut(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_1_2v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckBotPwr12v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:lm20_temp from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckBotLm20Temp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_5v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckBotPwr5v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_2_5v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckBotPwr25v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_3_3v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckBotPwr33v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_v_in from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckBotPwrVIn(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_1_25v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckBotPwr125v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check vhv from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckVhv(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check adc_nf from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckAdcNf(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check adc_stats from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckAdcStats(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check ixe from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckIxe(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check adctp_stat from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckAdctpStat(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  rclcpp::TimerBase::SharedPtr diagnostics_status_timer_;
  std::shared_ptr<boost::property_tree::ptree> current_status_tree;
  /// @brief Callback of the timer for getting the current lidar status
  void OnVelodyneStatusTimer();
  /// @brief Check gps:pps_state from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckGpsPpsState(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check gps:position from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckGpsPosition(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check motor:state from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckMotorState(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check motor:rpm from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckMotorRpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check motor:lock from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckMotorLock(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check motor:phase from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckMotorPhase(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check laser:state from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckLaserState(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  /// @brief Check the current snapshot for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckSnapshot(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  /// @brief Check the current states of motor & laser for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current gps information for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckPps(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current temperatures for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current rpm for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckRpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current voltages for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void VelodyneCheckVoltage(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  /// @brief Callback of the timer for getting the current lidar snapshot
  void OnVelodyneSnapshotTimer();
  rclcpp::TimerBase::SharedPtr diagnostics_snapshot_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_update_timer_;
  std::shared_ptr<std::string> current_snapshot;
  std::shared_ptr<boost::property_tree::ptree> current_snapshot_tree;
  std::shared_ptr<rclcpp::Time> current_snapshot_time;
  //  rclcpp::Time current_snapshot_time;
  //  std::shared_ptr<uint8_t> current_diag_status;
  uint8_t current_diag_status;

  uint16_t diag_span_;
  std::mutex mtx_diag;
  std::mutex mtx_status;
  std::mutex mtx_config_;

  /// @brief Test callback function for getting json with curl
  /// @param err Error
  /// @param body Received body
  void curl_callback(std::string err, std::string body);

  const char * key_volt_temp_top_hv;
  const char * key_volt_temp_top_ad_temp;
  const char * key_volt_temp_top_lm20_temp;
  const char * key_volt_temp_top_pwr_5v;
  const char * key_volt_temp_top_pwr_2_5v;
  const char * key_volt_temp_top_pwr_3_3v;
  const char * key_volt_temp_top_pwr_5v_raw;
  const char * key_volt_temp_top_pwr_raw;
  const char * key_volt_temp_top_pwr_vccint;
  const char * key_volt_temp_bot_i_out;
  const char * key_volt_temp_bot_pwr_1_2v;
  const char * key_volt_temp_bot_lm20_temp;
  const char * key_volt_temp_bot_pwr_5v;
  const char * key_volt_temp_bot_pwr_2_5v;
  const char * key_volt_temp_bot_pwr_3_3v;
  const char * key_volt_temp_bot_pwr_v_in;
  const char * key_volt_temp_bot_pwr_1_25v;
  const char * key_vhv;
  const char * key_adc_nf;
  const char * key_adc_stats;
  const char * key_ixe;
  const char * key_adctp_stat;
  const char * key_status_gps_pps_state;
  const char * key_status_gps_pps_position;
  const char * key_status_motor_state;
  const char * key_status_motor_rpm;
  const char * key_status_motor_lock;
  const char * key_status_motor_phase;
  const char * key_status_laser_state;

  /*
  const char* name_volt_temp_top_hv;
  const char* name_volt_temp_top_ad_temp;
  const char* name_volt_temp_top_lm20_temp;
  const char* name_volt_temp_top_pwr_5v;
  const char* name_volt_temp_top_pwr_2_5v;
  const char* name_volt_temp_top_pwr_3_3v;
  const char* name_volt_temp_top_pwr_raw;
  const char* name_volt_temp_top_pwr_vccint;
  const char* name_volt_temp_bot_i_out;
  const char* name_volt_temp_bot_pwr_1_2v;
  const char* name_volt_temp_bot_lm20_temp;
  const char* name_volt_temp_bot_pwr_5v;
  const char* name_volt_temp_bot_pwr_2_5v;
  const char* name_volt_temp_bot_pwr_3_3v;
  const char* name_volt_temp_bot_pwr_v_in;
  const char* name_volt_temp_bot_pwr_1_25v;
  const char* name_vhv;
  const char* name_adc_nf;
  const char* name_adc_stats;
  const char* name_ixe;
  const char* name_adctp_stat;
  const char* name_status_gps_pps_state;
  const char* name_status_gps_pps_position;
  const char* name_status_motor_state;
  const char* name_status_motor_rpm;
  const char* name_status_motor_lock;
  const char* name_status_motor_phase;
  const char* name_status_laser_state;
  */

  std::string name_volt_temp_top_hv;
  std::string name_volt_temp_top_ad_temp;
  std::string name_volt_temp_top_lm20_temp;
  std::string name_volt_temp_top_pwr_5v;
  std::string name_volt_temp_top_pwr_2_5v;
  std::string name_volt_temp_top_pwr_3_3v;
  std::string name_volt_temp_top_pwr_5v_raw;
  std::string name_volt_temp_top_pwr_raw;
  std::string name_volt_temp_top_pwr_vccint;
  std::string name_volt_temp_bot_i_out;
  std::string name_volt_temp_bot_pwr_1_2v;
  std::string name_volt_temp_bot_lm20_temp;
  std::string name_volt_temp_bot_pwr_5v;
  std::string name_volt_temp_bot_pwr_2_5v;
  std::string name_volt_temp_bot_pwr_3_3v;
  std::string name_volt_temp_bot_pwr_v_in;
  std::string name_volt_temp_bot_pwr_1_25v;
  std::string name_vhv;
  std::string name_adc_nf;
  std::string name_adc_stats;
  std::string name_ixe;
  std::string name_adctp_stat;
  std::string name_status_gps_pps_state;
  std::string name_status_gps_pps_position;
  std::string name_status_motor_state;
  std::string name_status_motor_rpm;
  std::string name_status_motor_lock;
  std::string name_status_motor_phase;
  std::string name_status_laser_state;

  const char * not_supported_message;
  const char * error_message;
  std::string message_sep;

  const char * key_info_model;
  const char * key_info_serial;

  std::string temperature_cold_message;
  std::string temperature_hot_message;
  std::string voltage_low_message;
  std::string voltage_high_message;
  std::string ampere_low_message;
  std::string ampere_high_message;

  std::string info_model;
  std::string info_serial;

  bool use_advanced_diagnostics;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  /// @brief rclcpp parameter callback
  /// @param parameters Received parameters
  /// @return SetParametersResult
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  //  rclcpp::callback_group::CallbackGroup::SharedPtr cbg_;
  rclcpp::CallbackGroup::SharedPtr cbg_r_;
  rclcpp::CallbackGroup::SharedPtr cbg_m_;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_VelodyneHwMonitorRosWrapper_H
