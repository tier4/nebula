#ifndef NEBULA_VelodyneHwMonitorRosWrapper_H
#define NEBULA_VelodyneHwMonitorRosWrapper_H

#include "common/nebula_common.hpp"
#include "common/nebula_hw_monitor_ros_wrapper_base.hpp"
#include "velodyne/velodyne_common.hpp"
#include "velodyne/velodyne_hw_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

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

class VelodyneHwMonitorRosWrapper final : public rclcpp::Node, NebulaHwMonitorWrapperBase
{
  drivers::VelodyneHwInterface hw_interface_;
  Status interface_status_;

  drivers::VelodyneSensorConfiguration sensor_configuration_;
  drivers::VelodyneCalibrationConfiguration calibration_configuration_;


  Status InitializeHwMonitor(
    const drivers::SensorConfigurationBase & sensor_configuration) override;
    
public:
  explicit VelodyneHwMonitorRosWrapper(const rclcpp::NodeOptions & options);

  Status MonitorStart() override;
  Status MonitorStop() override;
  Status Shutdown() override;
  Status GetParameters(drivers::VelodyneSensorConfiguration & sensor_configuration);

private://ROS Diagnostics
  diagnostic_updater::Updater diagnostics_updater_;
  void InitializeVelodyneDiagnostics();
  std::string GetPtreeValue(std::shared_ptr<boost::property_tree::ptree> pt, const std::string& key);
  std::string GetFixedPrecisionString(double val, int pre=2);
  rclcpp::TimerBase::SharedPtr diagnostics_diag_timer_;
  std::shared_ptr<boost::property_tree::ptree> current_diag_tree;
  void OnVelodyneDiagnosticsTimer();

  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopHv();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopAdTemp();//only32
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopLm20Temp();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwr5v();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwr25v();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwr33v();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwr5vRaw();//only16
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwrRaw();//only32
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetTopPwrVccint();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotIOut();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwr12v();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotLm20Temp();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwr5v();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwr25v();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwr33v();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwrVIn();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetBotPwr125v();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetVhv();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetAdcNf();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetAdcStats();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetIxe();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetAdctpStat();
  
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetGpsPpsState();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetGpsPosition();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetMotorState();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetMotorRpm();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetMotorLock();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetMotorPhase();
  std::tuple<bool, uint8_t, std::string, std::string> VelodyneGetLaserState();


  void VelodyneCheckTopHv(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckTopAdTemp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckTopLm20Temp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckTopPwr5v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckTopPwr25v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckTopPwr33v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckTopPwrRaw(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckTopPwrVccint(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckBotIOut(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckBotPwr12v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckBotLm20Temp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckBotPwr5v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckBotPwr25v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckBotPwr33v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckBotPwrVIn(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckBotPwr125v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckVhv(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckAdcNf(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckAdcStats(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckIxe(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckAdctpStat(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  rclcpp::TimerBase::SharedPtr diagnostics_status_timer_;
  std::shared_ptr<boost::property_tree::ptree> current_status_tree;
  void OnVelodyneStatusTimer();
  void VelodyneCheckGpsPpsState(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckGpsPosition(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckMotorState(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckMotorRpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckMotorLock(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckMotorPhase(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckLaserState(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void VelodyneCheckSnapshot(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void VelodyneCheckStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckPps(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckRpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckVoltage(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

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

  void curl_callback(std::string err, std::string body);

  const char* key_volt_temp_top_hv;
  const char* key_volt_temp_top_ad_temp;
  const char* key_volt_temp_top_lm20_temp;
  const char* key_volt_temp_top_pwr_5v;
  const char* key_volt_temp_top_pwr_2_5v;
  const char* key_volt_temp_top_pwr_3_3v;
  const char* key_volt_temp_top_pwr_5v_raw;
  const char* key_volt_temp_top_pwr_raw;
  const char* key_volt_temp_top_pwr_vccint;
  const char* key_volt_temp_bot_i_out;
  const char* key_volt_temp_bot_pwr_1_2v;
  const char* key_volt_temp_bot_lm20_temp;
  const char* key_volt_temp_bot_pwr_5v;
  const char* key_volt_temp_bot_pwr_2_5v;
  const char* key_volt_temp_bot_pwr_3_3v;
  const char* key_volt_temp_bot_pwr_v_in;
  const char* key_volt_temp_bot_pwr_1_25v;
  const char* key_vhv;
  const char* key_adc_nf;
  const char* key_adc_stats;
  const char* key_ixe;
  const char* key_adctp_stat;
  const char* key_status_gps_pps_state;
  const char* key_status_gps_pps_position;
  const char* key_status_motor_state;
  const char* key_status_motor_rpm;
  const char* key_status_motor_lock;
  const char* key_status_motor_phase;
  const char* key_status_laser_state;

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

  const char* not_supported_message;
  const char* error_message;
  std::string  message_sep;

  const char* key_info_model;
  const char* key_info_serial;

  std::string  temperature_cold_message;
  std::string  temperature_hot_message;
  std::string  voltage_low_message;
  std::string  voltage_high_message;
  std::string  ampere_low_message;
  std::string  ampere_high_message;

  std::string info_model;
  std::string info_serial;

  bool use_advanced_diagnostics;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & parameters);

//  rclcpp::callback_group::CallbackGroup::SharedPtr cbg_;
  rclcpp::CallbackGroup::SharedPtr cbg_r_;
  rclcpp::CallbackGroup::SharedPtr cbg_m_;

};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_VelodyneHwMonitorRosWrapper_H