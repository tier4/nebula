#ifndef NEBULA_VelodyneHwInterfaceRosWrapper_H
#define NEBULA_VelodyneHwInterfaceRosWrapper_H

#include "common/nebula_common.hpp"
#include "common/nebula_hw_interface_ros_wrapper_base.hpp"
#include "velodyne/velodyne_common.hpp"
#include "velodyne/velodyne_hw_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
//#include <diagnostic_updater/diagnostic_updater.hpp>

#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

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

class VelodyneHwInterfaceRosWrapper final : public rclcpp::Node, NebulaHwInterfaceWrapperBase
{
  drivers::VelodyneHwInterface hw_interface_;
  Status interface_status_;

  drivers::VelodyneSensorConfiguration sensor_configuration_;
  drivers::VelodyneCalibrationConfiguration calibration_configuration_;

  rclcpp::Publisher<velodyne_msgs::msg::VelodyneScan>::SharedPtr velodyne_scan_pub_;

  Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) override;
  void ReceiveScanDataCallback(std::unique_ptr<velodyne_msgs::msg::VelodyneScan> scan_buffer);

public:
  explicit VelodyneHwInterfaceRosWrapper(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  Status StreamStart() override;
  Status StreamStop() override;
  Status Shutdown() override;
  Status GetParameters(drivers::VelodyneSensorConfiguration & sensor_configuration);

private://ROS Diagnostics
/*
  diagnostic_updater::Updater diagnostics_updater_;
  void InitializeVelodyneDiagnostics();
*/
  std::string GetPtreeValue(std::shared_ptr<boost::property_tree::ptree> pt, const std::string& key);
/*
  rclcpp::TimerBase::SharedPtr diagnostics_diag_timer_;
*/
  std::shared_ptr<boost::property_tree::ptree> current_diag_tree;
/*
  void OnVelodyneDiagnosticsTimer();
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
*/
  std::shared_ptr<boost::property_tree::ptree> current_status_tree;
/*
  void OnVelodyneStatusTimer();
  void VelodyneCheckGpsPpsState(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckGpsPosition(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckMotorState(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckMotorRpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckMotorLock(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckMotorPhase(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void VelodyneCheckLaserState(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void VelodyneCheckSnapshot(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void OnVelodyneSnapshotTimer();
  rclcpp::TimerBase::SharedPtr diagnostics_snapshot_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_update_timer_;
*/
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
/*
  const char* key_volt_temp_top_hv;
  const char* key_volt_temp_top_ad_temp;
  const char* key_volt_temp_top_lm20_temp;
  const char* key_volt_temp_top_pwr_5v;
  const char* key_volt_temp_top_pwr_2_5v;
  const char* key_volt_temp_top_pwr_3_3v;
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
*/
  const char* not_supported_message;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & parameters);
  std::vector<rcl_interfaces::msg::SetParametersResult> updateParameters();

//  rclcpp::callback_group::CallbackGroup::SharedPtr cbg_;
  rclcpp::CallbackGroup::SharedPtr cbg_r_;
  rclcpp::CallbackGroup::SharedPtr cbg_m_;


};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_VelodyneHwInterfaceRosWrapper_H