#pragma once

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/velodyne/velodyne_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_velodyne/velodyne_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <array>
#include <memory>
#include <string>
#include <tuple>

namespace nebula
{
namespace ros
{
class VelodyneHwMonitorWrapper
{
public:
  VelodyneHwMonitorWrapper(
    rclcpp::Node * const parent_node,
    const std::shared_ptr<nebula::drivers::VelodyneHwInterface> & hw_interface,
    std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config);

  void OnConfigChange(
    const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & /* new_config */)
  {
  }

  nebula::Status Status();

private:
  void InitializeVelodyneDiagnostics();

  /// @brief Callback of the timer for getting the current lidar status
  void OnVelodyneStatusTimer();

  /// @brief Callback of the timer for getting the current lidar snapshot
  void OnVelodyneSnapshotTimer();

  /// @brief Callback of the timer for getting the current lidar status & updating the diagnostics
  void OnVelodyneDiagnosticsTimer();

  /// @brief Get value from property_tree
  /// @param pt property_tree
  /// @param mtx_pt the mutex associated with `pt`
  /// @param key Pey string
  /// @return Value
  std::string GetPtreeValue(
    std::shared_ptr<boost::property_tree::ptree> pt, std::mutex & mtx_pt, const std::string & key);

  /// @brief Making fixed precision string
  /// @param val Target value
  /// @param pre Precision
  /// @return Created string
  std::string GetFixedPrecisionString(double val, int pre = 2);

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

  rclcpp::Logger logger_;
  diagnostic_updater::Updater diagnostics_updater_;
  nebula::Status status_;

  const std::shared_ptr<nebula::drivers::VelodyneHwInterface> hw_interface_;
  rclcpp::Node * const parent_node_;

  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> sensor_configuration_;

  uint16_t diag_span_;
  bool show_advanced_diagnostics_;

  rclcpp::TimerBase::SharedPtr diagnostics_snapshot_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_update_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_diag_timer_;

  std::shared_ptr<std::string> current_snapshot;
  std::shared_ptr<boost::property_tree::ptree> current_snapshot_tree;
  std::shared_ptr<boost::property_tree::ptree> current_diag_tree;
  std::shared_ptr<rclcpp::Time> current_snapshot_time;
  uint8_t current_diag_status;

  std::mutex mtx_snapshot_;
  std::mutex mtx_status_;
  std::mutex mtx_diag_;

  std::string info_model_;
  std::string info_serial_;

  static constexpr auto key_volt_temp_top_hv = "volt_temp.top.hv";
  static constexpr auto key_volt_temp_top_ad_temp = "volt_temp.top.ad_temp";  // only32
  static constexpr auto key_volt_temp_top_lm20_temp = "volt_temp.top.lm20_temp";
  static constexpr auto key_volt_temp_top_pwr_5v = "volt_temp.top.pwr_5v";
  static constexpr auto key_volt_temp_top_pwr_2_5v = "volt_temp.top.pwr_2_5v";
  static constexpr auto key_volt_temp_top_pwr_3_3v = "volt_temp.top.pwr_3_3v";
  static constexpr auto key_volt_temp_top_pwr_5v_raw = "volt_temp.top.pwr_5v_raw";  // only16
  static constexpr auto key_volt_temp_top_pwr_raw = "volt_temp.top.pwr_raw";        // only32
  static constexpr auto key_volt_temp_top_pwr_vccint = "volt_temp.top.pwr_vccint";
  static constexpr auto key_volt_temp_bot_i_out = "volt_temp.bot.i_out";
  static constexpr auto key_volt_temp_bot_pwr_1_2v = "volt_temp.bot.pwr_1_2v";
  static constexpr auto key_volt_temp_bot_lm20_temp = "volt_temp.bot.lm20_temp";
  static constexpr auto key_volt_temp_bot_pwr_5v = "volt_temp.bot.pwr_5v";
  static constexpr auto key_volt_temp_bot_pwr_2_5v = "volt_temp.bot.pwr_2_5v";
  static constexpr auto key_volt_temp_bot_pwr_3_3v = "volt_temp.bot.pwr_3_3v";
  static constexpr auto key_volt_temp_bot_pwr_v_in = "volt_temp.bot.pwr_v_in";
  static constexpr auto key_volt_temp_bot_pwr_1_25v = "volt_temp.bot.pwr_1_25v";
  static constexpr auto key_vhv = "vhv";
  static constexpr auto key_adc_nf = "adc_nf";
  static constexpr auto key_adc_stats = "adc_stats";
  static constexpr auto key_ixe = "ixe";
  static constexpr auto key_adctp_stat = "adctp_stat";
  static constexpr auto key_status_gps_pps_state = "gps.pps_state";
  static constexpr auto key_status_gps_pps_position = "gps.position";
  static constexpr auto key_status_motor_state = "motor.state";
  static constexpr auto key_status_motor_rpm = "motor.rpm";
  static constexpr auto key_status_motor_lock = "motor.lock";
  static constexpr auto key_status_motor_phase = "motor.phase";
  static constexpr auto key_status_laser_state = "laser.state";

  static constexpr auto name_volt_temp_top_hv = "Top HV";
  static constexpr auto name_volt_temp_top_ad_temp = "Top A/D TD";
  static constexpr auto name_volt_temp_top_lm20_temp = "Top Temp";
  static constexpr auto name_volt_temp_top_pwr_5v = "Top 5v";
  static constexpr auto name_volt_temp_top_pwr_2_5v = "Top 2.5v";
  static constexpr auto name_volt_temp_top_pwr_3_3v = "Top 3.3v";
  static constexpr auto name_volt_temp_top_pwr_5v_raw = "Top 5v(RAW)";
  static constexpr auto name_volt_temp_top_pwr_raw = "Top RAW";
  static constexpr auto name_volt_temp_top_pwr_vccint = "Top VCCINT";
  static constexpr auto name_volt_temp_bot_i_out = "Bot I out";
  static constexpr auto name_volt_temp_bot_pwr_1_2v = "Bot 1.2v";
  static constexpr auto name_volt_temp_bot_lm20_temp = "Bot Temp";
  static constexpr auto name_volt_temp_bot_pwr_5v = "Bot 5v";
  static constexpr auto name_volt_temp_bot_pwr_2_5v = "Bot 2.5v";
  static constexpr auto name_volt_temp_bot_pwr_3_3v = "Bot 3.3v";
  static constexpr auto name_volt_temp_bot_pwr_v_in = "Bot V in";
  static constexpr auto name_volt_temp_bot_pwr_1_25v = "Bot 1.25v";  // N/A?
  static constexpr auto name_vhv = "VHV";
  static constexpr auto name_adc_nf = "adc_nf";
  static constexpr auto name_adc_stats = "adc_stats";
  static constexpr auto name_ixe = "ixe";
  static constexpr auto name_adctp_stat = "adctp_stat";
  static constexpr auto name_status_gps_pps_state = "GPS PPS";
  static constexpr auto name_status_gps_pps_position = "GPS Position";
  static constexpr auto name_status_motor_state = "Motor State";
  static constexpr auto name_status_motor_rpm = "Motor RPM";
  static constexpr auto name_status_motor_lock = "Motor Lock";
  static constexpr auto name_status_motor_phase = "Motor Phase";
  static constexpr auto name_status_laser_state = "Laser State";

  const std::string message_sep{": "};
  static constexpr auto not_supported_message = "Not supported";
  static constexpr auto error_message = "Error";

  static constexpr auto key_info_model = "info.model";
  static constexpr auto key_info_serial = "info.serial";

  static constexpr auto temperature_cold_message = "temperature cold";
  static constexpr auto temperature_hot_message = "temperature hot";

  static constexpr auto voltage_low_message = "voltage low";
  static constexpr auto voltage_high_message = "voltage high";

  static constexpr auto ampere_low_message = "ampere low";
  static constexpr auto ampere_high_message = "ampere high";
};
}  // namespace ros
}  // namespace nebula
