// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

namespace nebula::ros
{
class VelodyneHwMonitorWrapper
{
public:
  VelodyneHwMonitorWrapper(
    rclcpp::Node * const parent_node,
    const std::shared_ptr<nebula::drivers::VelodyneHwInterface> & hw_interface,
    std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & /* new_config */)
  {
  }

  nebula::Status status();

private:
  void initialize_velodyne_diagnostics();

  /// @brief Callback of the timer for getting the current lidar status
  void on_velodyne_status_timer();

  /// @brief Callback of the timer for getting the current lidar status & updating the diagnostics
  void on_velodyne_diagnostics_timer();

  /// @brief Get value from property_tree
  /// @param pt property_tree
  /// @param mtx_pt the mutex associated with `pt`
  /// @param key Pey string
  /// @return Value
  std::string get_ptree_value(
    std::shared_ptr<boost::property_tree::ptree> pt, std::mutex & mtx_pt, const std::string & key);

  /// @brief Making fixed precision string
  /// @param val Target value
  /// @param pre Precision
  /// @return Created string
  std::string get_fixed_precision_string(double val, int pre = 2);

  /// @brief Getting top:hv from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_top_hv();
  /// @brief Getting top:ad_temp from the current property_tree (only VLP32)
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_top_ad_temp();  // only32
  /// @brief Getting top:lm20_temp from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_top_lm20_temp();
  /// @brief Getting top:pwr_5v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_top_pwr5v();
  /// @brief Getting top:pwr_2_5v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_top_pwr25v();
  /// @brief Getting top:pwr_3_3v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_top_pwr33v();
  /// @brief Getting top:pwr_5v_raw from the current property_tree (only VLP16)
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_top_pwr5v_raw();  // only16
  /// @brief Getting top:pwr_raw from the current property_tree (only VLP32)
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_top_pwr_raw();  // only32
  /// @brief Getting top:pwr_vccint from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_top_pwr_vccint();
  /// @brief Getting bot:i_out from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_bot_i_out();
  /// @brief Getting bot:pwr_1_2v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_bot_pwr12v();
  /// @brief Getting bot:lm20_temp from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_bot_lm20_temp();
  /// @brief Getting bot:pwr_5v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_bot_pwr5v();
  /// @brief Getting bot:pwr_2_5v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_bot_pwr25v();
  /// @brief Getting bot:pwr_3_3v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_bot_pwr33v();
  /// @brief Getting bot:pwr_v_in from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_bot_pwr_v_in();
  /// @brief Getting bot:pwr_1_25v from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_bot_pwr125v();
  /// @brief Getting vhv from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_vhv();
  /// @brief Getting adc_nf from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_adc_nf();
  /// @brief Getting adc_stats from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_adc_stats();
  /// @brief Getting ixe from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_ixe();
  /// @brief Getting adctp_stat from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_adctp_stat();

  /// @brief Getting gps:pps_state from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_gps_pps_state();
  /// @brief Getting gps:position from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_gps_position();
  /// @brief Getting motor:state from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_motor_state();
  /// @brief Getting motor:rpm from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_motor_rpm();
  /// @brief Getting motor:lock from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_motor_lock();
  /// @brief Getting motor:phase from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_motor_phase();
  /// @brief Getting laser:state from the current property_tree
  /// @return tuple<Got exception, Error level, Information message, Error message>
  std::tuple<bool, uint8_t, std::string, std::string> velodyne_get_laser_state();

  /// @brief Check top:hv from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_top_hv(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:ad_temp from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_top_ad_temp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:lm20_temp from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_top_lm20_temp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:pwr_5v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_top_pwr5v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:pwr_2_5v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_top_pwr25v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:pwr_3_3v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_top_pwr33v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:pwr_raw from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_top_pwr_raw(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check top:pwr_vccint from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_top_pwr_vccint(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:i_out from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_bot_i_out(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_1_2v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_bot_pwr12v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:lm20_temp from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_bot_lm20_temp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_5v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_bot_pwr5v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_2_5v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_bot_pwr25v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_3_3v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_bot_pwr33v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_v_in from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_bot_pwr_v_in(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check bot:pwr_1_25v from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_bot_pwr125v(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check vhv from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_vhv(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check adc_nf from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_adc_nf(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check adc_stats from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_adc_stats(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check ixe from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_ixe(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check adctp_stat from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_adctp_stat(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  rclcpp::TimerBase::SharedPtr diagnostics_status_timer_;
  std::shared_ptr<boost::property_tree::ptree> current_status_tree_;
  /// @brief Check gps:pps_state from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_gps_pps_state(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check gps:position from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_gps_position(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check motor:state from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_motor_state(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check motor:rpm from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_motor_rpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check motor:lock from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_motor_lock(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check motor:phase from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_motor_phase(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check laser:state from the current property_tree for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_laser_state(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  /// @brief Check the current states of motor & laser for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_status(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current gps information for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_pps(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current temperatures for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_temperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current rpm for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_rpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check the current voltages for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void velodyne_check_voltage(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  rclcpp::Logger logger_;
  diagnostic_updater::Updater diagnostics_updater_;

  const std::shared_ptr<nebula::drivers::VelodyneHwInterface> hw_interface_;
  rclcpp::Node * const parent_node_;

  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> sensor_configuration_;

  uint16_t diag_span_;
  bool show_advanced_diagnostics_;

  std::shared_ptr<boost::property_tree::ptree> current_diag_tree_;

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

  const std::string message_sep_{": "};
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
}  // namespace nebula::ros
