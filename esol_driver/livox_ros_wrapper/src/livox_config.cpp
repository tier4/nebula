#include <cmath>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "LidarDriver/livox_common.hpp"
#include "ros2_driver_wrapper.hpp"

namespace lidar_driver
{
/// @brief get parameter LivoxSensorModel.
RosDriverWrapper::LivoxSensorModelEx RosDriverWrapper::GetParamLivoxSensorModel()
{
  std::map<const std::string, livox_driver::LivoxSensorModel> tbl = {
    {"horizon", livox_driver::LivoxSensorModel::HORIZON}};
  livox_driver::LivoxSensorModel ret;
  std::string str;
  this->declare_parameter("sensor_model", "horizon");
  this->get_parameter("sensor_model", str);
  try {
    ret = tbl.at(str);
  } catch (std::out_of_range &) {
    ret = livox_driver::LivoxSensorModel::UNKNOWN;
  }
  return {ret, str};
}

/// @brief get parameter LivoxEchoMode.
RosDriverWrapper::LivoxEchoModeEx RosDriverWrapper::GetParamLivoxEchoMode()
{
  std::map<const std::string, livox_driver::LivoxEchoMode> tbl = {
    {"single_first", livox_driver::LivoxEchoMode::SINGLE_FIRST},
    {"single_strongest", livox_driver::LivoxEchoMode::SINGLE_STRONGEST},
    {"dual", livox_driver::LivoxEchoMode::DUAL}};
  livox_driver::LivoxEchoMode ret;
  std::string str;

  this->declare_parameter("echo_mode", "single_first");
  this->get_parameter("echo_mode", str);
  try {
    ret = tbl.at(str);
  } catch (std::out_of_range &) {
    ret = livox_driver::LivoxEchoMode::UNKNOWN;
  }
  return {ret, str};
}

/// @brief get parameter LivoxCoordinateMode.
RosDriverWrapper::LivoxCoordinateModeEx RosDriverWrapper::GetParamLivoxCoordinateMode()
{
  std::map<const std::string, livox_driver::LivoxCoordinateMode> tbl = {
    {"cartesian", livox_driver::LivoxCoordinateMode::CARTESIAN},
    {"spherical", livox_driver::LivoxCoordinateMode::SPHERICAL}};
  livox_driver::LivoxCoordinateMode ret;
  std::string str;

  this->declare_parameter("coordinate_mode", "cartesian");
  this->get_parameter("coordinate_mode", str);
  try {
    ret = tbl.at(str);
  } catch (std::out_of_range &) {
    ret = livox_driver::LivoxCoordinateMode::UNKNOWN;
  }
  return {ret, str};
}

/// @brief get livox configuration parameter.
void RosDriverWrapper::GetLivoxParameter(
  SensorConfigEx & ex, livox_driver::LivoxCloudConfiguration & cloud_config)
{
  std::string str_coordinate_mode;

  this->declare_parameter("host_ip", "");
  this->declare_parameter("sensor_ip", "");
  this->declare_parameter("data_port", 0);
  this->declare_parameter("imu_port", 0);
  this->declare_parameter("cmd_port", 0);

  this->declare_parameter("frequency", 100);

  this->declare_parameter("intensity", true);
  this->declare_parameter("point_timestamp", true);
  this->declare_parameter("point_echo", true);
  this->declare_parameter("cloud_min_range", 0.0);
  this->declare_parameter("cloud_max_range", 0.0);
  this->declare_parameter("remove_nans", true);

  LivoxSensorModelEx sensor_model_ex = GetParamLivoxSensorModel();
  ex.sensor_config.sensor_model = std::get<0>(sensor_model_ex);
  ex.sensor_model_str = std::get<1>(sensor_model_ex);

  // LivoxSensorConfiguration
  this->get_parameter("host_ip", ex.sensor_config.host_ip);
  this->get_parameter("sensor_ip", ex.sensor_config.sensor_ip);
  this->get_parameter("data_port", ex.sensor_config.data_port);
  this->get_parameter("imu_port", ex.sensor_config.imu_port);
  this->get_parameter("cmd_port", ex.sensor_config.cmd_port);
  this->get_parameter("frequency", ex.sensor_config.frequency_ms);

  LivoxEchoModeEx echo_mode_ex = GetParamLivoxEchoMode();
  ex.sensor_config.echo_mode = std::get<0>(echo_mode_ex);
  cloud_config.echo_mode = std::get<0>(echo_mode_ex);
  ex.echo_mode_str = std::get<1>(echo_mode_ex);

  LivoxCoordinateModeEx coordinate_mode_ex = GetParamLivoxCoordinateMode();
  ex.sensor_config.coordinate_model = std::get<0>(coordinate_mode_ex);
  cloud_config.coordinate_model = std::get<0>(coordinate_mode_ex);
  ex.coordinate_mode_str = std::get<1>(coordinate_mode_ex);

  // LivoxCloudConfiguration
  this->get_parameter("intensity", cloud_config.intensity);
  this->get_parameter("point_timestamp", cloud_config.point_timestamp);

  this->get_parameter("cloud_min_range", cloud_config.cloud_min_range);
  this->get_parameter("cloud_max_range", cloud_config.cloud_max_range);
  this->get_parameter("remove_nans", cloud_config.remove_nans);
}

/// @brief Checks that the desired parameters are correct and apply for the selected sensor.
/// @param sensor_config: Livox Sensor Configuration
/// @return true on success, false on failure.
bool RosDriverWrapper::CheckLivoxSensorConfiguration(SensorConfigEx & ex)
{
  std::string err_str = "";
  bool ret = false;

  if (ex.sensor_config.sensor_model == livox_driver::LivoxSensorModel::UNKNOWN) {
    err_str = "sensor_model '" + ex.sensor_model_str + "'";
  } else if (!CheckIpAddress(ex.sensor_config.host_ip)) {
    err_str = "host_ip '" + ex.sensor_config.host_ip + "'";
  } else if (!CheckIpAddress(ex.sensor_config.sensor_ip)) {
    err_str = "sensor_ip '" + ex.sensor_config.sensor_ip + "'";
  } else if (!CheckPortNumber(ex.sensor_config.data_port)) {
    err_str = "data_port " + std::to_string(ex.sensor_config.data_port);
  } else if (!CheckPortNumber(ex.sensor_config.imu_port)) {
    err_str = "imu_port " + std::to_string(ex.sensor_config.imu_port);
  } else if (!CheckPortNumber(ex.sensor_config.cmd_port)) {
    err_str = "cmd_port " + std::to_string(ex.sensor_config.cmd_port);
  } else if ((ex.sensor_config.frequency_ms < 5) || (ex.sensor_config.frequency_ms > 1000)) {
    err_str = "frequency_ms " + std::to_string(ex.sensor_config.frequency_ms);
  }
#if 0  // ToDo: echo_mode and coordinate_mode must not be implemented in the first beta version of the driver.
  else if( ex.sensor_config.echo_mode == livox_driver::LivoxEchoMode::kUnknown ) {
    err_str = "echo_mode '" + ex.sensor_config.echo_mode_str + "'";
  }
  else if( ex.sensor_config.coordinate_model == livox_driver::LivoxCoordinateMode::kUnknown ) {
    err_str = "coordinate_mode '" + ex.sensor_config.coordinate_mode_str + "'";
  }
#endif  // ToDo: echo_mode and coordinate_mode must not be implemented in the first beta version of the driver.
  else {
    ret = true;
  }

  if (err_str.length() > 0) {
    RCLCPP_ERROR(
      this->get_logger(), "[Alert] Incorrect Sensor Configuration: %s error.", err_str.c_str());
  }

  return ret;
}

/// @brief Checks that the desired parameters are correct and apply for the selected sensor.
/// @param sensor_config: Livox Sensor Configuration
/// @param cloud_config: Livox Cloud Configuration
/// @return true on success, false on failure.
/// @details boolean is No check required.
bool RosDriverWrapper::CheckLivoxCloudConfiguration(
  SensorConfigEx & ex, livox_driver::LivoxCloudConfiguration & cloud_config)
{
  std::string err_str = "";
  bool ret = false;

  if (ex.sensor_config.sensor_model == livox_driver::LivoxSensorModel::UNKNOWN) {
    err_str = "sensor_model '" + ex.sensor_model_str + "'";
  }
#if 0  // ToDo: Point_echo and echo_mode must not be implemented in the first beta version of the driver.
  else if( cloud_config.echo_mode == livox_driver::LivoxEchoMode::kUnknown ) {
    err_str = "echo_mode '" + ex.sensor_config.echo_mode_str + "'";
  }
  else if( cloud_config.coordinate_model == livox_driver::LivoxCoordinateMode::kUnknown ) {
    err_str = "coordinate_mode '" + ex.sensor_config.coordinate_mode_str + "'";
  }
#endif  // ToDo: Point_echo and echo_mode must not be implemented in the first beta version of the driver.
  else if (std::signbit(cloud_config.cloud_min_range)) {
    err_str = "cloud_min_range" + std::to_string(cloud_config.cloud_min_range);
  } else if (std::signbit(cloud_config.cloud_max_range)) {
    err_str = "cloud_max_range" + std::to_string(cloud_config.cloud_max_range);
  } else {
    ret = true;
  }

  if (err_str.length() > 0) {
    RCLCPP_ERROR(
      this->get_logger(), "[Alert] Incorrect Cloud Configuration: %s error.", err_str.c_str());
  }

  return ret;
}

}  // namespace lidar_driver
