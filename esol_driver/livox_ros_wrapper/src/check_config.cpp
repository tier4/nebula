#include "ros2_driver_wrapper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>

#include <string>

namespace lidar_driver
{
/// @brief get yaml file parameters
void RosDriverWrapper::GetParameter()
{
  if (config_.sensor_type.empty()) {
    RCLCPP_ERROR(this->get_logger(), "[Alert] Incorrect GetParameter: sensor_type is empty.");
  } else if (config_.sensor_type == std::string("livox")) {
    GetLivoxParameter(sensor_config_ex_, cloud_configuration_);
  }
#if 0   // ToDo : sensor_type
	else if( config_.sensor_type == std::string("velodyne") ) {
		GetVelodyneParameter(sensor_config_ex_, cloud_configuration_);
	}
	else if( config_.sensor_type == std::string("pandar") ) {
		GetPandarParameter(sensor_config_ex_, cloud_configuration_);
	}
#endif  // ToDo : sensor_type
  else {
    RCLCPP_ERROR(
      this->get_logger(), "[Alert] Incorrect GetParameter: sensor_type %s unsupported.",
      config_.sensor_type.c_str());
  }
}

/// @brief check IP address.
/// @param ip_addr
/// @return true: good address. false: bad address.
bool RosDriverWrapper::CheckIpAddress(const std::string & ip_addr)
{
  bool ret = false;
  if (!ip_addr.empty()) {
    in_addr_t ip_value = inet_addr(ip_addr.c_str());
    if (ip_value == INADDR_NONE) {
    } else if (ip_value == 0) {
    } else {
      ret = true;
    }
  }
  return ret;
}

/// @brief Check Configuration.
/// @details
/// Figure 10.
///	CheckConfiguration: Checks the configuration set either at launch or during runtime using
///	the selected ROS method for configuration (services or dynamic reconfigure). It calls
///	consequently to the CheckSensorConfiguration, and CheckOutputCloudConfiguration.
/// @param sensor_configuration :
/// @param cloud_configuration :
/// @param sensor_calibration : optional param
bool RosDriverWrapper::CheckConfiguration(
  SensorConfigEx & sensor_config_ex, CloudConfiguration & cloud_config)
{
  bool ret = false;

  if (!CheckSensorConfiguration(sensor_config_ex)) {
  } else if (!CheckOutputCloudConfiguration(sensor_config_ex, cloud_config)) {
  } else {
    ret = true;
  }

  return ret;
}

/// Figure 10.
/// CheckSensorConfiguration
/// Checks that the desired parameters are correct and apply for the selected sensor.
bool RosDriverWrapper::CheckSensorConfiguration(SensorConfigEx & sensor_config_ex)
{
  bool ret = false;

  if (config_.sensor_type.empty()) {
    RCLCPP_ERROR(
      this->get_logger(), "[Alert] Incorrect Sensor Configuration: sensor_type is empty.");
  } else if (config_.sensor_type == std::string("livox")) {
    ret = CheckLivoxSensorConfiguration(sensor_config_ex);
  }
#if 0   // ToDo : sensor_type
	else if( config_.sensor_type == std::string("velodyne") ) {
		ret = CheckVelodyneSensorConfiguration();
	}
	else if( config_.sensor_type == std::string("pandar") ) {
		ret = CheckPandarSensorConfiguration();
	}
#endif  // ToDo : sensor_type
  else {
    RCLCPP_ERROR(
      this->get_logger(), "[Alert] Incorrect Sensor Configuration: sensor_type %s unsupported.",
      config_.sensor_type.c_str());
  }

  return ret;
}

/// Figure 10.
/// CheckOutputCloudConfiguration
/// @brief Checks that the desired parameters and fields for the output to be generated correspond
/// to the selected sensor.
bool RosDriverWrapper::CheckOutputCloudConfiguration(
  SensorConfigEx & sensor_config_ex, CloudConfiguration & cloud_config)
{
  bool ret = false;

  if (config_.sensor_type.empty()) {
    RCLCPP_ERROR(
      this->get_logger(), "[Alert] Incorrect Cloud Configuration: sensor_type is empty.");
  } else if (config_.sensor_type == std::string("livox")) {
    ret = CheckLivoxCloudConfiguration(sensor_config_ex, cloud_config);
  }
#if 0   // ToDo : sensor_type
	else if( config_.sensor_type == std::string("velodyne") ) {
		ret = CheckVelodyneCloudConfiguration(sensor_config_ex, cloud_config);
	}
	else if( config_.sensor_type == std::string("pandar") ) {
		ret = CheckPandarCloudConfiguration(sensor_config_ex, cloud_config);
	}
#endif  // ToDo : sensor_type
  else {
    RCLCPP_ERROR(
      this->get_logger(), "[Alert] Incorrect Cloud Configuration: sensor_type %s unsupported.",
      config_.sensor_type);
  }

  return ret;
}

// ToDo: ConfigureOutputCloudConfiguration: Sets the configuration that

/// Figure 9.
/// @brief GetSensorConfig: Obtains the actual configuration parameters from the sensor.
SensorConfiguration RosDriverWrapper::GetSensorConfig()
{
  SensorConfiguration sensor_config;
  driver_.GetConfiguration(sensor_config);
  return sensor_config;
}

}  // namespace lidar_driver
