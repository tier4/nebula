#include "nebula_ros/robosense/robosense_hw_monitor_ros_wrapper.hpp"

#include <memory>

namespace nebula
{
namespace ros
{
RobosenseHwMonitorRosWrapper::RobosenseHwMonitorRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("robosense_hw_monitor_ros_wrapper", options), diagnostics_updater_(this)
{
  interface_status_ = GetParameters(sensor_configuration_);
  if (Status::OK != interface_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
    return;
  }

  // robosense_info_sub_ = create_subscription<robosense_msgs::msg::RobosenseInfoPacket>(
  //   "robosense_difop_packets", rclcpp::SensorDataQoS(),
  //   std::bind(&RobosenseHwMonitorRosWrapper::ReceiveInfoMsgCallback, this, std::placeholders::_1));

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RobosenseHwMonitorRosWrapper::paramCallback, this, std::placeholders::_1));
}

Status RobosenseHwMonitorRosWrapper::MonitorStart()
{
  return interface_status_;
}

Status RobosenseHwMonitorRosWrapper::MonitorStop()
{
  return Status::OK;
}
Status RobosenseHwMonitorRosWrapper::Shutdown()
{
  return Status::OK;
}
//
Status RobosenseHwMonitorRosWrapper::InitializeHwMonitor(
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

Status RobosenseHwMonitorRosWrapper::GetParameters(
  drivers::RobosenseSensorConfiguration & sensor_configuration)
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", "");
    sensor_configuration.sensor_model =
      nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "milliseconds";
    this->declare_parameter<uint16_t>("diag_span", 1000, descriptor);
    diag_span_ = this->get_parameter("diag_span").as_int();
  }

  return Status::OK;
}

void RobosenseHwMonitorRosWrapper::InitializeRobosenseDiagnostics()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "InitializeRobosenseDiagnostics");
  diagnostics_updater_.setHardwareID(*hardware_id_);
  RCLCPP_INFO_STREAM(this->get_logger(), "hardware_id: " + *hardware_id_);

  diagnostics_updater_.add(
    "robosense_status", this, &RobosenseHwMonitorRosWrapper::RobosenseCheckStatus);

  auto on_timer_status = [this] { OnRobosenseStatusTimer(); };
  diagnostics_status_timer_ =
    this->create_wall_timer(std::chrono::milliseconds(diag_span_), std::move(on_timer_status));

  RCLCPP_DEBUG_STREAM(get_logger(), "add_timer");
}

void RobosenseHwMonitorRosWrapper::OnRobosenseStatusTimer()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "OnRobosenseStatusTimer" << std::endl);
  // info_driver_->DecodeInfoPacket(info_packet_buffer_);
  // current_sensor_info_ = info_driver_->GetSensorInfo();
  current_info_time_ = std::make_unique<rclcpp::Time>(this->get_clock()->now());
}

void RobosenseHwMonitorRosWrapper::RobosenseCheckStatus(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (!hardware_id_.has_value()) {
    return;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  for (const auto & info : current_sensor_info_) {
    diagnostics.add(info.first, info.second);
  }

  diagnostics.summary(level, "OK");
}

rcl_interfaces::msg::SetParametersResult RobosenseHwMonitorRosWrapper::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "add_on_set_parameters_callback");
  RCLCPP_DEBUG_STREAM(get_logger(), parameters);
  RCLCPP_DEBUG_STREAM(get_logger(), sensor_configuration_);
  RCLCPP_INFO_STREAM(this->get_logger(), parameters);

  drivers::RobosenseSensorConfiguration new_param{sensor_configuration_};
  RCLCPP_INFO_STREAM(this->get_logger(), new_param);
  uint16_t new_diag_span = 0;
  if (get_param(parameters, "diag_span", new_diag_span)) {
    sensor_configuration_ = new_param;
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
      std::make_shared<drivers::RobosenseSensorConfiguration>(sensor_configuration_);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "hw_interface_.SetSensorConfiguration");
  }

  auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  result->successful = true;
  result->reason = "success";

  RCLCPP_DEBUG_STREAM(this->get_logger(), "add_on_set_parameters_callback success");
  return *result;
}

// void RobosenseHwMonitorRosWrapper::ReceiveInfoMsgCallback(
//   const robosense_msgs::msg::RobosenseInfoPacket::SharedPtr info_msg)
// {
//   if (!info_driver_) {
//     auto sensor_cfg_ptr =
//       std::make_shared<drivers::RobosenseSensorConfiguration>(sensor_configuration_);

//     if (sensor_cfg_ptr->sensor_model == drivers::SensorModel::ROBOSENSE_BPEARL) {
//       if (
//         drivers::SensorModelFromString(info_msg->lidar_model) ==
//         drivers::SensorModel::ROBOSENSE_BPEARL_V3) {
//         sensor_cfg_ptr->sensor_model = drivers::SensorModel::ROBOSENSE_BPEARL_V3;
//       } else if (
//         drivers::SensorModelFromString(info_msg->lidar_model) ==
//         drivers::SensorModel::ROBOSENSE_BPEARL_V4) {
//         sensor_cfg_ptr->sensor_model = drivers::SensorModel::ROBOSENSE_BPEARL_V4;
//       } else {
//         RCLCPP_ERROR_STREAM(this->get_logger(), "No version for Bpearl.");
//         return;
//       }
//     }

//     info_driver_ = std::make_unique<drivers::RobosenseInfoDriver>(sensor_cfg_ptr);
//   }

//   info_packet_buffer_ =
//     std::vector<uint8_t>(info_msg->packet.data.begin(), info_msg->packet.data.end());

//   if (!hardware_id_.has_value()) {
//     info_driver_->DecodeInfoPacket(info_packet_buffer_);
//     current_sensor_info_ = info_driver_->GetSensorInfo();
//     current_info_time_ = std::make_unique<rclcpp::Time>(this->get_clock()->now());

//     RCLCPP_INFO_STREAM(this->get_logger(), "Model:" << sensor_configuration_.sensor_model);
//     RCLCPP_INFO_STREAM(this->get_logger(), "Serial:" << current_sensor_info_["serial_number"]);

//     hardware_id_.emplace(
//       nebula::drivers::SensorModelToString(sensor_configuration_.sensor_model) + ": " +
//       current_sensor_info_["serial_number"]);
//     InitializeRobosenseDiagnostics();
//   }
// }

RCLCPP_COMPONENTS_REGISTER_NODE(RobosenseHwMonitorRosWrapper)
}  // namespace ros
}  // namespace nebula
