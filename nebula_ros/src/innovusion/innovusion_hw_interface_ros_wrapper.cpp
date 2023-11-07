#include "nebula_ros/innovusion/innovusion_hw_interface_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
InnovusionHwInterfaceRosWrapper::InnovusionHwInterfaceRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("Innovusion_hw_interface_ros_wrapper", options), hw_interface_()
{
  if (mtx_config_.try_lock()) {
    interface_status_ = GetParameters(sensor_configuration_);
    mtx_config_.unlock();
  }
  if (Status::OK != interface_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
    return;
  }
  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::InnovusionSensorConfiguration>(sensor_configuration_);
  if (this->setup_sensor) {
    hw_interface_.SetSensorConfiguration(
      std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
  }

  hw_interface_.RegisterScanCallback(
    std::bind(&InnovusionHwInterfaceRosWrapper::ReceiveScanDataCallback, this, std::placeholders::_1));
  innovusion_scan_pub_ =
    this->create_publisher<innovusion_msgs::msg::InnovusionScan>("innovusion_packets", rclcpp::SensorDataQoS());

  StreamStart();
}

InnovusionHwInterfaceRosWrapper::~InnovusionHwInterfaceRosWrapper() {
  RCLCPP_INFO_STREAM(get_logger(), "Closing TcpDriver");
}

Status InnovusionHwInterfaceRosWrapper::StreamStart()
{
  if (Status::OK == interface_status_) {
    interface_status_ = hw_interface_.CloudInterfaceStart();
  }
  return interface_status_;
}

Status InnovusionHwInterfaceRosWrapper::StreamStop() { return Status::OK; }
Status InnovusionHwInterfaceRosWrapper::Shutdown() { return Status::OK; }

Status InnovusionHwInterfaceRosWrapper::InitializeHwInterface(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

Status InnovusionHwInterfaceRosWrapper::GetParameters(
  drivers::InnovusionSensorConfiguration & sensor_configuration)
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
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("return_mode", "", descriptor);
    sensor_configuration.return_mode = nebula::drivers::ReturnModeFromStringInnovusion(
      this->get_parameter("return_mode").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("host_ip", "255.255.255.255", descriptor);
    sensor_configuration.host_ip = this->get_parameter("host_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_ip", "192.168.1.201", descriptor);
    sensor_configuration.sensor_ip = this->get_parameter("sensor_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "innovusion", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("data_port", 2368, descriptor);
    sensor_configuration.data_port = this->get_parameter("data_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<bool>("setup_sensor", true, descriptor);
    this->setup_sensor = this->get_parameter("setup_sensor").as_bool();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void InnovusionHwInterfaceRosWrapper::ReceiveScanDataCallback(
  std::unique_ptr<innovusion_msgs::msg::InnovusionScan> scan_buffer)
{
  // Publish
  scan_buffer->header.frame_id = sensor_configuration_.frame_id;
  scan_buffer->header.stamp = scan_buffer->packets.front().stamp;
  innovusion_scan_pub_->publish(*scan_buffer);
}

RCLCPP_COMPONENTS_REGISTER_NODE(InnovusionHwInterfaceRosWrapper)
}  // namespace ros
}  // namespace nebula
