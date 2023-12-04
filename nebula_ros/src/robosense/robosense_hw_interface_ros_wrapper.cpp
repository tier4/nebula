#include "nebula_ros/robosense/robosense_hw_interface_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
ros::RobosenseHwInterfaceRosWrapper::RobosenseHwInterfaceRosWrapper(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("robosense_hw_interface_ros_wrapper", options)
{
  interface_status_ = GetParameters(sensor_configuration_);

  if (Status::OK != interface_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
    return;
  }

  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));

  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::RobosenseSensorConfiguration>(sensor_configuration_);

  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  hw_interface_.RegisterScanCallback(std::bind(
    &RobosenseHwInterfaceRosWrapper::ReceiveScanDataCallback, this, std::placeholders::_1));

  hw_interface_.RegisterInfoCallback(std::bind(
    &RobosenseHwInterfaceRosWrapper::ReceiveInfoDataCallback, this, std::placeholders::_1));

  robosense_scan_pub_ = this->create_publisher<robosense_msgs::msg::RobosenseScan>(
    "robosense_packets", rclcpp::SensorDataQoS());

  robosense_difop_pub_ = this->create_publisher<robosense_msgs::msg::RobosenseInfoPacket>(
    "robosense_difop_packets", rclcpp::SensorDataQoS());

  StreamStart();
}

Status RobosenseHwInterfaceRosWrapper::StreamStart()
{
  if (Status::OK == interface_status_) {
    RCLCPP_INFO_STREAM(get_logger(), "Starting interface.");
    hw_interface_.CloudInterfaceStart();
    hw_interface_.InfoInterfaceStart();
  }
  return interface_status_;
}
Status RobosenseHwInterfaceRosWrapper::StreamStop()
{
  return Status::OK;
}
Status RobosenseHwInterfaceRosWrapper::Shutdown()
{
  return Status::OK;
}

Status RobosenseHwInterfaceRosWrapper::InitializeHwInterface(
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

Status RobosenseHwInterfaceRosWrapper::GetParameters(
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
    this->declare_parameter<std::string>("sensor_ip", "192.168.1.200", descriptor);
    sensor_configuration.sensor_ip = this->get_parameter("sensor_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "robosense", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("data_port", 6699, descriptor);
    sensor_configuration.data_port = this->get_parameter("data_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("gnss_port", 7788, descriptor);
    sensor_configuration.gnss_port = this->get_parameter("gnss_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0).set__to_value(360).set__step(0.01);
    descriptor.floating_point_range = {range};
    this->declare_parameter<double>("scan_phase", 0., descriptor);
    sensor_configuration.scan_phase = this->get_parameter("scan_phase").as_double();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void RobosenseHwInterfaceRosWrapper::ReceiveScanDataCallback(
  std::unique_ptr<robosense_msgs::msg::RobosenseScan> scan_buffer)
{
  // Publish
  scan_buffer->header.frame_id = sensor_configuration_.frame_id;
  scan_buffer->header.stamp = scan_buffer->packets.front().stamp;
  robosense_scan_pub_->publish(*scan_buffer);
}

void RobosenseHwInterfaceRosWrapper::ReceiveInfoDataCallback(
  std::unique_ptr<robosense_msgs::msg::RobosenseInfoPacket> difop_buffer)
{
  // Publish
  robosense_difop_pub_->publish(*difop_buffer);
}

RCLCPP_COMPONENTS_REGISTER_NODE(RobosenseHwInterfaceRosWrapper)

}  // namespace ros
}  // namespace nebula