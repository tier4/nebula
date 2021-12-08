#include "hesai/hesai_hw_interface_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
HesaiHwInterfaceRosWrapper::HesaiHwInterfaceRosWrapper(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options), hw_interface_()
{
  interface_status_ = GetParameters(sensor_configuration_, calibration_configuration_, cloud_configuration_);
  if (Status::OK != interface_status_)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
    return;
  }
  // Initialize sensor_configuration
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration_);
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  // register scan callback and publisher
  hw_interface_.RegisterScanCallback(
    std::bind(&HesaiHwInterfaceRosWrapper::ReceiveScanDataCallback, this, std::placeholders::_1));
  pandar_scan_pub_ =
    this->create_publisher<pandar_msgs::msg::PandarScan>("pandar_packets", rclcpp::SensorDataQoS());
}

Status HesaiHwInterfaceRosWrapper::StreamStart()
{
  if(Status::OK == interface_status_ ){
    interface_status_ = hw_interface_.CloudInterfaceStart();
  }
  return interface_status_;
}

Status HesaiHwInterfaceRosWrapper::StreamStop() { return Status::OK; }
Status HesaiHwInterfaceRosWrapper::Shutdown() { return Status::OK; }

Status HesaiHwInterfaceRosWrapper::InitializeHwInterface(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  return Status::OK;
}

Status HesaiHwInterfaceRosWrapper::GetParameters(
  drivers::HesaiSensorConfiguration & sensor_configuration,
  drivers::HesaiCalibrationConfiguration & calibration_configuration,
  drivers::HesaiCloudConfiguration & cloud_configuration)
{
  sensor_configuration.sensor_model = nebula::drivers::SensorModelFromString(
    this->declare_parameter<std::string>("sensor_model", ""));

  sensor_configuration.return_mode =
    nebula::drivers::ReturnModeFromString(this->declare_parameter<std::string>("return_mode", ""));

  sensor_configuration.host_ip = this->declare_parameter<std::string>("host_ip", "255.255.255.255");
  sensor_configuration.sensor_ip =
    this->declare_parameter<std::string>("sensor_ip", "192.168.1.201");
  sensor_configuration.frame_id = this->declare_parameter<std::string>("frame_id", "pandar");
  sensor_configuration.data_port = this->declare_parameter<uint16_t>("data_port", 2368);
  sensor_configuration.gnss_port = this->declare_parameter<uint16_t>("gnss_port", 2369);
  sensor_configuration.scan_phase = this->declare_parameter<double>("scan_phase", 0.);
  sensor_configuration.frequency_ms = this->declare_parameter<uint16_t>("frequency_ms", 100);
  sensor_configuration.packet_mtu_size = this->declare_parameter<uint16_t>("packet_mtu_size", 1500);

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (
    sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 365 ||
    sensor_configuration.frequency_ms == 0) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void HesaiHwInterfaceRosWrapper::ReceiveScanDataCallback(
  std::unique_ptr<pandar_msgs::msg::PandarScan> scan_buffer)
{
  // Publish
  scan_buffer->header.frame_id = sensor_configuration_.frame_id;
  scan_buffer->header.stamp = scan_buffer->packets.front().stamp;
  pandar_scan_pub_->publish(*scan_buffer);
}

}  // namespace ros
}  // namespace nebula
