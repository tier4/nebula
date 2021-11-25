#include "hesai/hesai_hw_interface_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
HesaiHwInterfaceRosWrapper::HesaiHwInterfaceRosWrapper(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options), hw_interface_()
{
  // Get these params form YAML or XML file
  sensor_model_ = "Pandar64";
  host_ip_ = "255.255.255.255";
  sensor_ip_ = "192.168.1.201";
  data_port_ = 2368;
  gnss_port_ = 0;
  frequency_ms_ = 100;
  echo_mode_ = "Dual";

  // Build sensor_configuration file
  sensor_configuration_.sensor_ip = sensor_ip_;
  sensor_configuration_.host_ip = host_ip_;
  sensor_configuration_.data_port = data_port_;
  sensor_configuration_.gnss_port = gnss_port_;
  sensor_configuration_.frequency_ms =
    frequency_ms_;  // todo: this is weird, frequency_hz is better maybe?
  // Echo mode and model will need some type of switch/case or if/else
  sensor_configuration_.echo_mode = drivers::ReturnMode::SINGLE_STRONGEST;
  sensor_configuration_.sensor_model = drivers::SensorModel::HESAI_PANDAR64;
  sensor_configuration_.frame_id = "hesai_pandar";  // maybe we need it

  // Initialize sensor_configuration
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration_);
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  // register scan callback and publisher
  hw_interface_.RegisterScanCallback(
    std::bind(&HesaiHwInterfaceRosWrapper::ReceiveScanDataCallback, this, std::placeholders::_1));
  pandar_scan_pub_ = this->create_publisher<pandar_msgs::msg::PandarScan>("pandar_packets", rclcpp::SensorDataQoS());
}

Status HesaiHwInterfaceRosWrapper::StreamStart()
{
  hw_interface_.CloudInterfaceStart();
  return Status::OK;
}

Status HesaiHwInterfaceRosWrapper::StreamStop() { return Status::OK; }
Status HesaiHwInterfaceRosWrapper::Shutdown() { return Status::OK; }

Status HesaiHwInterfaceRosWrapper::InitializeHwInterface(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  return Status::OK;
}

void HesaiHwInterfaceRosWrapper::ReceiveScanDataCallback(
  std::unique_ptr<pandar_msgs::msg::PandarScan> scan_buffer)
{
  // Publish
  scan_buffer->header.frame_id = sensor_configuration_.frame_id;
  pandar_scan_pub_->publish(*scan_buffer);
}

}  // namespace ros
}  // namespace nebula
