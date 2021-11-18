#include "hesai/hesai_hw_interface_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
HesaiHwInterfaceWrapper::HesaiHwInterfaceWrapper(
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
  sensor_configuration_.frequency_ms = frequency_ms_;   // todo: this is weird, frequency_hz is better maybe?
  // Echo mode and model will need some type of switch/case or if/else
  sensor_configuration_.echo_mode = drivers::EchoMode::SINGLE_STRONGEST;
  sensor_configuration_.sensor_model = drivers::SensorModel::PANDAR64;

  // Initialize sensor_configuration
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration_);
  std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr);
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
}

Status HesaiHwInterfaceWrapper::StreamStart() {
hw_interface_.CloudInterfaceStart();
  return Status::OK; }

Status HesaiHwInterfaceWrapper::StreamStop() { return Status::OK; }
Status HesaiHwInterfaceWrapper::Shutdown() { return Status::OK; }

Status HesaiHwInterfaceWrapper::InitializeHwInterface(      // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration) {
  return Status::OK;
}

void HesaiHwInterfaceWrapper::ReceiveDataPacketCallback(const std::vector<uint8_t> & buffer) {
  // Todo: how do I wrap this in a callback for cur_pkt_?
  // todo: how to bring msgs in hesai_hw_interface_ros_wrapper.hpp?

  // Create new PandarPacket
  // Add current packet, fix header
  // Publish

  // Check if beginning of scan (somehow must be based on azimuth angle? or total size, but I still
  // to check and start at the right place...)
    // Create new hesai_msgs::PandarScan

  // Add packet to scan

  // Check if scan is complete
    // Publish
}

}  // namespace ros
}  // namespace nebula
