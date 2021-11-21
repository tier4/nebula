#include "hesai/hesai_hw_interface_ros_wrapper.hpp"

#include "../../../../../build/pandar_msgs/rosidl_generator_cpp/pandar_msgs/msg/detail/pandar_scan__struct.hpp"

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
  sensor_configuration_.frequency_ms =
    frequency_ms_;  // todo: this is weird, frequency_hz is better maybe?
  // Echo mode and model will need some type of switch/case or if/else
  sensor_configuration_.echo_mode = drivers::ReturnMode::SINGLE_STRONGEST;
  sensor_configuration_.sensor_model = drivers::SensorModel::HESAI_PANDAR64;
  sensor_configuration_.frame_id = "hesai_pandar";  // maybe we need it?

  // Initialize sensor_configuration
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration_);
  std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr);
  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  // register scan callback
  hw_interface_.RegisterScanCallback(
    std::bind(&HesaiHwInterfaceWrapper::ReceiveScanDataCallback, this, std::placeholders::_1));
}

Status HesaiHwInterfaceWrapper::StreamStart()
{
  hw_interface_.CloudInterfaceStart();
  return Status::OK;
}

Status HesaiHwInterfaceWrapper::StreamStop() { return Status::OK; }
Status HesaiHwInterfaceWrapper::Shutdown() { return Status::OK; }

Status HesaiHwInterfaceWrapper::InitializeHwInterface(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  return Status::OK;
}

void HesaiHwInterfaceWrapper::ReceiveScanDataCallback(
  std::unique_ptr<std::vector<std::vector<uint8_t>>> scan_buffer)
{
  // Publish
  pandar_msgs::msg::PandarScan scan;
  scan.header.frame_id = sensor_configuration_.frame_id;
  std::chrono::duration<float> now = std::chrono::system_clock::now().time_since_epoch();
  scan.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
  scan.header.stamp.nanosec =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now)
      .count();  // TODO: not sure this will work, probably need to remove the secs part
  // scan.packets;
  //  move buffer to scan
}

}  // namespace ros
}  // namespace nebula
