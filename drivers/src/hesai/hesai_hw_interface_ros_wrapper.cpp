#include "hesai/hesai_hw_interface_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{

HesaiHwInterfaceWrapper::HesaiHwInterfaceWrapper(const rclcpp::NodeOptions & options, const std::string & node_name):
  rclcpp::Node(node_name, options)
{
  drivers::HesaiSensorConfiguration sensor_configuration;
  sensor_configuration.sensor_ip = "127.0.0.1";
  sensor_configuration.data_port = 3360;
  sensor_configuration.gnss_port = 3361;
  drivers::HesaiHwInterface hw_interface;
  std::shared_ptr<drivers::SensorConfigurationBase> a
    = std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration);
  hw_interface.SetSensorConfiguration(std::static_pointer_cast<drivers::SensorConfigurationBase>(a));
  hw_interface.CloudInterfaceStart();
}

Status HesaiHwInterfaceWrapper::StreamStart() { return Status::OK; }
Status HesaiHwInterfaceWrapper::StreamStop() { return Status::OK; }
Status HesaiHwInterfaceWrapper::Shutdown() { return Status::OK; }

Status HesaiHwInterfaceWrapper::InitializeHwInterface(
  const drivers::SensorConfigurationBase & sensor_configuration)
{

}
void HesaiHwInterfaceWrapper::ReceiveDataPacketCallback(const std::vector<uint8_t> & buffer) {

}

}  //ros
}  //nebula