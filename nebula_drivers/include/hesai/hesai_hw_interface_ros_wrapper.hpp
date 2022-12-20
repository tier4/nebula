#ifndef NEBULA_HesaiHwInterfaceRosWrapper_H
#define NEBULA_HesaiHwInterfaceRosWrapper_H

#include "common/nebula_common.hpp"
#include "common/nebula_hw_interface_ros_wrapper_base.hpp"
#include "hesai/hesai_common.hpp"
#include "hesai/hesai_hw_interface.hpp"

//#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include "pandar_msgs/msg/pandar_jumbo_packet.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <boost/asio.hpp>
#include "tcp_driver/tcp_driver.hpp"
#include <mutex>

namespace nebula
{
namespace ros
{

template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

class HesaiHwInterfaceRosWrapper final : public rclcpp::Node, NebulaHwInterfaceWrapperBase
{
  drivers::HesaiHwInterface hw_interface_;
  Status interface_status_;

  drivers::HesaiSensorConfiguration sensor_configuration_;

  rclcpp::Publisher<pandar_msgs::msg::PandarScan>::SharedPtr pandar_scan_pub_;

  Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) override;
  void ReceiveScanDataCallback(std::unique_ptr<pandar_msgs::msg::PandarScan> scan_buffer);

public:
  explicit HesaiHwInterfaceRosWrapper(const rclcpp::NodeOptions & options);
  Status StreamStart() override;
  Status StreamStop() override;
  Status Shutdown() override;
  Status GetParameters(
    drivers::HesaiSensorConfiguration & sensor_configuration);

private:
//  std::shared_ptr<boost::asio::io_service> m_owned_ios;
//  std::unique_ptr<::drivers::tcp_driver::TcpDriver> m_tcp_driver;
  std::mutex mtx_config_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & parameters);
  std::vector<rcl_interfaces::msg::SetParametersResult> updateParameters();

};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_HesaiHwInterfaceRosWrapper_H