#ifndef NEBULA_HW_INTERFACE_WRAPPER_BASE_H
#define NEBULA_HW_INTERFACE_WRAPPER_BASE_H

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "configuration_base.hpp"
#include "status.hpp"

namespace nebula
{
namespace ros {

class NebulaHwInterfaceWrapperBase {
public:
  NebulaHwInterfaceWrapperBase(const NebulaHwInterfaceWrapperBase&) = delete;
  NebulaHwInterfaceWrapperBase & operator=(const NebulaHwInterfaceWrapperBase&) = delete;

  NebulaHwInterfaceWrapperBase(const rclcpp::NodeOptions & options, const std::string & node_name)
      : rclcpp::Node(node_name, options), driver_(), diagnostics_updater_(this);

  virtual STATUS Start();
  virtual STATUS Stop();

private:
  SensorConfigurationBase sensor_configuration;
  virtual STATUS InitializeHwInterface(const drivers::SensorConfigurationBase &sensor_configuration);
  void ReceiveDataPacketCallback(const std::vector<uint8_t> &buffer);     // TODO: Can we replace with unique_ptr? or shared?
//  void SendDataPacket(const std::vector<uint8_t> &buffer);        // Ideally this will be implemented as specific funtions, GetFanStatus, GetEchoMode
};

} // ros namespace
} // nebula namespace


#endif //NEBULA_HW_INTERFACE_WRAPPER_BASE_H
