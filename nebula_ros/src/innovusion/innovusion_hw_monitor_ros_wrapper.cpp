#include "nebula_ros/innovusion/innovusion_hw_monitor_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
InnovusionHwMonitorRosWrapper::InnovusionHwMonitorRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("Innovusion_hw_monitor_ros_wrapper", options)
{
}

Status InnovusionHwMonitorRosWrapper::MonitorStart() { return interface_status_; }

Status InnovusionHwMonitorRosWrapper::MonitorStop() { return Status::OK; }
Status InnovusionHwMonitorRosWrapper::Shutdown() { return Status::OK; }

Status InnovusionHwMonitorRosWrapper::InitializeHwMonitor(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  return Status::OK;
}

  InnovusionHwMonitorRosWrapper::~InnovusionHwMonitorRosWrapper() {
    RCLCPP_INFO_STREAM(get_logger(), "Closing TcpDriver");
  }

  RCLCPP_COMPONENTS_REGISTER_NODE(InnovusionHwMonitorRosWrapper)
}  // namespace ros
}  // namespace nebula
