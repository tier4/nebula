#ifndef NEBULA_InnovusionHwMonitorRosWrapper_H
#define NEBULA_InnovusionHwMonitorRosWrapper_H

#include "nebula_common/innovusion/innovusion_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_innovusion/innovusion_hw_interface.hpp"
#include "nebula_ros/common/nebula_hw_monitor_ros_wrapper_base.hpp"
#include "boost_tcp_driver/tcp_driver.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <boost/asio.hpp>

#include <mutex>
#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

#include <thread>

namespace nebula
{
namespace ros
{
/// @brief Get parameter from rclcpp::Parameter
/// @tparam T
/// @param p Parameter from rclcpp parameter callback
/// @param name Target parameter name
/// @param value Corresponding value
/// @return Whether the target name existed
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

/// @brief Hardware monitor ros wrapper of Innovusion driver
class InnovusionHwMonitorRosWrapper final : public rclcpp::Node, NebulaHwMonitorWrapperBase
{
  drivers::InnovusionHwInterface hw_interface_;
  Status interface_status_;

  drivers::InnovusionSensorConfiguration sensor_configuration_;

  /// @brief Initializing hardware monitor ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  Status InitializeHwMonitor(
    const drivers::SensorConfigurationBase & sensor_configuration) override;

public:
  explicit InnovusionHwMonitorRosWrapper(const rclcpp::NodeOptions & options);
  ~InnovusionHwMonitorRosWrapper() noexcept override;
  /// @brief Not used
  /// @return Current status
  Status MonitorStart() override;
  /// @brief Not used
  /// @return Status::OK
  Status MonitorStop() override;
  /// @brief Not used
  /// @return Status::OK
  Status Shutdown() override;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_InnovusionHwMonitorRosWrapper_H
