#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>
#include <vector>

namespace nebula
{
namespace ros
{
/// @brief Base class for ros wrapper of each sensor driver
class NebulaRosWrapperBase
{
public:
  NebulaRosWrapperBase() = default;

  NebulaRosWrapperBase(NebulaRosWrapperBase && c) = delete;
  NebulaRosWrapperBase & operator=(NebulaRosWrapperBase && c) = delete;
  NebulaRosWrapperBase(const NebulaRosWrapperBase & c) = delete;
  NebulaRosWrapperBase & operator=(const NebulaRosWrapperBase & c) = delete;

  /// @brief Start point cloud streaming (Call CloudInterfaceStart of HwInterface)
  /// @return Resulting status
  virtual Status StreamStart() = 0;

  /// @brief Stop point cloud streaming (not used)
  /// @return Resulting status
  virtual Status StreamStop() = 0;

  /// @brief Shutdown (not used)
  /// @return Resulting status
  virtual Status Shutdown() = 0;

private:
  /// @brief Virtual function for initializing hardware interface ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  virtual Status InitializeHwInterface(
    const drivers::SensorConfigurationBase & sensor_configuration) = 0;

  /// @brief Virtual function for initializing hardware monitor ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @return Resulting status
  virtual Status InitializeHwMonitor(
    const drivers::SensorConfigurationBase & sensor_configuration) = 0;

  /// @brief Virtual function for initializing ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  /// @return Resulting status
  virtual Status InitializeCloudDriver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) = 0;

  /// @brief Point cloud publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace ros
}  // namespace nebula
