#ifndef NEBULA_DRIVER_WRAPPER_BASE_H
#define NEBULA_DRIVER_WRAPPER_BASE_H

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
class NebulaDriverRosWrapperBase
{
public:
  NebulaDriverRosWrapperBase() = default;

  NebulaDriverRosWrapperBase(NebulaDriverRosWrapperBase && c) = delete;
  NebulaDriverRosWrapperBase & operator=(NebulaDriverRosWrapperBase && c) = delete;
  NebulaDriverRosWrapperBase(const NebulaDriverRosWrapperBase & c) = delete;
  NebulaDriverRosWrapperBase & operator=(const NebulaDriverRosWrapperBase & c) = delete;

private:
  /// @brief Virtual function for initializing ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  /// @return Resulting status
  virtual Status InitializeDriver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) = 0;

  //  status ReceiveScanMsgCallback(void * ScanMsg);  // ROS message callback for individual packet
  //  type

  /// @brief Point cloud publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace ros
}  // namespace nebula
#endif  // NEBULA_DRIVER_WRAPPER_BASE_H
