#ifndef NEBULA_DRIVER_WRAPPER_BASE_H
#define NEBULA_DRIVER_WRAPPER_BASE_H

#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"

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
class NebulaDriverRosWrapperBase
{

public:
  NebulaDriverRosWrapperBase() = default;

  NebulaDriverRosWrapperBase(NebulaDriverRosWrapperBase && c) = delete;
  NebulaDriverRosWrapperBase & operator=(NebulaDriverRosWrapperBase && c) = delete;
  NebulaDriverRosWrapperBase(const NebulaDriverRosWrapperBase & c) = delete;
  NebulaDriverRosWrapperBase & operator=(const NebulaDriverRosWrapperBase & c) = delete;

  virtual Status StreamStart() = 0;  // Activate callback function
  virtual Status StreamStop() = 0;   // Deactivate callback function
  virtual Status Shutdown() = 0;     // Shutdown the driver

private:
  virtual Status InitializeDriver(
    std::shared_ptr<drivers::CloudConfigurationBase> cloud_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) = 0;

//  status ReceiveScanMsgCallback(void * ScanMsg);  // ROS message callback for individual packet type
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace ros
}  // namespace nebula
#endif  // NEBULA_DRIVER_WRAPPER_BASE_H
