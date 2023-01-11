#ifndef NEBULA_DRIVER_WRAPPER_BASE_H
#define NEBULA_DRIVER_WRAPPER_BASE_H

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"

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

private:
  virtual Status InitializeDriver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) = 0;

  //  status ReceiveScanMsgCallback(void * ScanMsg);  // ROS message callback for individual packet
  //  type
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace ros
}  // namespace nebula
#endif  // NEBULA_DRIVER_WRAPPER_BASE_H
