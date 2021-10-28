#ifndef NEBULA_DRIVER_WRAPPER_BASE_H
#define NEBULA_DRIVER_WRAPPER_BASE_H

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include "nebula_common.hpp"
#include "nebula_status.hpp"

namespace nebula
{
namespace ros
{
class NebulaDriverWrapperBase final : public rclcpp::Node
{
public:
  explicit NebulaDriverWrapperBase(
    const rclcpp::NodeOptions & options, const std::string & node_name)
  : rclcpp::Node(node_name, options), driver_(), diagnostics_updater_(this);
  ~NebulaDriverWrapperBase() override;

  NebulaDriverWrapperBase(NebulaDriverWrapperBase && c) = delete;
  NebulaDriverWrapperBase & operator=(NebulaDriverWrapperBase && c) = delete;
  NebulaDriverWrapperBase(const NebulaDriverWrapperBase & c) = delete;
  NebulaDriverWrapperBase & operator=(const NebulaDriverWrapperBase & c) = delete;

  virtual Status StreamStart();  // Activate callback function
  virtual Status StreamStop();   // Deactivate callback function
  virtual Status Shutdown();     // Shutdown the driver

private:
  drivers::CloudConfigurationBase cloud_configuration_;
  drivers::CalibrationConfigurationBase calibration_configuration_;

  virtual STATUS InitializeDriver(
    const drivers::CloudConfigurationBase & cloud_configuration,
    const drivers::CalibrationConfigurationBase & calibration_configuration);

  CloudPacketCallback(void * PacketMsg);  // ROS message callback for individual packet type
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace ros
}  // namespace nebula
#endif  //NEBULA_DRIVER_WRAPPER_BASE_H
