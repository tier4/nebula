#ifndef NEBULA_HesaiDriverRosWrapper_H
#define NEBULA_HesaiDriverRosWrapper_H

#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"
#include "common/nebula_driver_ros_wrapper_base.hpp"
#include "hesai/hesai_common.hpp"
#include "hesai/hesai_driver.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>


namespace nebula
{
namespace ros
{

class HesaiDriverRosWrapper final : public rclcpp::Node, NebulaDriverRosWrapperBase
{
  drivers::HesaiDriver driver_;
  rclcpp::Subscription<pandar_msgs::msg::PandarScan>::SharedPtr pandar_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pandar_points_pub_;

private:
  Status InitializeDriver(
    std::shared_ptr<drivers::CloudConfigurationBase> cloud_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) override;

public:
  explicit HesaiDriverRosWrapper(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  Status StreamStart() override;
  Status StreamStop() override;
  Status Shutdown() override;
  Status ReceiveScanMsgCallback(pandar_msgs::msg::PandarScan::SharedPtr scan_msg);
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_HesaiDriverRosWrapper_H
