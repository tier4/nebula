#ifndef NEBULA_HesaiDriverRosWrapper_H
#define NEBULA_HesaiDriverRosWrapper_H

#include "common/nebula_common.hpp"
#include "common/nebula_driver_ros_wrapper_base.hpp"
#include "common/nebula_status.hpp"
#include "hesai/hesai_common.hpp"
#include "hesai/hesai_driver.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace ros
{
class HesaiDriverRosWrapper final : public rclcpp::Node, NebulaDriverRosWrapperBase
{
  std::shared_ptr<drivers::HesaiDriver> driver_ptr_;
  Status wrapper_status_;
  rclcpp::Subscription<pandar_msgs::msg::PandarScan>::SharedPtr pandar_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pandar_points_pub_;

  std::shared_ptr<drivers::HesaiCalibrationConfiguration> calibration_cfg_ptr_;
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr_;

  Status InitializeDriver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) override;

  Status GetParameters(
    drivers::HesaiSensorConfiguration & sensor_configuration,
    drivers::HesaiCalibrationConfiguration & calibration_configuration);

  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

public:
  explicit HesaiDriverRosWrapper(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  void ReceiveScanMsgCallback(const pandar_msgs::msg::PandarScan::SharedPtr scan_msg);
  Status GetStatus();
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_HesaiDriverRosWrapper_H
