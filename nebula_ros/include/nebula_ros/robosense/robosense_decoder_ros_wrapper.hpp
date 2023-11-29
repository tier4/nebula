#ifndef NEBULA_RobosenseDriverRosWrapper_H
#define NEBULA_RobosenseDriverRosWrapper_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/robosense_driver.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp"
#include "nebula_ros/common/nebula_driver_ros_wrapper_base.hpp"
#include "time.h"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "robosense_msgs/msg/robosense_bp_ruby_monitor_info.hpp"
#include "robosense_msgs/msg/robosense_flash_monitor_info.hpp"
#include "robosense_msgs/msg/robosense_helios_monitor_info.hpp"
#include "robosense_msgs/msg/robosense_mems_monitor_info.hpp"
#include "robosense_msgs/msg/robosense_packet.hpp"
#include "robosense_msgs/msg/robosense_scan.hpp"
namespace nebula
{
namespace ros
{
/// @brief Ros wrapper of robosense driver
class RobosenseDriverRosWrapper final : public rclcpp::Node, NebulaDriverRosWrapperBase
{
  std::shared_ptr<drivers::RobosenseDriver> driver_ptr_;
  Status wrapper_status_;
  rclcpp::Subscription<robosense_msgs::msg::RobosenseScan>::SharedPtr robosense_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nebula_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_ex_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_base_pub_;

  std::shared_ptr<drivers::RobosenseCalibrationConfiguration> calibration_cfg_ptr_;
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr_;

  drivers::RobosenseHwInterface hw_interface_;

  /// @brief Initializing ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  /// @return Resulting status
  Status InitializeDriver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) override;

  /// @brief Get configurations from ros parameters
  /// @param sensor_configuration Output of SensorConfiguration
  /// @param calibration_configuration Output of CalibrationConfiguration
  /// @return Resulting status
  Status GetParameters(
    drivers::RobosenseSensorConfiguration & sensor_configuration,
    drivers::RobosenseCalibrationConfiguration & calibration_configuration);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  /***
   * Publishes a sensor_msgs::msg::PointCloud2 to the specified publisher
   * @param pointcloud unique pointer containing the point cloud to publish
   * @param publisher
   */
  void PublishCloud(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher);

public:
  explicit RobosenseDriverRosWrapper(const rclcpp::NodeOptions & options);

  /// @brief Callback for RobosenseScan subscriber
  /// @param scan_msg Received RobosenseScan
  void ReceiveScanMsgCallback(const robosense_msgs::msg::RobosenseScan::SharedPtr scan_msg);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

private:
  /// @brief File path of Correction data (Only required only for AT)
  std::string correction_file_path;

  std::function<void(const drivers::AllDeviceInfo &)> difop_callback_;
  // RobosenseSensorConfiguration
  drivers::RobosenseSensorConfiguration sensor_configuration;

  rclcpp::Publisher<robosense_msgs::msg::RobosenseBpRubyMonitorInfo>::SharedPtr
    robosense_bp_ruby_monitor_info_pub_{nullptr};
  rclcpp::Publisher<robosense_msgs::msg::RobosenseHeliosMonitorInfo>::SharedPtr
    robosense_helios_monitor_info_pub_{nullptr};
  rclcpp::Publisher<robosense_msgs::msg::RobosenseMemsMonitorInfo>::SharedPtr
    robosense_mems_monitor_info_pub_{nullptr};
  rclcpp::Publisher<robosense_msgs::msg::RobosenseFlashMonitorInfo>::SharedPtr
    robosense_flash_monitor_info_pub_{nullptr};

  int lidar_type{0};  // 0:helios 1:bp or rubyplus 2:mems 3:e1
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_RobosenseDriverRosWrapper_H
