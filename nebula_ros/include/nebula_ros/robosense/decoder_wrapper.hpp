#pragma once

#include "nebula_ros/common/watchdog_timer.hpp"

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/robosense/robosense_common.hpp>
#include <nebula_decoders/nebula_decoders_robosense/robosense_driver.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <robosense_msgs/msg/robosense_scan.hpp>

#include <chrono>

namespace nebula
{
namespace ros
{
/// @brief Ros wrapper of robosense driver
class RobosenseDecoderWrapper
{
public:
  explicit RobosenseDecoderWrapper(
    rclcpp::Node * const parent_node,
    const std::shared_ptr<nebula::drivers::RobosenseHwInterface> & hw_interface,
    const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & config,
    const std::shared_ptr<const nebula::drivers::RobosenseCalibrationConfiguration> & calibration);

  void ProcessCloudPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void OnConfigChange(
    const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & new_config);

  nebula::Status Status();

private:
  void PublishCloud(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  nebula::Status status_;
  rclcpp::Logger logger_;

  std::shared_ptr<nebula::drivers::RobosenseHwInterface> hw_interface_;
  std::shared_ptr<const drivers::RobosenseSensorConfiguration> sensor_cfg_;
  std::shared_ptr<const drivers::RobosenseCalibrationConfiguration> calibration_cfg_ptr_{};

  std::shared_ptr<drivers::RobosenseDriver> driver_ptr_;
  std::mutex mtx_driver_ptr_;

  rclcpp::Publisher<robosense_msgs::msg::RobosenseScan>::SharedPtr packets_pub_{};
  robosense_msgs::msg::RobosenseScan::UniquePtr current_scan_msg_{};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nebula_points_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_ex_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_base_pub_{};

  std::shared_ptr<WatchdogTimer> cloud_watchdog_;
};

}  // namespace ros
}  // namespace nebula
