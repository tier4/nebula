#pragma once

#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/common/watchdog_timer.hpp"

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/seyond/seyond_common.hpp>
#include <nebula_decoders/nebula_decoders_seyond/seyond_driver.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_seyond/seyond_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>

#include <filesystem>
#include <memory>
#include <mutex>

namespace nebula
{
namespace ros
{
using SeyondDriver = nebula::drivers::SeyondDriver;
using SeyondHwInterface = nebula::drivers::SeyondHwInterface;
using SeyondSensorConfiguration = nebula::drivers::SeyondSensorConfiguration;
using SeyondCalibrationConfiguration = nebula::drivers::SeyondCalibrationConfiguration;
using get_calibration_result_t =
  nebula::util::expected<std::shared_ptr<SeyondCalibrationConfiguration>, nebula::Status>;

class SeyondDecoderWrapper
{
public:
  SeyondDecoderWrapper(
    rclcpp::Node * const parent_node, const std::shared_ptr<SeyondHwInterface> & hw_interface,
    std::shared_ptr<const SeyondSensorConfiguration> & config);

  void ProcessCloudPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void OnConfigChange(const std::shared_ptr<const SeyondSensorConfiguration> & new_config);

  nebula::Status Status();

private:
  void PublishCloud(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher);

  nebula::Status status_;
  rclcpp::Logger logger_;

  const std::shared_ptr<SeyondHwInterface> hw_interface_;
  std::shared_ptr<const SeyondSensorConfiguration> sensor_cfg_;

  std::string calibration_file_path_{};
  std::shared_ptr<const SeyondCalibrationConfiguration> calibration_cfg_ptr_{};

  std::shared_ptr<SeyondDriver> driver_ptr_{};
  std::mutex mtx_driver_ptr_;

  rclcpp::Publisher<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_pub_{};
  nebula_msgs::msg::NebulaPackets::UniquePtr current_scan_msg_{};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nebula_points_pub_{};

  std::shared_ptr<WatchdogTimer> cloud_watchdog_;
};
}  // namespace ros
}  // namespace nebula
