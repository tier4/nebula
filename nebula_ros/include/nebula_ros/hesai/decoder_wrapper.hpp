#pragma once

#include "nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp"

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nebula_msgs/msg/nebula_packet.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <memory>
#include <filesystem>

namespace nebula
{
namespace ros
{
class HesaiDecoderWrapper
{
public:
  HesaiDecoderWrapper(rclcpp::Node* const parent_node,
                      const std::shared_ptr<nebula::drivers::HesaiHwInterface>& hw_interface,
                      std::shared_ptr<nebula::drivers::HesaiSensorConfiguration>& config);

  nebula::util::expected<std::shared_ptr<drivers::HesaiCalibrationConfigurationBase>, nebula::Status>
  GetCalibrationData();

  void ProcessCloudPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void PublishCloud(std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
                    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);

  nebula::Status Status();

private:
  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds));
  }

  nebula::Status status_;
  rclcpp::Logger logger_;

  const std::shared_ptr<nebula::drivers::HesaiHwInterface> hw_interface_;
  std::shared_ptr<nebula::drivers::HesaiSensorConfiguration> sensor_cfg_;

  std::string calibration_file_path_{};
  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_cfg_ptr_{};

  std::shared_ptr<drivers::HesaiDriver> driver_ptr_;

  rclcpp::Publisher<pandar_msgs::msg::PandarScan>::SharedPtr packets_pub_{};
  pandar_msgs::msg::PandarScan::UniquePtr current_scan_msg_{};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nebula_points_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_ex_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_base_pub_{};
};
}  // namespace ros
}  // namespace nebula