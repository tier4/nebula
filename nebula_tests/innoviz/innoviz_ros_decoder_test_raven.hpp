#ifndef NEBULA_HesaiRosDecoderTest64_H
#define NEBULA_HesaiRosDecoderTest64_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_decoders/nebula_decoders_innoviz/innoviz_driver.hpp"
#include "nebula_ros/common/nebula_driver_ros_wrapper_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "innoviz_msgs/msg/innoviz_packet.hpp"
#include "innoviz_msgs/msg/innoviz_scan.hpp"

#include <gtest/gtest.h>

namespace nebula
{
namespace ros
{
/// @brief Testing Innoviz decoder
class InnovizRosDecoderTest final : public rclcpp::Node, NebulaDriverRosWrapperBase  //, testing::Test
{
    std::shared_ptr<drivers::InnovizDriver> driver_ptr_;
    Status wrapper_status_;

    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_cfg_ptr_;
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr_;

  /// @brief Initializing ros wrapper
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  /// @return Resulting status
  Status InitializeDriver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) override;


  /// @brief Get configurations
  /// @param sensor_configuration Output of SensorConfiguration
  /// @return Resulting status
  Status GetParameters(drivers::InnovizSensorConfiguration & sensor_configuration);


public:
  InnovizRosDecoderTest(const rclcpp::NodeOptions & options, const std::string & node_name);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Read the specified bag file and compare the constructed point clouds with the
  /// corresponding PCD files
  void ReadBag();

private:
  std::string bag_path;
  std::string storage_id;
  std::string format;
  std::string target_topic;
  std::string correction_file_path;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_HesaiRosDecoderTest64_H
