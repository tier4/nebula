#ifndef NEBULA_RobosenseRosOfflineExtractBag_H
#define NEBULA_RobosenseRosOfflineExtractBag_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/robosense_driver.hpp"
#include "nebula_ros/common/nebula_driver_ros_wrapper_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace nebula
{
namespace ros
{
class RobosenseRosOfflineExtractBag final : public rclcpp::Node, NebulaDriverRosWrapperBase
{
  std::shared_ptr<drivers::RobosenseDriver> driver_ptr_;
  Status wrapper_status_;

  std::shared_ptr<drivers::RobosenseCalibrationConfiguration> calibration_cfg_ptr_;
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr_;

  Status InitializeDriver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) override;

  Status GetParameters(
    drivers::RobosenseSensorConfiguration & sensor_configuration,
    drivers::RobosenseCalibrationConfiguration & calibration_configuration);

  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

public:
  explicit RobosenseRosOfflineExtractBag(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  Status GetStatus();
  Status ReadBag();

private:
  std::string bag_path;
  std::string storage_id;
  std::string out_path;
  std::string format;
  std::string target_topic;
  std::string correction_file_path;
  int out_num;
  int skip_num;
  bool only_xyz;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_RobosenseRosOfflineExtractBag_H
