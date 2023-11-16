#ifndef NEBULA_VelodyneRosDecoderTestVlp16_H
#define NEBULA_VelodyneRosDecoderTestVlp16_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/velodyne/velodyne_common.hpp"
#include "nebula_decoders/nebula_decoders_velodyne/velodyne_driver.hpp"
#include "nebula_ros/common/nebula_driver_ros_wrapper_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <gtest/gtest.h>

namespace nebula
{
namespace ros
{
/// @brief Testing decoder of VLP16 (Keeps VelodyneDriverRosWrapper structure as much as possible)
class VelodyneRosDecoderTest final : public rclcpp::Node,
                                     NebulaDriverRosWrapperBase  //, testing::Test
{
  std::shared_ptr<drivers::VelodyneDriver> driver_ptr_;
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

  /// @brief Get configurations (Magic numbers for VLP16 is described, each function can be
  /// integrated if the ros parameter can be passed to Google Test)
  /// @param sensor_configuration Output of SensorConfiguration
  /// @param calibration_configuration Output of CalibrationConfiguration
  /// @return Resulting status
  Status GetParameters(
    drivers::VelodyneSensorConfiguration & sensor_configuration,
    drivers::VelodyneCalibrationConfiguration & calibration_configuration);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

public:
  explicit VelodyneRosDecoderTest(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Read the specified bag file and compare the constructed point clouds with the
  /// corresponding PCD files
  void ReadBag();
  /*
  void SetUp() override {
    // Setup things that should occur before every test instance should go here
    RCLCPP_ERROR_STREAM(this->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }
*/
private:
  std::string bag_path;
  std::string storage_id;
  std::string format;
  std::string target_topic;
  std::string correction_file_path;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_VelodyneRosDecoderTestVlp16_H
