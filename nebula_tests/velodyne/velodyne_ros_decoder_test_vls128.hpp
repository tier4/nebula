#ifndef NEBULA_VelodyneRosDecoderTestVls128_H
#define NEBULA_VelodyneRosDecoderTestVls128_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/velodyne/velodyne_common.hpp"
#include "nebula_decoders/nebula_decoders_velodyne/velodyne_driver.hpp"
#include "nebula_ros/common/nebula_driver_ros_wrapper_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

#include <gtest/gtest.h>

namespace nebula
{
namespace ros
{
class VelodyneRosDecoderTest final : public rclcpp::Node,
                                     NebulaDriverRosWrapperBase  //, testing::Test
{
  std::shared_ptr<drivers::VelodyneDriver> driver_ptr_;
  Status wrapper_status_;

  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_cfg_ptr_;
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr_;

  Status InitializeDriver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) override;

  Status GetParameters(
    drivers::VelodyneSensorConfiguration & sensor_configuration,
    drivers::VelodyneCalibrationConfiguration & calibration_configuration);

  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

public:
  explicit VelodyneRosDecoderTest(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  void ReceiveScanMsgCallback(const velodyne_msgs::msg::VelodyneScan::SharedPtr scan_msg);
  Status GetStatus();
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

#endif  // NEBULA_VelodyneRosDecoderTestVls128_H
