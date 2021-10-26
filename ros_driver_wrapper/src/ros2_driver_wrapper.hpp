#ifndef NEBULA_DRIVER_ROS2_DRIVER_WRAPPER_HPP
#define NEBULA_DRIVER_ROS2_DRIVER_WRAPPER_HPP
#include <future>
#include <memory>
#include <string>
#include <mutex>

#include <sensor_msgs/msg/point_cloud2.hpp>
/// Include base classes?

/// @class RosDriverWrapper
namespace lidar_driver
{
class RosDriverWrapper final : public rclcpp::Node
{
public:
  explicit RosDriverWrapper(
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
      const std::string & node_name = "lidar_driver_node");
  ~RosDriverWrapper() override;

  RosDriverWrapper(RosDriverWrapper && c) = delete;
  RosDriverWrapper & operator=(RosDriverWrapper && c) = delete;
  RosDriverWrapper(const RosDriverWrapper & c) = delete;
  RosDriverWrapper & operator=(const RosDriverWrapper & c) = delete;

  bool StreamStart();
  bool StreamStop();
  void Shutdown();

private:
  // MAIN THREAD
  void MainThread();
  std::thread main_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

  // LIDAR DRIVER
  LidarDriver driver_;
  RosHwInterface hw_interface_;

  // CONFIGURATION
  std::string calibration_configuration_file_;
  CalibrationConfiguration calibration_configuration_;
  std::string sensor_configuration_file_;
  SensorConfiguration sensor_configuration_;
  std::string cloud_configuration_file_;
  CloudConfiguration cloud_configuration_;

  // Load and validate calibration
  CalibrationConfiguration LoadCalibrationConfiguration(std::string calibration_configuration_file);
  SensorConfiguration LoadSensorConfiguration(SensorConfiguration sensor_configuration_file_);
  CloudConfiguration LoadCloudConfiguration(CloudConfiguration cloud_configuration_file_);

  // Verify configuration match, or just print current calibration

  bool CheckCalibrationConfiguration(CalibrationConfiguration calibration_configuration);
  bool GetCalibrationConfiguration();
  bool CheckSensorConfiguration(SensorConfiguration sensor_configuration);
  bool GetSensorConfiguration();
  bool CheckCloudConfiguration(CloudConfiguration cloud_configuration);
  bool GetCloudConfiguration();

  // POINT CLOUD PUBLISHING
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  void PublishCloud();
};
} // namespace lidar_driver

#endif //NEBULA_DRIVER_ROS2_DRIVER_WRAPPER_HPP
