#ifndef LIDAR_DRIVER_ROS2_DRIVER_WRAPPER_HPP_
#define LIDAR_DRIVER_ROS2_DRIVER_WRAPPER_HPP_

#include <future>
#include <memory>
#include <string>
//#include <functional>
//#include <thread>
#include <mutex>

//#include <diagnostic_updater/diagnostic_updater.hpp>
//#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
//#include <pcl_conversions/pcl_conversions.h>
#include <livox_msgs/msg/lidar_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "LidarDriver/lidar_driver.hpp"
#include "LidarDriver/livox_common.hpp"

// ToDo: livox only
using SensorConfiguration = livox_driver::LivoxSensorConfiguration;
using CloudConfiguration = livox_driver::LivoxCloudConfiguration;

namespace lidar_driver
{
/// @class RosDriverWrapper
class RosDriverWrapper final : public rclcpp::Node
{
public:  // Figure 9.
  explicit RosDriverWrapper(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "lidar_driver_node");
  ~RosDriverWrapper() override;

  RosDriverWrapper(RosDriverWrapper && c) = delete;
  RosDriverWrapper & operator=(RosDriverWrapper && c) = delete;
  RosDriverWrapper(const RosDriverWrapper & c) = delete;
  RosDriverWrapper & operator=(const RosDriverWrapper & c) = delete;

  bool StreamStart();
  void StreamStop();
  void Shutdown();

private:  // Figure 9.
  LidarDriver driver_;

  //rclcpp::Node * node_;

  void MainThread();
  std::thread main_thread_;

  // We use this future/promise pair to notify threads that we are shutting down
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

private:  // Command
  bool StartInterface();

private:                                                                  // pub/sub
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;           // IMU publish
  rclcpp::Publisher<livox_msgs::msg::LidarScan>::SharedPtr driver_scan_;  // scan data publish

  rclcpp::Subscription<livox_msgs::msg::LidarScan>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_;  // point cloud2 publish
  void CreatePubSub();

  //std::unique_ptr<velodyne_rawdata::DataContainerBase> container_ptr_;

  void RosLidarDataSubscriber(const livox_msgs::msg::LidarScan::SharedPtr scan_msg);
  void PublishCloud(int data_cnt);
  void PublishLidarData(
    const std::vector<uint8_t> & buff, int pkt_len, uint64_t timestamp, uint32_t point_num);
  void PublishImuPacket(const livox_driver::LivoxImuPoint & imu_pkt, uint64_t timestamp);

  void InitPointcloud2MsgHeader(sensor_msgs::msg::PointCloud2 & cloud);

private:  // Configuration
  // configuration parameters
  struct
  {
    std::string sensor_type;
    std::string frame_id;
    int debug_mode;
    std::string debug_str1;
    std::string debug_str2;
    int npackets;
  } config_;

  //SensorConfigEx sensor_configuration_;
  struct SensorConfigEx_
  {
    SensorConfiguration sensor_config;
    std::string sensor_model_str;
    std::string echo_mode_str;
    std::string coordinate_mode_str;
  };
  using SensorConfigEx = struct SensorConfigEx_;
  SensorConfigEx sensor_config_ex_;
  CloudConfiguration cloud_configuration_;

  bool CheckConfiguration(SensorConfigEx &, CloudConfiguration &);
  bool CheckSensorConfiguration(SensorConfigEx &);
  bool CheckOutputCloudConfiguration(SensorConfigEx &, CloudConfiguration &);
  void ConfigureSensor(SensorConfiguration &);
  //void SetCalibration(sensor_calibration);			// ToDo
  SensorConfiguration GetSensorConfig();

  void GetParameter();
  bool CheckIpAddress(const std::string & ip_addr);
  bool CheckPortNumber(int port) { return ((0 < port) && (port <= 0xFFFF)) ? true : false; }

  using LivoxSensorModelEx = std::tuple<livox_driver::LivoxSensorModel, std::string>;
  LivoxSensorModelEx GetParamLivoxSensorModel();
  using LivoxEchoModeEx = std::tuple<livox_driver::LivoxEchoMode, std::string>;
  LivoxEchoModeEx GetParamLivoxEchoMode();
  using LivoxCoordinateModeEx = std::tuple<livox_driver::LivoxCoordinateMode, std::string>;
  LivoxCoordinateModeEx GetParamLivoxCoordinateMode();

  void GetLivoxParameter(SensorConfigEx & ex, livox_driver::LivoxCloudConfiguration & cloud_config);
  bool CheckLivoxSensorConfiguration(SensorConfigEx & ex);
  bool CheckLivoxCloudConfiguration(
    SensorConfigEx & ex, livox_driver::LivoxCloudConfiguration & cloud_config);

#ifdef UTEST
public:
  friend class TestFriend_RosWrapper;
#endif  // UTEST
};
}  // namespace lidar_driver
#endif  //LIDAR_DRIVER_ROS2_DRIVER_WRAPPER_HPP_
