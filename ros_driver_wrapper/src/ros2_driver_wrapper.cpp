#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <fstream>
#include <iostream>

#include "ros2_driver_wrapper.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_driver
{
/// @brief Constructor
  RosDriverWrapper::RosDriverWrapper(
      const rclcpp::NodeOptions & options, const std::string & node_name)
      : rclcpp::Node(node_name, options), driver_(), diagnostics_updater_(this)
  {
    // Set Params
    this->get_parameter("sensor_type", sensor_type);
    this->get_parameter("frame_id", frame_id);
    this->get_parameter("sensor_configuration", sensor_configuration_file_);
    this->get_parameter("cloud_configuration", cloud_configuration_file_);
    this->get_parameter("calibration_configuration", calibration_configuration_file_);

    main_thread_ = std::thread(&RosDriverWrapper::MainThread, this);
  }

/// @brief Destructor
  RosDriverWrapper::~RosDriverWrapper()
  {
    Shutdown();
    main_thread_.join();
  }

/// @brief Shutdown
  void RosDriverWrapper::Shutdown() { exit_signal_.set_value(); }

/// @brief StreamStart
  bool RosDriverWrapper::StreamStart()
  {
    driver_.StreamStart()
  }
  /// Instantiates the corresponding Lidar Driver for the selected sensor.

/// @brief StreamStop
  void RosDriverWrapper::StreamStop()
  {
    RCLCPP_INFO(this->get_logger(), "StreamStop request.");
    driver_.StopStream();  // stream stop request.
  }

/// @brief Main thread
  void RosDriverWrapper::MainThread()
  {
    bool running = false;
    /// Based on sensor_type, instantiate the correct driver and _hw_interface
    sensor_configuration_ = LoadSensorConfiguration(sensor_configuration_file_);
    driver_ = 0;    // TODO
    hw_interface_ = 0; // TODO

    /// Load configurations and check they are valid
    calibration_configuration_ = LoadCalibrationConfiguration(calibration_configuration_file_);
    cloud_configuration_ = LoadCloudConfiguration(cloud_configuration_file_);

    /// Set sensor configurations in the driver the uses them.
    driver_.SetCalibrationConfiguration(sensor_calibration_file_);
    driver_.SetCloudConfiguration(sensor_cloud_file_);

    /// If start failed, shutdown
    if (!running) {
      rclcpp::shutdown();
    } else {

      /// do something with threads here to get and publish pointcloud as they come
//      std::future_status status;
//      do {
//        status = future_.wait_for(std::chrono::seconds(3));
//      } while (status == std::future_status::timeout);

      DataInterfaceStart.StreamStop();
    return;
  }

/// @brief PublishCloud
  void RosDriverWrapper::PublishCloud()
  {
    /// get latest cloud from the driver
    sensor_msgs::msg::PointCloud2 cloud = driver_.cloud;

    /// set header properly

    /// publish
    cloud_pub_->publish(cloud);

    return;
  }

}  // namespace lidar_driver

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_driver::RosDriverWrapper)
