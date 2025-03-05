// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/robosense/decoder_wrapper.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <tuple>
#include <utility>

namespace nebula::ros
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

RobosenseDecoderWrapper::RobosenseDecoderWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<nebula::drivers::RobosenseHwInterface> & hw_interface,
  const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & config,
  const std::shared_ptr<const nebula::drivers::RobosenseCalibrationConfiguration> & calibration)
: status_(nebula::Status::NOT_INITIALIZED),
  logger_(parent_node->get_logger().get_child("DecoderWrapper")),
  hw_interface_(hw_interface),
  sensor_cfg_(config),
  calibration_cfg_ptr_(calibration),
  driver_ptr_(new drivers::RobosenseDriver(config, calibration))
{
  status_ = driver_ptr_->get_status();

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  RCLCPP_INFO(logger_, ". Starting...");
  RCLCPP_INFO(logger_, ". Driver ");

  // Publish packets only if HW interface is connected
  if (hw_interface_) {
    current_scan_msg_ = std::make_unique<robosense_msgs::msg::RobosenseScan>();
    packets_pub_ = parent_node->create_publisher<robosense_msgs::msg::RobosenseScan>(
      "robosense_packets", rclcpp::SensorDataQoS());
  }

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  nebula_points_pub_ = parent_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "robosense_points", pointcloud_qos);
  aw_points_base_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points", pointcloud_qos);
  aw_points_ex_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points_ex", pointcloud_qos);

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);

  cloud_watchdog_ =
    std::make_shared<WatchdogTimer>(*parent_node, 100'000us, [this, parent_node](bool ok) {
      if (ok) return;
      RCLCPP_WARN_THROTTLE(
        logger_, *parent_node->get_clock(), 5000, "Missed pointcloud output deadline");
    });

  RCLCPP_INFO(logger_, "Initialized decoder wrapper.");
}

void RobosenseDecoderWrapper::process_cloud_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  // Accumulate packets for recording only if someone is subscribed to the topic (for performance)
  if (
    hw_interface_ && (packets_pub_->get_subscription_count() > 0 ||
                      packets_pub_->get_intra_process_subscription_count() > 0)) {
    if (current_scan_msg_->packets.size() == 0) {
      current_scan_msg_->header.stamp = packet_msg->stamp;
    }

    robosense_msgs::msg::RobosensePacket robosense_packet_msg{};
    robosense_packet_msg.stamp = packet_msg->stamp;
    std::copy(packet_msg->data.begin(), packet_msg->data.end(), robosense_packet_msg.data.begin());
    current_scan_msg_->packets.emplace_back(std::move(robosense_packet_msg));
  }

  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts{};
  nebula::drivers::NebulaPointCloudPtr pointcloud = nullptr;

  {
    std::lock_guard lock(mtx_driver_ptr_);
    pointcloud_ts = driver_ptr_->parse_cloud_packet(packet_msg->data);
    pointcloud = std::get<0>(pointcloud_ts);
  }

  if (pointcloud == nullptr) {
    return;
  }

  cloud_watchdog_->update();

  // Publish scan message only if it has been written to
  if (current_scan_msg_ && !current_scan_msg_->packets.empty()) {
    packets_pub_->publish(std::move(current_scan_msg_));
    current_scan_msg_ = std::make_unique<robosense_msgs::msg::RobosenseScan>();
  }

  if (
    nebula_points_pub_->get_subscription_count() > 0 ||
    nebula_points_pub_->get_intra_process_subscription_count() > 0) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(seconds_to_chrono_nano_seconds(std::get<1>(pointcloud_ts)).count());
    publish_cloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
  if (
    aw_points_base_pub_->get_subscription_count() > 0 ||
    aw_points_base_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_cloud_xyzi =
      nebula::drivers::convert_point_xyzircaedt_to_point_xyzir(pointcloud);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_cloud_xyzi, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(seconds_to_chrono_nano_seconds(std::get<1>(pointcloud_ts)).count());
    publish_cloud(std::move(ros_pc_msg_ptr), aw_points_base_pub_);
  }
  if (
    aw_points_ex_pub_->get_subscription_count() > 0 ||
    aw_points_ex_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_ex_cloud = nebula::drivers::convert_point_xyzircaedt_to_point_xyziradt(
      pointcloud, std::get<1>(pointcloud_ts));
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_ex_cloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(seconds_to_chrono_nano_seconds(std::get<1>(pointcloud_ts)).count());
    publish_cloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
  }
}

void RobosenseDecoderWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<drivers::RobosenseDriver>(new_config, calibration_cfg_ptr_);
  driver_ptr_ = new_driver;
  sensor_cfg_ = new_config;
}

/// @brief Get current status of this driver
/// @return Current status
nebula::Status RobosenseDecoderWrapper::status()
{
  return status_;
}

void RobosenseDecoderWrapper::publish_cloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(logger_, "Timestamp error, verify clock source.");
    return;
  }
  pointcloud->header.frame_id = sensor_cfg_->frame_id;
  publisher->publish(std::move(pointcloud));
}

}  // namespace nebula::ros
