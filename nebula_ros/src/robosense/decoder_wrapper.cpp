// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/robosense/decoder_wrapper.hpp"

namespace nebula
{
namespace ros
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
  driver_ptr_(new drivers::RobosenseDriver(config, calibration)),
  current_scan_msg_(std::make_unique<nebula_msgs::msg::NebulaPackets>()),
  publish_queue_(1)
{
  status_ = driver_ptr_->GetStatus();

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  RCLCPP_INFO(logger_, ". Starting...");
  RCLCPP_INFO(logger_, ". Driver ");

  // Publish packets only if HW interface is connected
  if (hw_interface_) {
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

  pub_thread_ = std::thread([&]() {
    while (true) {
      auto publish_data = publish_queue_.pop();
      publish(std::move(publish_data));
    }
  });

  RCLCPP_INFO(logger_, "Initialized decoder wrapper.");
}

void RobosenseDecoderWrapper::ProcessCloudPacket(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  auto & packet = *packet_msg;

  // Accumulate packets for recording only if someone is subscribed to the topic (for performance)
  if (hw_interface_) {
    if (current_scan_msg_->packets.size() == 0) {
      current_scan_msg_->header.stamp = packet_msg->stamp;
      current_scan_msg_->header.frame_id = sensor_cfg_->frame_id;
    }

    packet = current_scan_msg_->packets.emplace_back(std::move(*packet_msg));
  }

  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts{};
  nebula::drivers::NebulaPointCloudPtr pointcloud = nullptr;

  {
    std::lock_guard lock(mtx_driver_ptr_);
    pointcloud_ts = driver_ptr_->ParseCloudPacket(packet.data);
    pointcloud = std::get<0>(pointcloud_ts);
  }

  if (pointcloud == nullptr) {
    return;
  }

  publish_queue_.try_push({std::move(current_scan_msg_), pointcloud, std::get<1>(pointcloud_ts)});
  current_scan_msg_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
}

void RobosenseDecoderWrapper::OnConfigChange(
  const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<drivers::RobosenseDriver>(new_config, calibration_cfg_ptr_);
  driver_ptr_ = new_driver;
  sensor_cfg_ = new_config;
}

/// @brief Get current status of this driver
/// @return Current status
nebula::Status RobosenseDecoderWrapper::Status()
{
  return status_;
}

void RobosenseDecoderWrapper::publish(PublishData && data)
{
  auto pointcloud = data.cloud;
  auto header_stamp = rclcpp::Time(SecondsToChronoNanoSeconds(data.cloud_timestamp_s).count());

  cloud_watchdog_->update();

  // Publish scan message only if it has been written to
  if (!data.packets->packets.empty()) {
    auto robosense_scan = std::make_unique<robosense_msgs::msg::RobosenseScan>();
    robosense_scan->header = data.packets->header;

    for (const auto & pkt : data.packets->packets) {
      auto & robosense_pkt = robosense_scan->packets.emplace_back();
      robosense_pkt.stamp = pkt.stamp;
      std::copy(pkt.data.begin(), pkt.data.end(), robosense_pkt.data.begin());
    }

    packets_pub_->publish(std::move(robosense_scan));
  }

  if (
    nebula_points_pub_->get_subscription_count() > 0 ||
    nebula_points_pub_->get_intra_process_subscription_count() > 0) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = header_stamp;
    PublishCloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
  if (
    aw_points_base_pub_->get_subscription_count() > 0 ||
    aw_points_base_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_cloud_xyzi =
      nebula::drivers::convertPointXYZIRCAEDTToPointXYZIR(pointcloud);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_cloud_xyzi, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = header_stamp;
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_base_pub_);
  }
  if (
    aw_points_ex_pub_->get_subscription_count() > 0 ||
    aw_points_ex_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_ex_cloud =
      nebula::drivers::convertPointXYZIRCAEDTToPointXYZIRADT(pointcloud, data.cloud_timestamp_s);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_ex_cloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = header_stamp;
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
  }
}

void RobosenseDecoderWrapper::PublishCloud(
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

}  // namespace ros
}  // namespace nebula
