// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/tutorial/decoder_wrapper.hpp"

namespace nebula::ros
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

TutorialDecoderWrapper::TutorialDecoderWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<nebula::drivers::TutorialHwInterface> & hw_interface,
  std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & config)
: status_(nebula::Status::NOT_INITIALIZED),
  logger_(parent_node->get_logger().get_child("TutorialDecoder")),
  hw_interface_(hw_interface),
  sensor_cfg_(config)
{
  if (!config) {
    throw std::runtime_error(
      "TutorialDecoderWrapper cannot be instantiated without a valid config!");
  }

  // ////////////////////////////////////////
  // Instantiate decoder
  // ////////////////////////////////////////

  RCLCPP_INFO(logger_, "Starting Decoder");

  driver_ptr_ = std::make_shared<TutorialDriver>(config, nullptr);
  status_ = driver_ptr_->get_status();

  if (Status::OK != status_) {
    throw std::runtime_error(
      (std::stringstream() << "Error instantiating decoder: " << status_).str());
  }

  // ////////////////////////////////////////
  // Start ROS integrations
  // ////////////////////////////////////////

  // Publish packets only if HW interface is connected
  if (hw_interface_) {
    current_scan_msg_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    packets_pub_ = parent_node->create_publisher<nebula_msgs::msg::NebulaPackets>(
      "nebula_packets", rclcpp::SensorDataQoS());
  }

  const auto & qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

  cloud_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("nebula_points", pointcloud_qos);

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);

  cloud_watchdog_ =
    std::make_shared<WatchdogTimer>(*parent_node, 100'000us, [this, parent_node](bool ok) {
      if (ok) return;
      RCLCPP_WARN_THROTTLE(
        logger_, *parent_node->get_clock(), 5000, "Missed pointcloud output deadline");
    });
}

void TutorialDecoderWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  // Re-instantiate driver on config change
  auto new_driver = std::make_shared<TutorialDriver>(new_config, nullptr);
  driver_ptr_ = new_driver;
  sensor_cfg_ = new_config;
}

void TutorialDecoderWrapper::process_cloud_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  // ////////////////////////////////////////
  // Accumulate packets for recording
  // ////////////////////////////////////////

  // Accumulate packets for recording only if someone is subscribed to the topic (for performance)
  if (
    hw_interface_ && (packets_pub_->get_subscription_count() > 0 ||
                      packets_pub_->get_intra_process_subscription_count() > 0)) {
    if (current_scan_msg_->packets.size() == 0) {
      current_scan_msg_->header.stamp = packet_msg->stamp;
    }

    current_scan_msg_->packets.emplace_back(*packet_msg);
  }

  // ////////////////////////////////////////
  // Decode packet
  // ////////////////////////////////////////

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

  // ////////////////////////////////////////
  // If scan completed, publish pointcloud
  // ////////////////////////////////////////

  // A pointcloud has been produced, reset the watchdog timer
  cloud_watchdog_->update();

  // Publish scan message only if it has been written to
  if (current_scan_msg_ && !current_scan_msg_->packets.empty()) {
    packets_pub_->publish(std::move(current_scan_msg_));
    current_scan_msg_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  }

  if (
    cloud_pub_->get_subscription_count() > 0 ||
    cloud_pub_->get_intra_process_subscription_count() > 0) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::duration<double>(std::get<1>(pointcloud_ts)))
                         .count();
    ros_pc_msg_ptr->header.stamp = rclcpp::Time(nanoseconds);
    pointcloud->header.frame_id = sensor_cfg_->frame_id;
    cloud_pub_->publish(std::move(ros_pc_msg_ptr));
  }
}

nebula::Status TutorialDecoderWrapper::status()
{
  std::lock_guard lock(mtx_driver_ptr_);

  if (!driver_ptr_) {
    return nebula::Status::NOT_INITIALIZED;
  }

  return driver_ptr_->get_status();
}
}  // namespace nebula::ros
