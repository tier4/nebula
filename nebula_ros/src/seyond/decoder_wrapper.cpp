#include "nebula_ros/seyond/decoder_wrapper.hpp"

namespace nebula
{
namespace ros
{

using namespace std::chrono_literals;

SeyondDecoderWrapper::SeyondDecoderWrapper(
  rclcpp::Node * const parent_node, const std::shared_ptr<SeyondHwInterface> & hw_interface,
  std::shared_ptr<const SeyondSensorConfiguration> & config)
: status_(nebula::Status::NOT_INITIALIZED),
  logger_(parent_node->get_logger().get_child("SeyondDecoder")),
  hw_interface_(hw_interface),
  sensor_cfg_(config)
{
  if (!config) {
    throw std::runtime_error("SeyondDecoderWrapper cannot be instantiated without a valid config!");
  }

  RCLCPP_INFO(logger_, "Starting Decoder");

  driver_ptr_ = std::make_shared<SeyondDriver>(config, calibration_cfg_ptr_);
  status_ = driver_ptr_->GetStatus();

  if (Status::OK != status_) {
    throw std::runtime_error(
      (std::stringstream() << "Error instantiating decoder: " << status_).str());
  }

  // Publish packets only if HW interface is connected
  if (hw_interface_) {
    current_scan_msg_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    packets_pub_ = parent_node->create_publisher<nebula_msgs::msg::NebulaPackets>(
      "nebula_packets", rclcpp::SensorDataQoS());
  }

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  nebula_points_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("nebula_points", pointcloud_qos);

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);

  cloud_watchdog_ =
    std::make_shared<WatchdogTimer>(*parent_node, 100'000us, [this, parent_node](bool ok) {
      if (ok) return;
      RCLCPP_WARN_THROTTLE(
        logger_, *parent_node->get_clock(), 5000, "Missed pointcloud output deadline");
    });
}

void SeyondDecoderWrapper::OnConfigChange(
  const std::shared_ptr<const SeyondSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<SeyondDriver>(new_config, calibration_cfg_ptr_);
  driver_ptr_ = new_driver;
  sensor_cfg_ = new_config;
}

void SeyondDecoderWrapper::ProcessCloudPacket(
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
    pointcloud_ts = driver_ptr_->ParseCloudPacket(packet_msg->data);
    pointcloud = std::get<0>(pointcloud_ts);
  }

  if (pointcloud == nullptr) {
    return;
  };

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
    nebula_points_pub_->get_subscription_count() > 0 ||
    nebula_points_pub_->get_intra_process_subscription_count() > 0) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::duration<double>(std::get<1>(pointcloud_ts)))
                         .count();
    ros_pc_msg_ptr->header.stamp = rclcpp::Time(nanoseconds);
    PublishCloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
}

void SeyondDecoderWrapper::PublishCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(logger_, "Timestamp error, verify clock source.");
  }
  pointcloud->header.frame_id = sensor_cfg_->frame_id;
  publisher->publish(std::move(pointcloud));
}

nebula::Status SeyondDecoderWrapper::Status()
{
  std::lock_guard lock(mtx_driver_ptr_);

  if (!driver_ptr_) {
    return nebula::Status::NOT_INITIALIZED;
  }

  return driver_ptr_->GetStatus();
}
}  // namespace ros
}  // namespace nebula
