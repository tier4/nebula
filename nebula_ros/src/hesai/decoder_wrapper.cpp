// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/hesai/decoder_wrapper.hpp"

#include "nebula_decoders/nebula_decoders_hesai/decoders/functional_safety.hpp"
#include "nebula_ros/common/rclcpp_logger.hpp"
#include "nebula_ros/hesai/diagnostics/functional_safety_diagnostic_task.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/util/string_conversions.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <memory>
#include <tuple>
#include <utility>

#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"
namespace nebula::ros
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

HesaiDecoderWrapper::HesaiDecoderWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config,
  const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> & calibration,
  diagnostic_updater::Updater & diagnostic_updater, bool publish_packets)
: status_(nebula::Status::NOT_INITIALIZED),
  logger_(parent_node->get_logger().get_child("HesaiDecoder")),
  parent_node_(*parent_node),
  sensor_cfg_(config),
  calibration_cfg_ptr_(calibration),
  publish_diagnostic_(make_rate_bound_status(sensor_cfg_->rotation_speed, *parent_node))
{
  if (!sensor_cfg_) {
    throw std::runtime_error("HesaiDecoderWrapper cannot be instantiated without a valid config!");
  }

  if (!calibration_cfg_ptr_) {
    throw std::runtime_error("HesaiDecoderWrapper cannot be instantiated without a valid config!");
  }

  RCLCPP_INFO_STREAM(
    logger_, "Using calibration data from " << calibration_cfg_ptr_->calibration_file);

  RCLCPP_INFO(logger_, "Starting Decoder");

  initialize_functional_safety(diagnostic_updater);
  drivers::FunctionalSafetyDecoderBase::alive_cb_t alive_cb;
  drivers::FunctionalSafetyDecoderBase::stuck_cb_t stuck_cb;
  drivers::FunctionalSafetyDecoderBase::status_cb_t status_cb;

  if (functional_safety_diagnostic_) {
    alive_cb = [this]() { functional_safety_diagnostic_->on_alive(); };
    stuck_cb = [this]() { functional_safety_diagnostic_->on_stuck(); };
    status_cb = [this](
                  drivers::FunctionalSafetySeverity severity,
                  const drivers::FunctionalSafetyErrorCodes & codes) {
      functional_safety_diagnostic_->on_status(severity, codes);
    };
  }

  initialize_packet_loss_diagnostic(diagnostic_updater);
  drivers::PacketLossDetectorBase::lost_cb_t lost_cb;

  if (packet_loss_diagnostic_) {
    lost_cb = [this](uint64_t n_lost) { packet_loss_diagnostic_->on_lost(n_lost); };
  }

  driver_ptr_ = std::make_shared<drivers::HesaiDriver>(
    config, calibration_cfg_ptr_, std::make_shared<drivers::loggers::RclcppLogger>(logger_),
    alive_cb, stuck_cb, status_cb, lost_cb);

  status_ = driver_ptr_->get_status();

  if (Status::OK != status_) {
    throw std::runtime_error("Error instantiating decoder: " + util::to_string(status_));
  }

  // Publish packets only if enabled by the ROS wrapper
  if (publish_packets) {
    current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
    packets_pub_ = parent_node->create_publisher<pandar_msgs::msg::PandarScan>(
      "pandar_packets", rclcpp::SensorDataQoS());
  }

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  nebula_points_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points", pointcloud_qos);
  aw_points_base_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points", pointcloud_qos);
  aw_points_ex_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points_ex", pointcloud_qos);

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);

  diagnostic_updater.add(publish_diagnostic_);

  cloud_watchdog_ =
    std::make_shared<WatchdogTimer>(*parent_node, 100'000us, [this, parent_node](bool ok) {
      if (ok) return;
      RCLCPP_WARN_THROTTLE(
        logger_, *parent_node->get_clock(), 5000, "Missed pointcloud output deadline");
    });
}

void HesaiDecoderWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<drivers::HesaiDriver>(
    new_config, calibration_cfg_ptr_, std::make_shared<drivers::loggers::RclcppLogger>(logger_));
  driver_ptr_ = new_driver;
  sensor_cfg_ = new_config;
}

void HesaiDecoderWrapper::on_calibration_change(
  const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> & new_calibration)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<drivers::HesaiDriver>(
    sensor_cfg_, new_calibration, std::make_shared<drivers::loggers::RclcppLogger>(logger_));
  driver_ptr_ = new_driver;
  calibration_cfg_ptr_ = new_calibration;
}

void HesaiDecoderWrapper::process_cloud_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  // Accumulate packets for recording only if someone is subscribed to the topic (for performance)
  if (
    packets_pub_ && (packets_pub_->get_subscription_count() > 0 ||
                     packets_pub_->get_intra_process_subscription_count() > 0)) {
    if (current_scan_msg_->packets.size() == 0) {
      current_scan_msg_->header.stamp = packet_msg->stamp;
    }

    pandar_msgs::msg::PandarPacket pandar_packet_msg{};
    pandar_packet_msg.stamp = packet_msg->stamp;
    pandar_packet_msg.size = packet_msg->data.size();
    std::copy(packet_msg->data.begin(), packet_msg->data.end(), pandar_packet_msg.data.begin());
    current_scan_msg_->packets.emplace_back(std::move(pandar_packet_msg));
  }

  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts{};
  nebula::drivers::NebulaPointCloudPtr pointcloud = nullptr;
  {
    std::lock_guard lock(mtx_driver_ptr_);
    pointcloud_ts = driver_ptr_->parse_cloud_packet(packet_msg->data);
    pointcloud = std::get<0>(pointcloud_ts);
  }

  // A pointcloud is only emitted when a scan completes (e.g. 3599 packets do not emit, the 3600th
  // emits one)
  if (pointcloud == nullptr) {
    // Since this ends the function early, the `cloud_watchdog_` will not be updated.
    // Thus, if pointclouds are not emitted for too long (e.g. when decoder settings are wrong or no
    // packets come in), the watchdog will log a warning automatically
    return;
  }

  cloud_watchdog_->update();

  // Publish scan message only if it has been written to
  if (current_scan_msg_ && !current_scan_msg_->packets.empty()) {
    packets_pub_->publish(std::move(current_scan_msg_));
    current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
  }

  rclcpp::Time cloud_stamp =
    rclcpp::Time(seconds_to_chrono_nano_seconds(std::get<1>(pointcloud_ts)).count());

  if (
    nebula_points_pub_->get_subscription_count() > 0 ||
    nebula_points_pub_->get_intra_process_subscription_count() > 0) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = cloud_stamp;
    publish_cloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
  if (
    aw_points_base_pub_->get_subscription_count() > 0 ||
    aw_points_base_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_cloud_xyzi =
      nebula::drivers::convert_point_xyzircaedt_to_point_xyzir(pointcloud);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_cloud_xyzi, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = cloud_stamp;
    publish_cloud(std::move(ros_pc_msg_ptr), aw_points_base_pub_);
  }
  if (
    aw_points_ex_pub_->get_subscription_count() > 0 ||
    aw_points_ex_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_ex_cloud = nebula::drivers::convert_point_xyzircaedt_to_point_xyziradt(
      pointcloud, std::get<1>(pointcloud_ts));
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_ex_cloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = cloud_stamp;
    publish_cloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
  }

  publish_diagnostic_.tick();
}

void HesaiDecoderWrapper::publish_cloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(logger_, "Timestamp error, verify clock source.");
  }
  pointcloud->header.frame_id = sensor_cfg_->frame_id;
  publisher->publish(std::move(pointcloud));
}

void HesaiDecoderWrapper::initialize_functional_safety(
  diagnostic_updater::Updater & diagnostic_updater)
{
  if (!drivers::supports_functional_safety(sensor_cfg_->sensor_model)) {
    return;
  }

  functional_safety_diagnostic_.emplace(&parent_node_);
  diagnostic_updater.add(functional_safety_diagnostic_.value());
}

void HesaiDecoderWrapper::initialize_packet_loss_diagnostic(
  diagnostic_updater::Updater & diagnostic_updater)
{
  if (!drivers::supports_packet_loss_detection(sensor_cfg_->sensor_model)) {
    return;
  }

  uint64_t error_threshold =
    parent_node_.declare_parameter<uint16_t>("diagnostics.packet_loss.error_threshold");
  packet_loss_diagnostic_.emplace(error_threshold, parent_node_.get_clock());
  diagnostic_updater.add(packet_loss_diagnostic_.value());
}

nebula::Status HesaiDecoderWrapper::status()
{
  std::lock_guard lock(mtx_driver_ptr_);

  if (!driver_ptr_) {
    return nebula::Status::NOT_INITIALIZED;
  }

  return driver_ptr_->get_status();
}
}  // namespace nebula::ros
