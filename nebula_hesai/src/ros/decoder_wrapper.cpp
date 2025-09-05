// Copyright 2024 TIER IV, Inc.

#include "nebula/ros/hesai/decoder_wrapper.hpp"

#include "nebula/decoders/nebula_decoders_hesai/decoders/functional_safety.hpp"
#include "nebula/common/ros/agnocast_wrapper/nebula_agnocast_wrapper.hpp"
#include "nebula/common/ros/rclcpp_logger.hpp"
#include "nebula/ros/hesai/diagnostics/functional_safety_diagnostic_task.hpp"

#include <nebula/common/common/hesai/hesai_common.hpp>
#include <nebula/common/common/util/string_conversions.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <memory>
#include <tuple>
#include <utility>

#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"
namespace nebula::ros
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

HesaiDecoderWrapper::HesaiDecoderWrapper(
  rclcpp::Node * parent_node,
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
  initialize_packet_loss_diagnostic(diagnostic_updater);

  driver_ptr_ = initialize_driver(sensor_cfg_, calibration_cfg_ptr_);
  status_ = driver_ptr_->get_status();

  if (Status::OK != status_) {
    throw std::runtime_error("Error instantiating decoder: " + util::to_string(status_));
  }

  // Publish packets only if enabled by the ROS wrapper
  if (publish_packets) {
    current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
    packets_pub_ = parent_node->create_publisher<pandar_msgs::msg::PandarScan>(
      "pandar_packets", rclcpp::SensorDataQoS());
    packets_pub_thread_.emplace(
      [this](pandar_msgs::msg::PandarScan::UniquePtr && msg) {
        if (packets_pub_) {
          packets_pub_->publish(std::move(msg));
        }
      },
      10);
  }

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  nebula_points_pub_ = NEBULA_CREATE_PUBLISHER2(
    sensor_msgs::msg::PointCloud2, &parent_node_, "pandar_points", pointcloud_qos);
  aw_points_base_pub_ = NEBULA_CREATE_PUBLISHER2(
    sensor_msgs::msg::PointCloud2, &parent_node_, "aw_points", pointcloud_qos);
  aw_points_ex_pub_ = NEBULA_CREATE_PUBLISHER2(
    sensor_msgs::msg::PointCloud2, &parent_node_, "aw_points_ex", pointcloud_qos);

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);

  diagnostic_updater.add(publish_diagnostic_);
}

void HesaiDecoderWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  driver_ptr_ = initialize_driver(new_config, calibration_cfg_ptr_);
  sensor_cfg_ = new_config;
}

void HesaiDecoderWrapper::on_calibration_change(
  const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> & new_calibration)
{
  std::lock_guard lock(mtx_driver_ptr_);
  driver_ptr_ = initialize_driver(sensor_cfg_, new_calibration);
  calibration_cfg_ptr_ = new_calibration;
}

nebula::util::expected<drivers::PacketMetadata, drivers::DecodeError>
HesaiDecoderWrapper::process_cloud_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  // Ideally, we would only accumulate packets if someone is subscribed to the packets topic.
  // However, checking for subscriptions in the decode thread (here) causes contention with the
  // publish thread, negating the benefits of multi-threading.
  // Not checking is an okay trade-off for the time being.
  if (packets_pub_) {
    if (current_scan_msg_->packets.size() == 0) {
      current_scan_msg_->header.stamp = packet_msg->stamp;
    }

    pandar_msgs::msg::PandarPacket pandar_packet_msg{};
    pandar_packet_msg.stamp = packet_msg->stamp;
    pandar_packet_msg.size = packet_msg->data.size();
    std::copy(packet_msg->data.begin(), packet_msg->data.end(), pandar_packet_msg.data.begin());
    current_scan_msg_->packets.emplace_back(pandar_packet_msg);
  }

  std::lock_guard lock(mtx_driver_ptr_);
  return driver_ptr_->parse_cloud_packet(packet_msg->data);
}

void HesaiDecoderWrapper::on_pointcloud_decoded(
  const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s)
{
  // Publish scan message only if it has been written to
  if (current_scan_msg_ && !current_scan_msg_->packets.empty() && packets_pub_thread_) {
    bool success = packets_pub_thread_->try_push(std::move(current_scan_msg_));
    if (!success) {
      RCLCPP_WARN_STREAM(logger_, "Packet publish queue is full, dropping scan.");
    }

    current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
  }

  rclcpp::Time cloud_stamp = rclcpp::Time(seconds_to_chrono_nano_seconds(timestamp_s).count());

  if (NEBULA_HAS_ANY_SUBSCRIPTIONS(nebula_points_pub_)) {
    auto ros_pc_msg_ptr = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(nebula_points_pub_);
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = cloud_stamp;
    publish_cloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
  if (NEBULA_HAS_ANY_SUBSCRIPTIONS(aw_points_base_pub_)) {
    const auto autoware_cloud_xyzi =
      nebula::drivers::convert_point_xyzircaedt_to_point_xyzir(pointcloud);
    auto ros_pc_msg_ptr = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(aw_points_base_pub_);
    pcl::toROSMsg(*autoware_cloud_xyzi, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = cloud_stamp;
    publish_cloud(std::move(ros_pc_msg_ptr), aw_points_base_pub_);
  }
  if (NEBULA_HAS_ANY_SUBSCRIPTIONS(aw_points_ex_pub_)) {
    const auto autoware_ex_cloud =
      nebula::drivers::convert_point_xyzircaedt_to_point_xyziradt(pointcloud, timestamp_s);
    auto ros_pc_msg_ptr = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(aw_points_ex_pub_);
    pcl::toROSMsg(*autoware_ex_cloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = cloud_stamp;
    publish_cloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
  }

  publish_diagnostic_.tick();
}

void HesaiDecoderWrapper::publish_cloud(
  NEBULA_MESSAGE_UNIQUE_PTR(sensor_msgs::msg::PointCloud2) && pointcloud,
  const NEBULA_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) & publisher)
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

std::pair<
  std::shared_ptr<drivers::point_filters::BlockageMaskPlugin>,
  NEBULA_PUBLISHER_PTR(sensor_msgs::msg::Image)>
HesaiDecoderWrapper::initialize_blockage_mask_plugin()
{
  if (!sensor_cfg_->blockage_mask_horizontal_bin_size_mdeg) {
    return {nullptr, nullptr};
  }

  auto blockage_mask_plugin = std::make_shared<drivers::point_filters::BlockageMaskPlugin>(
    sensor_cfg_->blockage_mask_horizontal_bin_size_mdeg.value());
  auto blockage_mask_pub = NEBULA_CREATE_PUBLISHER2(
    sensor_msgs::msg::Image, &parent_node_, "blockage_mask", rclcpp::SensorDataQoS());

  blockage_mask_plugin->set_callback(
    [this, blockage_mask_pub](
      const drivers::point_filters::BlockageMask & blockage_mask, double timestamp_s) {
      auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(blockage_mask_pub_);
      msg->header.stamp = rclcpp::Time(seconds_to_chrono_nano_seconds(timestamp_s).count());
      msg->header.frame_id = sensor_cfg_->frame_id;
      msg->encoding = sensor_msgs::image_encodings::MONO8;
      msg->width = blockage_mask.get_width();
      msg->height = blockage_mask.get_height();
      msg->is_bigendian = false;
      msg->step = blockage_mask.get_width() * sizeof(uint8_t);
      msg->data = blockage_mask.get_mask();
      blockage_mask_pub->publish(std::move(msg));
    });

  return {blockage_mask_plugin, blockage_mask_pub};
}

std::shared_ptr<drivers::HesaiDriver> HesaiDecoderWrapper::initialize_driver(
  const std::shared_ptr<const drivers::HesaiSensorConfiguration> & config,
  const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> & calibration)
{
  auto pointcloud_cb = [this](const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s) {
    on_pointcloud_decoded(pointcloud, timestamp_s);
  };

  auto [blockage_mask_plugin, blockage_mask_pub] = initialize_blockage_mask_plugin();
  blockage_mask_pub_ = blockage_mask_pub;

  drivers::FunctionalSafetyDecoderBase::alive_cb_t alive_cb;
  drivers::FunctionalSafetyDecoderBase::stuck_cb_t stuck_cb;
  drivers::FunctionalSafetyDecoderBase::status_cb_t status_cb;

  if (functional_safety_diagnostic_) {
    alive_cb = [this]() { functional_safety_diagnostic_->on_alive(); };
    stuck_cb = [this](bool is_stuck) { functional_safety_diagnostic_->on_stuck(is_stuck); };
    status_cb = [this](
                  drivers::FunctionalSafetySeverity severity,
                  const drivers::FunctionalSafetyErrorCodes & codes) {
      functional_safety_diagnostic_->on_status(severity, codes);
    };
  }

  drivers::PacketLossDetectorBase::lost_cb_t lost_cb;

  if (packet_loss_diagnostic_) {
    lost_cb = [this](uint64_t n_lost) { packet_loss_diagnostic_->on_lost(n_lost); };
  }

  return std::make_shared<drivers::HesaiDriver>(
    config, calibration, std::make_shared<drivers::loggers::RclcppLogger>(logger_),
    std::move(pointcloud_cb), std::move(alive_cb), std::move(stuck_cb), std::move(status_cb),
    std::move(lost_cb), std::move(blockage_mask_plugin));
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
