// Copyright 2024 TIER IV, Inc.

#include "nebula_hesai/decoder_wrapper.hpp"

#include "nebula_core_ros/agnocast_wrapper/nebula_agnocast_wrapper.hpp"
#include "nebula_core_ros/cie_thread_factory.hpp"
#include "nebula_core_ros/point_cloud_conversions.hpp"
#include "nebula_core_ros/rclcpp_logger.hpp"

#include <nebula_core_common/util/stopwatch.hpp>
#include <nebula_core_common/util/string_conversions.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int64_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <memory>
#include <mutex>
#include <utility>

#if defined(__clang__)
#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"
#endif
namespace nebula::ros
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

HesaiDecoderWrapper::HesaiDecoderWrapper(
  nebula::agnocast_wrapper::Node * parent_node,
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config,
  const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> & calibration,
  nebula::agnocast_wrapper::diagnostic_updater::Updater & diagnostic_updater, bool publish_packets)
: status_(nebula::Status::NOT_INITIALIZED),
  logger_(parent_node->get_logger().get_child("DecoderWrapper")),
  parent_node_(*parent_node),
  sensor_cfg_(config),
  debug_publisher_(parent_node, "nebula")
{
  if (!sensor_cfg_) {
    throw std::runtime_error("HesaiDecoderWrapper cannot be instantiated without a valid config!");
  }

  if (!calibration) {
    throw std::runtime_error("HesaiDecoderWrapper cannot be instantiated without a valid config!");
  }

  RCLCPP_INFO_STREAM(logger_, "Using calibration data from " << calibration->calibration_file);

  RCLCPP_INFO(logger_, "Starting Decoder");

  // Publish packets only if enabled by the ROS wrapper
  if (publish_packets) {
    current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
    packets_pub_ = parent_node->create_publisher<pandar_msgs::msg::PandarScan>(
      "pandar_packets", rclcpp::SensorDataQoS());
    packets_pub_thread_.emplace(
      [this](pandar_msgs::msg::PandarScan::UniquePtr && msg) {
        if (packets_pub_) {
          packets_pub_->publish(*msg);
        }
      },
      10, make_cie_thread_factory("nebula_hesai_packets_pub@" + sensor_cfg_->frame_id));
  }

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  nebula_points_pub_ =
    parent_node_.create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points", pointcloud_qos);
  aw_points_base_pub_ =
    parent_node_.create_publisher<sensor_msgs::msg::PointCloud2>("aw_points", pointcloud_qos);
  aw_points_ex_pub_ =
    parent_node_.create_publisher<sensor_msgs::msg::PointCloud2>("aw_points_ex", pointcloud_qos);

  drivers::HesaiDecodePipeline::blockage_mask_callback_t blockage_mask_cb;
  if (sensor_cfg_->blockage_mask_horizontal_bin_size_mdeg) {
    blockage_mask_pub_ = parent_node_.create_publisher<sensor_msgs::msg::Image>(
      "blockage_mask", rclcpp::SensorDataQoS());
    blockage_mask_cb = [this](
                         const drivers::point_filters::BlockageMask & blockage_mask,
                         double timestamp_s) { on_blockage_mask(blockage_mask, timestamp_s); };
  }

  pipeline_.emplace(
    sensor_cfg_, calibration,
    declare_diagnostics_configuration(sensor_cfg_->sensor_model, parent_node_),
    std::make_shared<drivers::loggers::RclcppLogger>(logger_), make_diagnostics_clock(parent_node_),
    [this](const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s) {
      on_pointcloud_decoded(pointcloud, timestamp_s);
    },
    std::move(blockage_mask_cb));
  status_ = pipeline_->status();

  using diagnostic_updater::DiagnosticStatusWrapper;

  if (drivers::supports_functional_safety(sensor_cfg_->sensor_model)) {
    diagnostic_updater.add("Functional safety status", [this](DiagnosticStatusWrapper & status) {
      render(pipeline_->report_functional_safety().value(), status);
    });
  }

  if (drivers::supports_packet_loss_detection(sensor_cfg_->sensor_model)) {
    diagnostic_updater.add("Packet loss status", [this](DiagnosticStatusWrapper & status) {
      render(pipeline_->report_packet_loss().value(), status);
    });
  }

  diagnostic_updater.add("rate bound check", [this](DiagnosticStatusWrapper & status) {
    render(pipeline_->report_pointcloud_rate(), status);
  });

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);
}

drivers::HesaiDecoderDiagnosticsConfiguration
HesaiDecoderWrapper::declare_diagnostics_configuration(
  drivers::SensorModel sensor_model, nebula::agnocast_wrapper::Node & node)
{
  drivers::HesaiDecoderDiagnosticsConfiguration config;

  auto & rate = config.pointcloud_publish_rate;
  rate.frequency_ok.min_hz =
    node.declare_parameter<double>("diagnostics.pointcloud_publish_rate.frequency_ok.min_hz");
  rate.frequency_ok.max_hz = node.declare_parameter<double>(
    "diagnostics.pointcloud_publish_rate.frequency_ok.max_hz", rate.frequency_ok.max_hz);
  rate.frequency_warn.min_hz =
    node.declare_parameter<double>("diagnostics.pointcloud_publish_rate.frequency_warn.min_hz");
  rate.frequency_warn.max_hz = node.declare_parameter<double>(
    "diagnostics.pointcloud_publish_rate.frequency_warn.max_hz", rate.frequency_warn.max_hz);
  rate.num_frame_transition =
    node.declare_parameter<int>("diagnostics.pointcloud_publish_rate.num_frame_transition", 1);

  if (drivers::supports_packet_loss_detection(sensor_model)) {
    config.packet_loss.emplace();
    config.packet_loss->error_threshold =
      node.declare_parameter<uint16_t>("diagnostics.packet_loss.error_threshold");
  }

  return config;
}

void HesaiDecoderWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_pipeline_);
  sensor_cfg_ = new_config;
  current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
  current_scan_perf_counters_ = {};
  pipeline_->set_sensor_configuration(new_config);
}

void HesaiDecoderWrapper::on_calibration_change(
  const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> & new_calibration)
{
  std::lock_guard lock(mtx_pipeline_);
  current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
  current_scan_perf_counters_ = {};
  pipeline_->set_calibration(new_calibration);
}

void HesaiDecoderWrapper::reset_decoder()
{
  std::lock_guard lock(mtx_pipeline_);
  current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
  current_scan_perf_counters_ = {};
  pipeline_->reset();
}

drivers::PacketDecodeResult HesaiDecoderWrapper::process_cloud_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg, uint64_t receive_time_ns)
{
  current_scan_perf_counters_.receive_time_current_scan_ns += receive_time_ns;

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

  std::lock_guard lock(mtx_pipeline_);
  auto decode_result = pipeline_->process_packet(packet_msg->data);

  current_scan_perf_counters_.decode_time_current_scan_ns +=
    decode_result.performance_counters.decode_time_ns;

  return decode_result;
}

void HesaiDecoderWrapper::on_pointcloud_decoded(
  const drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s)
{
  util::Stopwatch publish_watch;

  // Publish scan message only if it has been written to
  if (current_scan_msg_ && !current_scan_msg_->packets.empty() && packets_pub_thread_) {
    bool success = packets_pub_thread_->try_push(std::move(current_scan_msg_));
    if (!success) {
      RCLCPP_WARN_STREAM(logger_, "Packet publish queue is full, dropping scan.");
    }

    current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
  }

  rclcpp::Time cloud_stamp = rclcpp::Time(seconds_to_chrono_nano_seconds(timestamp_s).count());

  if (
    nebula_points_pub_->get_subscription_count() > 0 ||
    nebula_points_pub_->get_intra_process_subscription_count() > 0) {
    auto ros_pc_msg_ptr = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(nebula_points_pub_);
    *ros_pc_msg_ptr = nebula::ros::to_ros_msg(*pointcloud);
    ros_pc_msg_ptr->header.stamp = cloud_stamp;
    publish_cloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
  if (
    aw_points_base_pub_->get_subscription_count() > 0 ||
    aw_points_base_pub_->get_intra_process_subscription_count() > 0) {
    auto autoware_cloud_xyzi =
      nebula::drivers::convert_point_xyzircaedt_to_point_xyzir(*pointcloud);
    auto ros_pc_msg_ptr = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(aw_points_base_pub_);
    *ros_pc_msg_ptr = nebula::ros::to_ros_msg(autoware_cloud_xyzi);
    ros_pc_msg_ptr->header.stamp = cloud_stamp;
    publish_cloud(std::move(ros_pc_msg_ptr), aw_points_base_pub_);
  }
  if (
    aw_points_ex_pub_->get_subscription_count() > 0 ||
    aw_points_ex_pub_->get_intra_process_subscription_count() > 0) {
    auto autoware_ex_cloud =
      nebula::drivers::convert_point_xyzircaedt_to_point_xyziradt(*pointcloud, timestamp_s);
    auto ros_pc_msg_ptr = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(aw_points_ex_pub_);
    *ros_pc_msg_ptr = nebula::ros::to_ros_msg(autoware_ex_cloud);
    ros_pc_msg_ptr->header.stamp = cloud_stamp;
    publish_cloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
  }

  uint64_t publish_duration_ns = publish_watch.elapsed_ns();
  current_scan_perf_counters_.publish_time_current_scan_ns += publish_duration_ns;

  publish_and_reset_performance_counters(cloud_stamp);
}

void HesaiDecoderWrapper::publish_and_reset_performance_counters(
  const rclcpp::Time & scan_timestamp)
{
  autoware_internal_debug_msgs::msg::Float64Stamped receive_time_msg;
  receive_time_msg.stamp = scan_timestamp;
  receive_time_msg.data = (current_scan_perf_counters_.receive_time_current_scan_ns) / 1'000'000.0;
  debug_publisher_.publish("debug/receive_duration_ms", receive_time_msg);

  autoware_internal_debug_msgs::msg::Float64Stamped decode_time_msg;
  decode_time_msg.stamp = scan_timestamp;
  decode_time_msg.data = (current_scan_perf_counters_.decode_time_current_scan_ns) / 1'000'000.0;
  debug_publisher_.publish("debug/decode_duration_ms", decode_time_msg);

  autoware_internal_debug_msgs::msg::Float64Stamped publish_time_msg;
  publish_time_msg.stamp = scan_timestamp;
  publish_time_msg.data = (current_scan_perf_counters_.publish_time_current_scan_ns) / 1'000'000.0;
  debug_publisher_.publish("debug/publish_duration_ms", publish_time_msg);

  current_scan_perf_counters_ = {};
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

void HesaiDecoderWrapper::on_blockage_mask(
  const drivers::point_filters::BlockageMask & blockage_mask, double timestamp_s)
{
  auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(blockage_mask_pub_);
  msg->header.stamp = rclcpp::Time(seconds_to_chrono_nano_seconds(timestamp_s).count());
  msg->header.frame_id = sensor_cfg_->frame_id;
  msg->encoding = sensor_msgs::image_encodings::MONO8;
  msg->width = blockage_mask.get_width();
  msg->height = blockage_mask.get_height();
  msg->is_bigendian = false;
  msg->step = blockage_mask.get_width() * sizeof(uint8_t);
  msg->data = blockage_mask.get_mask();
  blockage_mask_pub_->publish(std::move(msg));
}

nebula::Status HesaiDecoderWrapper::status()
{
  std::lock_guard lock(mtx_pipeline_);

  if (!pipeline_) {
    return nebula::Status::NOT_INITIALIZED;
  }

  return pipeline_->status();
}
}  // namespace nebula::ros
