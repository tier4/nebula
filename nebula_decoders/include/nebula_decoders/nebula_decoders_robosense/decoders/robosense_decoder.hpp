#pragma once

#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/angles.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/channel.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/distance.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/intensity.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/return_mode.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/scan_completion.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/timestamp.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/validity.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_scan_decoder.hpp"

#include <rclcpp/rclcpp.hpp>

#include "robosense_msgs/msg/robosense_packet.hpp"
#include "robosense_msgs/msg/robosense_scan.hpp"

#include <boost/container/static_vector.hpp>

#include <mutex>
#include <type_traits>

namespace nebula
{
namespace drivers
{

using namespace nebula::drivers::sensor_mixins;

template <typename SensorT>
class RobosenseDecoder : public RobosenseScanDecoder
{
  // I want C++20 concepts :')
  static_assert(std::is_base_of_v<SensorBase<typename SensorT::packet_t>, SensorT>);
  static_assert(std::is_base_of_v<AnglesMixin<typename SensorT::packet_t>, SensorT>);
  static_assert(std::is_base_of_v<ChannelMixin<typename SensorT::packet_t>, SensorT>);
  static_assert(std::is_base_of_v<ValidityMixin<typename SensorT::packet_t>, SensorT>);
  static_assert(std::is_base_of_v<DistanceMixin<typename SensorT::packet_t>, SensorT>);
  static_assert(std::is_base_of_v<IntensityMixin<typename SensorT::packet_t>, SensorT>);
  static_assert(std::is_base_of_v<ReturnModeMixin<typename SensorT::packet_t>, SensorT>);
  static_assert(std::is_base_of_v<PointTimestampMixin<typename SensorT::packet_t>, SensorT>);
  static_assert(std::is_base_of_v<ScanCompletionMixin<typename SensorT::packet_t>, SensorT>);
  static_assert(std::is_base_of_v<AngleCorrectorMixin<typename SensorT::packet_t>, SensorT>);
  static_assert(std::is_base_of_v<PacketTimestampMixin<typename SensorT::packet_t>, SensorT>);

protected:
  /// @brief Configuration for this decoder
  std::shared_ptr<const drivers::RobosenseSensorConfiguration> sensor_configuration_;
  std::mutex sensor_configuration_mutex_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_;

  /// @brief The point cloud new points get added to
  NebulaPointCloudPtr decode_pc_;
  /// @brief The point cloud that is returned when a scan is complete
  NebulaPointCloudPtr output_pc_;

  /// @brief Some sensors need to decode return groups across multiple packets. This is dictated by
  /// whether or not their return groups are strided along the packet axis (0)
  static constexpr size_t decode_group_size_ =
    SensorT::RETURN_GROUP_STRIDE[0] ? SensorT::packet_t::MAX_RETURNS : 1;
  /// @brief The current group of packets being decoded.
  std::vector<typename SensorT::packet_t> decode_group_;
  std::vector<uint64_t> decode_group_timestamps_;

  /// @brief The timestamp of the last completed scan in nanoseconds
  uint64_t output_scan_timestamp_ns_;
  /// @brief The timestamp of the scan currently in progress
  uint64_t decode_scan_timestamp_ns_;
  /// @brief Whether a full scan has been processed
  bool has_scanned_;

  rclcpp::Logger logger_;
  rclcpp::Clock clock_;

  /// @brief Validates and parses MsopPacket. Currently only checks size, not checksums etc.
  /// @param msop_packet The incoming MsopPacket
  /// @return Whether the packet was parsed successfully
  bool parsePacket(const robosense_msgs::msg::RobosensePacket & msop_packet)
  {
    if (msop_packet.size < sizeof(typename SensorT::packet_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch:" << msop_packet.size << " | Expected at least:"
                                         << sizeof(typename SensorT::packet_t));
      return false;
    }

    const size_t packet_idx = sensor_.getDecodeGroupIndex(msop_packet.data.data());
    if (packet_idx >= decode_group_size_) {
      RCLCPP_ERROR(
        logger_, "Packet index out of bounds: %lu (expected at most: %lu)", packet_idx,
        decode_group_size_);
      decode_group_.clear();
      decode_group_timestamps_.clear();
      return false;
    }

    if (packet_idx > decode_group_.size()) {
      RCLCPP_WARN_THROTTLE(logger_, clock_, 1000, "Dropped packets detected");
      decode_group_.clear();
      decode_group_timestamps_.clear();
      return false;
    }

    if (packet_idx == 0) {
      decode_group_.clear();
      decode_group_timestamps_.clear();
    }

    decode_group_.emplace_back();  // Guaranteed to be at packet_idx
    if (std::memcpy(
          &decode_group_[packet_idx], msop_packet.data.data(),
          sizeof(typename SensorT::packet_t))) {
      const auto & parsed_packet = decode_group_[packet_idx];
      decode_group_timestamps_.emplace_back(sensor_.getPacketTimestamp(parsed_packet));
      return true;
    }

    RCLCPP_ERROR(logger_, "Packet memcopy failed");
    return false;
  }

  /// @brief Converts a group of returns (i.e. 1 for single return, 2 for dual return, etc.) to
  /// points and appends them to the point cloud
  /// @param start_packet_id The index of the first packet in the current decode group
  /// @param start_block_id The first block in the group of returns
  /// @param start_unit_id The first channel in the return group
  /// @param n_returns The number of returns in the group
  /// @param return_mode The currently active return mode
  void convertReturnGroup(
    size_t start_packet_id, size_t start_block_id, size_t start_unit_id, size_t n_returns,
    ReturnMode return_mode)
  {
    boost::container::static_vector<
      const typename SensorT::packet_t::body_t::block_t::unit_t *, SensorT::packet_t::MAX_RETURNS>
      return_units{};
    boost::container::static_vector<size_t, SensorT::packet_t::MAX_RETURNS> packet_idxs{};
    boost::container::static_vector<size_t, SensorT::packet_t::MAX_RETURNS> block_idxs{};
    boost::container::static_vector<size_t, SensorT::packet_t::MAX_RETURNS> unit_idxs{};

    // Find the units corresponding to the same return group as the current one.
    // These are used to find duplicates in multi-return mode.
    for (size_t return_idx = 0; return_idx < n_returns; ++return_idx) {
      size_t packet_idx = start_packet_id + return_idx * SensorT::RETURN_GROUP_STRIDE[0];
      size_t block_idx = start_block_id + return_idx * SensorT::RETURN_GROUP_STRIDE[1];
      size_t unit_idx = start_unit_id + return_idx * SensorT::RETURN_GROUP_STRIDE[2];

      packet_idxs[return_idx] = packet_idx;
      block_idxs[return_idx] = block_idx;
      unit_idxs[return_idx] = unit_idx;

      return_units[return_idx] = getUnit(decode_group_[packet_idx], block_idx, unit_idx);
    }

    // For each return unit, validate it and check if it is a duplicate of any other
    // unit in the return group. If not, convert it to a point and add it to the point cloud.
    for (size_t return_idx = 0; return_idx < n_returns; ++return_idx) {
      const auto & packet = decode_group_[packet_idxs[return_idx]];
      const auto packet_idx = packet_idxs[return_idx];
      const auto block_idx = block_idxs[return_idx];
      const auto unit_idx = unit_idxs[return_idx];

      // 1. Validate point
      if (!sensor_.isValid(packet, block_idx, unit_idx)) {
        continue;
      }

      // 2. Range checks
      auto distance = sensor_.getDistance(packet, block_idx, unit_idx);

      if (distance < SensorT::MIN_RANGE || distance > SensorT::MAX_RANGE) {
        continue;
      }

      // 3. Return group duplicate checks
      auto return_type = sensor_.getReturnType(return_units, return_idx, return_mode);

      // Keep only last of multiple identical points
      if (return_type == ReturnType::IDENTICAL && return_idx != n_returns - 1) {
        continue;
      }

      // Keep only last (if any) of multiple points that are too close
      if (return_idx != n_returns - 1) {
        bool is_below_multi_return_threshold = false;

        for (size_t other_return_idx = 0; return_idx < n_returns; ++return_idx) {
          if (other_return_idx == return_idx) {
            continue;
          }

          const auto & other_packet = decode_group_[packet_idxs[other_return_idx]];
          const auto other_block_idx = block_idxs[other_return_idx];
          const auto other_unit_idx = unit_idxs[other_return_idx];

          if (
            fabsf(sensor_.getDistance(other_packet, other_block_idx, other_unit_idx) - distance) <
            sensor_configuration_->dual_return_distance_threshold) {
            is_below_multi_return_threshold = true;
            break;
          }
        }

        if (is_below_multi_return_threshold) {
          continue;
        }
      }

      // 4. Get fields intrinsic to the point
      NebulaPoint point;
      point.distance = distance;
      point.intensity = sensor_.getIntensity(packet, block_idx, unit_idx);

      // 5. Do timing correction
      point.time_stamp = getPointTimeRelative(packet_idx, block_idx, unit_idx, return_mode);

      // 6. Add point metadata
      point.return_type = static_cast<uint8_t>(return_type);
      point.channel = sensor_.getChannel(packet, block_idx, unit_idx);

      // 7. Do angle correction
      const auto raw_azimuth = sensor_.getRawAzimuth(packet, block_idx, unit_idx);
      const auto raw_elevation = sensor_.getRawElevation(packet, block_idx, unit_idx);
      auto corrected_angle_data = sensor_.getCorrectedAngleData(raw_azimuth, raw_elevation);

      // The driver wrapper converts to degrees, expects radians
      point.azimuth = corrected_angle_data.azimuth_rad;
      point.elevation = corrected_angle_data.elevation_rad;

      // 8. Convert to cartesian coordinates
      // The raw_azimuth and channel are only used as indices, sin/cos functions use the precise
      // corrected angles
      float xyDistance = distance * corrected_angle_data.cos_elevation;
      point.x = xyDistance * corrected_angle_data.cos_azimuth;
      point.y = xyDistance * corrected_angle_data.sin_azimuth;
      point.z = distance * corrected_angle_data.sin_elevation;

      decode_pc_->emplace_back(point);
    }
  }

  /// @brief Get timestamp of point in nanoseconds, relative to scan timestamp. Includes firing time
  /// offset correction for channel and block
  /// @param packet_idx The index of the packet in the current decode group
  /// @param block_idx The block index of the point
  /// @param unit_idx The channel index of the point
  /// @param return_mode The currently active return mode
  uint32_t getPointTimeRelative(
    size_t packet_idx, size_t block_idx, size_t unit_idx,
    ReturnMode return_mode)
  {
    const auto point_to_packet_offset_ns =
      sensor_.getPacketRelativeTimestamp(decode_group_[packet_idx], block_idx, unit_idx, return_mode);
    const auto packet_timestamp_ns = decode_group_timestamps_[packet_idx];
    auto packet_to_scan_offset_ns =
      static_cast<uint32_t>(packet_timestamp_ns - decode_scan_timestamp_ns_);
    return packet_to_scan_offset_ns + point_to_packet_offset_ns;
  }

  void onScanCompleted(size_t packet_id, size_t block_id)
  {
    std::swap(decode_pc_, output_pc_);
    decode_pc_->clear();
    has_scanned_ = true;
    output_scan_timestamp_ns_ = decode_scan_timestamp_ns_;

    // A new scan starts within the current packet, so the new scan's timestamp must be
    // calculated as the packet timestamp plus the lowest time offset of any point in the
    // remainder of the packet
    decode_scan_timestamp_ns_ =
      decode_group_timestamps_[packet_id] +
      sensor_.getEarliestPointTimeOffsetForBlock(block_id, sensor_configuration_);
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  /// calibration_configuration is set)
  explicit RobosenseDecoder(
    const std::shared_ptr<RobosenseSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const SensorT> & sensor)
  : sensor_configuration_(sensor_configuration),
    sensor_(*sensor),
    decode_group_(decode_group_size_),
    logger_(rclcpp::get_logger("RobosenseDecoder")),
    clock_(RCL_STEADY_TIME)
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO_STREAM(logger_, sensor_configuration_);

    decode_pc_.reset(new NebulaPointCloud);
    output_pc_.reset(new NebulaPointCloud);

    decode_pc_->reserve(SensorT::MAX_SCAN_BUFFER_POINTS);
    output_pc_->reserve(SensorT::MAX_SCAN_BUFFER_POINTS);
  }

  int unpack(const robosense_msgs::msg::RobosensePacket & msop_packet) override
  {
    std::lock_guard<std::mutex> lock(sensor_configuration_mutex_);
    if (sensor_configuration_ == nullptr) {
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 1000, "Sensor configuration not set, cannot decode packet.");
      return false;
    }

    if (!parsePacket(msop_packet)) {
      return -1;
    }

    if (decode_scan_timestamp_ns_ == 0) {
      decode_scan_timestamp_ns_ = decode_group_timestamps_.back();
    }

    if (has_scanned_) {
      has_scanned_ = false;
    }

    // For the dual return mode, the packet contains two blocks with the same azimuth, one for each
    // return. For the single return mode, the packet contains only one block per azimuth.
    // So, if the return mode is dual, we process two blocks per iteration, otherwise one.
    const ReturnMode return_mode = sensor_.getReturnMode(decode_group_[0], *sensor_configuration_);
    const size_t n_returns = ReturnModeToNReturns(return_mode);

    // The advance/stride BETWEEN return groups. Not to be confused with RETURN_GROUP_STRIDE, which
    // is the stride between units WITHIN a return group.
    std::array<size_t, SensorT::RETURN_GROUP_STRIDE.size()> advance;
    std::transform(
      SensorT::RETURN_GROUP_STRIDE.begin(), SensorT::RETURN_GROUP_STRIDE.end(), advance.begin(),
      [=](bool is_strided_dimension) { return is_strided_dimension ? n_returns : 1; });

    for (size_t packet_id = 0; packet_id < decode_group_.size(); packet_id += advance[0]) {
      for (size_t block_id = 0; block_id < SensorT::packet_t::N_BLOCKS; block_id += advance[1]) {
        bool scan_completed = sensor_.checkScanCompleted(decode_group_[packet_id], block_id);
        if (scan_completed) {
          onScanCompleted(packet_id, block_id);
        }
        for (size_t channel_id = 0; channel_id < SensorT::packet_t::N_CHANNELS;
             channel_id += advance[2]) {
          convertReturnGroup(packet_id, block_id, channel_id, n_returns, return_mode);
        }
      }
    }

    return 0;  // TODO(mojomex): azimuth has no meaning for some sensors, what to return instead?
               // #Points decoded?
  }

  bool hasScanned() override { return has_scanned_; }

  std::tuple<drivers::NebulaPointCloudPtr, double> getPointcloud() override
  {
    double scan_timestamp_s = static_cast<double>(output_scan_timestamp_ns_) * 1e-9;
    return std::make_pair(output_pc_, scan_timestamp_s);
  }

  void updateSensorConfiguration(
    const std::shared_ptr<const drivers::RobosenseSensorConfiguration> & sensor_configuration)
    override
  {
    std::lock_guard<std::mutex> lock(sensor_configuration_mutex_);
    sensor_configuration_ = sensor_configuration;
  }
};

}  // namespace drivers
}  // namespace nebula