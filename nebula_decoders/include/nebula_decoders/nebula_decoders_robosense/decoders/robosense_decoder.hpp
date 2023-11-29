#pragma once

#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_scan_decoder.hpp"

#include <rclcpp/rclcpp.hpp>

#include "robosense_msgs/msg/robosense_packet.hpp"
#include "robosense_msgs/msg/robosense_scan.hpp"

namespace nebula
{
namespace drivers
{
template <typename SensorT>
class RobosenseDecoder : public RobosenseScanDecoder
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<drivers::RobosenseSensorConfiguration> sensor_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  /// @brief Decodes azimuth/elevation angles given calibration/correction data
  typename SensorT::angle_corrector_t angle_corrector_;
  /// @brief Computes the exact timestamp of each point
  typename SensorT::timing_corrector_t timing_corrector_;
  /// @brief Checks whether a scan is complete at every block
  typename SensorT::scan_completion_checker_t scan_completion_checker_;

  /// @brief The point cloud new points get added to
  NebulaPointCloudPtr decode_pc_;
  /// @brief The point cloud that is returned when a scan is complete
  NebulaPointCloudPtr output_pc_;

  /// @brief The current group of packets being decoded. In most cases this is just the latest
  /// packet received, but for some sensors, return groups can be split across packets.
  typename SensorT::decode_group_t decode_group_;
  /// @brief The last azimuth processed
  int last_phase_;
  /// @brief The timestamp of the last completed scan in nanoseconds
  uint64_t output_scan_timestamp_ns_;
  /// @brief The timestamp of the scan currently in progress
  uint64_t decode_scan_timestamp_ns_;
  /// @brief Whether a full scan has been processed
  bool has_scanned_;

  rclcpp::Logger logger_;

  /// @brief Validates and parses MsopPacket. Currently only checks size, not checksums etc.
  /// @param msop_packet The incoming MsopPacket
  /// @return Whether the packet was parsed successfully
  bool parsePacket(const robosense_msgs::msg::RobosensePacket & msop_packet)
  {
    if (msop_packet.data.size() < sizeof(typename SensorT::packet_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch:" << msop_packet.data.size() << " | Expected at least:"
                                         << sizeof(typename SensorT::packet_t));
      return false;
    }
    if (std::memcpy(&packets_, msop_packet.data.data(), sizeof(typename SensorT::packet_t))) {
      return true;
    }

    RCLCPP_ERROR(logger_, "Packet memcopy failed");
    return false;
  }

  /// @brief Converts a group of returns (i.e. 1 for single return, 2 for dual return, etc.) to
  /// points and appends them to the point cloud
  /// @param start_packet_id The index of the first packet in the current decode group
  /// @param start_block_id The first block in the group of returns
  /// @param start_channel_id The first channel in the return group
  /// @param n_returns The number of returns in the group
  void convertReturnGroup(
    size_t start_packet_id, size_t start_block_id, size_t start_channel_id, size_t n_returns)
  {
    std::array<const typename SensorT::packet_t::body_t::block_t::unit_t *, n_returns> return_units;
    std::array<size_t, n_returns> packet_idxs;
    std::array<size_t, n_returns> block_idxs;

    // Find the units corresponding to the same return group as the current one.
    // These are used to find duplicates in multi-return mode.
    for (size_t return_idx = 0; return_idx < n_returns; ++return_idx) {
      size_t packet_idx = start_packet_id + return_idx * SensorT::RETURN_GROUP_STRIDE[0];
      size_t block_idx = start_block_id + return_idx * SensorT::RETURN_GROUP_STRIDE[1];
      size_t channel_idx = start_channel_id + return_idx * SensorT::RETURN_GROUP_STRIDE[2];

      packet_idxs[return_idx] = packet_idx;
      block_idxs[return_idx] = block_idx;

      return_units[return_idx] = &packets_[packet_idx].body.blocks[block_idx].units[channel_idx];
    }

    // For each return unit, validate it and check if it is a duplicate of any other
    // unit in the return group. If not, convert it to a point and add it to the point cloud.
    for (size_t return_idx = 0; return_idx < n_returns; ++return_idx) {
      uint64_t packet_timestamp_ns =
        robosense_packet::get_timestamp_ns(packets_[packet_idxs[return_idx]]);
      uint32_t raw_azimuth = packets_.body.blocks[block_idxs[return_idx]].get_azimuth();
      auto & unit = *return_units[return_idx];

      // 1. Validate point
      if (unit.distance.value() == 0) {
        continue;
      }

      // 2. Range checks
      auto distance = getDistance(unit);

      if (distance < SensorT::MIN_RANGE || distance > SensorT::MAX_RANGE) {
        continue;
      }

      // 3. Return group duplicate checks
      auto return_type =
        sensor_.getReturnType(sensor_configuration_->return_mode, return_idx, return_units);

      // Keep only last of multiple identical points
      if (return_type == ReturnType::IDENTICAL && return_idx != n_returns - 1) {
        continue;
      }

      // Keep only last (if any) of multiple points that are too close
      if (return_idx != n_returns - 1) {
        bool is_below_multi_return_threshold = false;

        for (size_t return_idx = 0; return_idx < n_returns; ++return_idx) {
          if (return_idx == return_idx) {
            continue;
          }

          if (
            fabsf(getDistance(*return_units[return_idx]) - distance) <
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
      point.intensity = unit.reflectivity.value();
      
      // 5. Do timing correction
      point.time_stamp =
        getPointTimeRelative(packet_timestamp_ns, return_idx + start_block_id, start_channel_id);

      // 6. Add point metadata
      point.return_type = static_cast<uint8_t>(return_type);
      point.channel = start_channel_id; //TODO(mojomex): get correct channel

      // 7. Do angle correction
      auto corrected_angle_data =
        angle_corrector_.getCorrectedAngleData(start_block_id + return_idx, start_channel_id);

      // The driver wrapper converts to degrees, expects radians
      point.azimuth = corrected_angle_data.azimuth_rad;
      point.elevation = corrected_angle_data.elevation_rad;
      
      // 8. Convert to cartesian coordinates
      // The raw_azimuth and channel are only used as indices, sin/cos functions use the precise
      // corrected angles
      float xyDistance = distance * corrected_angle_data.cos_elevation;
      point.x = xyDistance * corrected_angle_data.cos_azimuth;
      point.y = -xyDistance * corrected_angle_data.sin_azimuth;
      point.z = distance * corrected_angle_data.sin_elevation;

      decode_pc_->emplace_back(point);
    }
  }

  /// @brief Get the distance of the given unit in meters
  /// @param unit The unit to get the distance from
  /// @return The distance in meters
  float getDistance(const typename SensorT::packet_t::body_t::block_t::unit_t & unit)
  {
    return unit.distance.value() * robosense_packet::get_dis_unit(packets_);
  }

  /// @brief Get timestamp of point in nanoseconds, relative to scan timestamp. Includes firing time
  /// offset correction for channel and block
  /// @param packet_timestamp_ns The timestamp of the current MsopPacket in nanoseconds
  /// @param block_id The block index of the point
  /// @param channel_id The channel index of the point
  uint32_t getPointTimeRelative(uint64_t packet_timestamp_ns, size_t block_id, size_t channel_id)
  {
    auto point_to_packet_offset_ns =
      sensor_.getPacketRelativePointTimeOffset(block_id, channel_id, sensor_configuration_);
    auto packet_to_scan_offset_ns =
      static_cast<uint32_t>(packet_timestamp_ns - decode_scan_timestamp_ns_);
    return packet_to_scan_offset_ns + point_to_packet_offset_ns;
  }

  void onScanCompleted()
  {
    std::swap(decode_pc_, output_pc_);
    decode_pc_->clear();
    has_scanned_ = true;
    output_scan_timestamp_ns_ = decode_scan_timestamp_ns_;

    // A new scan starts within the current packet, so the new scan's timestamp must be
    // calculated as the packet timestamp plus the lowest time offset of any point in the
    // remainder of the packet
    decode_scan_timestamp_ns_ =
      robosense_packet::get_timestamp_ns(packets_) +
      sensor_.getEarliestPointTimeOffsetForBlock(block_id, sensor_configuration_);
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  /// calibration_configuration is set)
  explicit RobosenseDecoder(
    const std::shared_ptr<RobosenseSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<RobosenseCalibrationConfiguration> & calibration_configuration)
  : sensor_configuration_(sensor_configuration),
    angle_corrector_(calibration_configuration),
    logger_(rclcpp::get_logger("RobosenseDecoder"))
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
    if (!parsePacket(msop_packet)) {
      return -1;
    }

    if (decode_scan_timestamp_ns_ == 0) {
      decode_scan_timestamp_ns_ = robosense_packet::get_timestamp_ns(packets_);
    }

    if (has_scanned_) {
      has_scanned_ = false;
    }

    // For the dual return mode, the packet contains two blocks with the same azimuth, one for each
    // return. For the single return mode, the packet contains only one block per azimuth.
    // So, if the return mode is dual, we process two blocks per iteration, otherwise one.
    const size_t n_returns = robosense_packet::get_n_returns(sensor_configuration_->return_mode);
    int current_azimuth;

    // The advance/stride BETWEEN return groups. Not to be confused with RETURN_GROUP_STRIDE, which
    // is the stride between units WITHIN a return group.
    std::array<size_t, SensorT::RETURN_GROUP_STRIDE.size()> advance;
    std::transform(
      SensorT::RETURN_GROUP_STRIDE.begin(), SensorT::RETURN_GROUP_STRIDE.end(), advance.begin(),
      [=](bool is_strided_dimension) { return is_strided_dimension ? n_returns : 1; });

    for (size_t packet_id = 0; packet_id < decode_group_.size(); packet_id += advance[0]) {
      for (size_t block_id = 0; block_id < SensorT::packet_t::N_BLOCKS; block_id += advance[1]) {
        bool scan_completed =
          scan_completion_checker_.checkScanCompleted(packets_[packet_id], block_id);
        if (scan_completed) {
          onScanCompleted();
        }
        for (size_t channel_id = 0; channel_id < SensorT::packet_t::N_CHANNELS;
             channel_id += advance[2]) {
          convertReturnGroup(packet_id, block_id, channel_id, n_returns);
        }
      }
    }

    return 0; // TODO(mojomex): azimuth has no meaning for some sensors, what to return instead? #Points decoded?
  }

  bool hasScanned() override { return has_scanned_; }

  std::tuple<drivers::NebulaPointCloudPtr, double> getPointcloud() override
  {
    double scan_timestamp_s = static_cast<double>(output_scan_timestamp_ns_) * 1e-9;
    return std::make_pair(output_pc_, scan_timestamp_s);
  }
};

}  // namespace drivers
}  // namespace nebula