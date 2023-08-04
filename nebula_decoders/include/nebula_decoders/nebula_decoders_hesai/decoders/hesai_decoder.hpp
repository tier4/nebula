#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"

#include <rclcpp/rclcpp.hpp>

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace drivers
{
template <typename SensorT>
class HesaiDecoder : public HesaiScanDecoder
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<drivers::HesaiSensorConfiguration> sensor_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  /// @brief Decodes azimuth/elevation angles given calibration/correction data
  typename SensorT::angle_corrector_t angle_corrector_;

  /// @brief The point cloud new points get added to
  NebulaPointCloudPtr decode_pc_;
  /// @brief The point cloud that is returned when a scan is complete
  NebulaPointCloudPtr output_pc_;

  /// @brief The last decoded packet
  typename SensorT::packet_t packet_;
  /// @brief The last azimuth processed
  int last_phase_;
  /// @brief The timestamp of the last scan in nanoseconds
  uint64_t scan_timestamp_ns_;
  /// @brief Whether a full scan has been processed
  bool has_scanned_;

  rclcpp::Logger logger_;

  /// @brief For each channel, its firing offset relative to the block in nanoseconds
  std::array<int, SensorT::packet_t::N_CHANNELS> channel_firing_offset_ns_;
  /// @brief For each return mode, the firing offset of each block relative to its packet in
  /// nanosconds
  std::array<std::array<int, SensorT::packet_t::N_BLOCKS>, SensorT::packet_t::MAX_RETURNS>
    block_firing_offset_ns_;

  /// @brief Validates and parse PandarPacket. Currently only checks size, not checksums etc.
  /// @param pandar_packet The incoming PandarPacket
  /// @return Whether the packet was parsed successfully
  bool parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet)
  {
    if (pandar_packet.size < sizeof(typename SensorT::packet_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch:" << pandar_packet.size << " | Expected at least:"
                                         << sizeof(typename SensorT::packet_t));
      return false;
    }
    if (std::memcpy(&packet_, pandar_packet.data.data(), sizeof(typename SensorT::packet_t))) {
      // FIXME(mojomex) do validation?
      // RCLCPP_DEBUG(logger_, "Packet parsed successfully");
      return true;
    }

    RCLCPP_ERROR(logger_, "Packet memcopy failed");
    return false;
  }

  /// @brief Converts a group of returns (i.e. 1 for single return, 2 for dual return, etc.) to
  /// points and appends them to the point cloud
  /// @param start_block_id The first block in the group of returns
  /// @param n_blocks The number of returns in the group (has to align with the `n_returns` field in
  /// the packet footer)
  void convertReturns(size_t start_block_id, size_t n_blocks)
  {
    uint64_t packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    uint32_t raw_azimuth = packet_.body.blocks[start_block_id].get_azimuth();

    for (size_t channel_id = 0; channel_id < SensorT::packet_t::N_CHANNELS; ++channel_id) {
      typename SensorT::packet_t::body_t::block_t::unit_t * return_units[n_blocks];

      for (size_t block_id = start_block_id; block_id < start_block_id + n_blocks; ++block_id) {
        return_units[block_id - start_block_id] = &packet_.body.blocks[block_id].units[channel_id];
      }

      for (size_t block_id = start_block_id; block_id < start_block_id + n_blocks; ++block_id) {
        auto & unit = *return_units[block_id - start_block_id];

        auto distance = getDistance(unit);
        if (
          distance < sensor_configuration_->min_range ||
          distance > sensor_configuration_->max_range) {
          continue;
        }

        // TODO(mojomex) handle dual_return_distance_threshold_

        NebulaPoint point;
        point.distance = distance;
        point.intensity = unit.reflectivity;
        // TODO(mojomex) add header offset to scan offset correction
        point.time_stamp = getPointTimeRelative(packet_timestamp_ns, block_id, channel_id);

        auto return_type =
          sensor_.getReturnType(static_cast<hesai_packet::return_mode::ReturnMode>(packet_.tail.return_mode), block_id - start_block_id, return_units);
        point.return_type = static_cast<uint8_t>(return_type);
        point.channel = channel_id;

        auto [azimuth_idx, elevation_idx, azimuth_rad, elevation_rad] =
          angle_corrector_.getCorrectedAzimuthAndElevation(raw_azimuth, channel_id);
        float xyDistance = distance * angle_corrector_.cos_map_[elevation_idx];
        point.x = xyDistance * angle_corrector_.sin_map_[azimuth_idx];
        point.y = xyDistance * angle_corrector_.cos_map_[azimuth_idx];
        point.z = distance * angle_corrector_.sin_map_[elevation_idx];
        point.azimuth = azimuth_rad;
        point.elevation = elevation_rad;

        // NDDUMP(_(point.x), _(point.y), _(point.z));
        decode_pc_->emplace_back(point);
      }
    }
  }

  /// @brief Checks whether the last processed block was the last block of a scan
  /// @param current_phase The azimuth of the last processed block
  /// @return Whether the scan has completed
  bool checkScanCompleted(int current_phase)
  {
    return angle_corrector_.hasScanned(current_phase, last_phase_);
  }

  /// @brief Get the distance of the given unit in millimeters
  float getDistance(typename SensorT::packet_t::body_t::block_t::unit_t & unit)
  {
    return unit.distance * hesai_packet::get_dis_unit(packet_) / 1000.f;
  }

  /// @brief Get timestamp of point in nanoseconds, relative to scan timestamp. Includes firing time
  /// offset correction for channel and block
  /// @param packet_timestamp_ns The timestamp of the current PandarPacket in nanoseconds
  /// @param block_id The block index of the point
  /// @param channel_id The channel index of the point
  uint32_t getPointTimeRelative(uint64_t packet_timestamp_ns, size_t block_id, size_t channel_id)
  {
    auto point_to_packet_offset_ns = sensor_.getPacketRelativePointTimeOffset(block_id, channel_id, packet_);
    auto packet_to_scan_offset_ns = static_cast<uint32_t>(packet_timestamp_ns - scan_timestamp_ns_);
    return packet_to_scan_offset_ns + point_to_packet_offset_ns;
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder (can be nullptr if
  /// correction_configuration is set)
  /// @param correction_configuration Correction for this decoder (can be nullptr if
  /// calibration_configuration is set)
  explicit HesaiDecoder(
    const std::shared_ptr<HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<HesaiCalibrationConfiguration> & calibration_configuration,
    const std::shared_ptr<HesaiCorrection> & correction_configuration)
  : sensor_configuration_(sensor_configuration),
    angle_corrector_(calibration_configuration, correction_configuration),
    logger_(rclcpp::get_logger("HesaiDecoder"))
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_DEBUG(logger_, "Hi from HesaiDecoder");

    decode_pc_.reset(new NebulaPointCloud);
    output_pc_.reset(new NebulaPointCloud);
  }

  int unpack(const pandar_msgs::msg::PandarPacket & pandar_packet) override
  {
    if (!parsePacket(pandar_packet)) {
      return -1;
    }

    // At the start of a scan, set the timestamp to its absolute time since epoch.
    // Point timestamps in the scan are relative to this value.
    if (has_scanned_ || scan_timestamp_ns_ == 0) {
      scan_timestamp_ns_ = hesai_packet::get_timestamp_ns(packet_);
    }

    if (has_scanned_) {
      has_scanned_ = false;
    }

    const size_t n_returns = hesai_packet::get_n_returns(packet_.tail.return_mode);
    int current_azimuth;

    // FIXME(mojomex) for XT32M in triple return mode, blocks 6 and 7 are discarded
    for (size_t block_id = 0; block_id < SensorT::packet_t::N_BLOCKS; block_id += n_returns) {
      // FIXME(mojomex) respect scan phase
      current_azimuth =
        packet_.body.blocks[block_id].get_azimuth() -
        static_cast<int>(
          sensor_configuration_->scan_phase * SensorT::packet_t::DEGREE_SUBDIVISIONS);

      bool scan_completed = checkScanCompleted(current_azimuth);
      if (scan_completed) {
        std::swap(decode_pc_, output_pc_);
        decode_pc_->clear();
        has_scanned_ = true;
      }

      convertReturns(block_id, n_returns);
      last_phase_ = current_azimuth;
      // NDDUMP(
      //   _(n_returns), _(block_id), _(last_phase_), _(current_azimuth), _(scan_completed),
      //   _(decode_pc_->size()),
      //   _(output_pc_->size()));
    }

    return last_phase_;
  }

  bool hasScanned() override { return has_scanned_; }

  std::tuple<drivers::NebulaPointCloudPtr, double> getPointcloud() override
  {
    double scan_timestamp_s = static_cast<double>(scan_timestamp_ns_) * 1e-9;
    // RCLCPP_DEBUG(
    //   logger_, "output_pc_ size: %ld, decode_pc_ size: %ld", output_pc_->size(),
    //   decode_pc_->size());
    return std::make_pair(output_pc_, scan_timestamp_s);
  }
};

}  // namespace drivers
}  // namespace nebula
