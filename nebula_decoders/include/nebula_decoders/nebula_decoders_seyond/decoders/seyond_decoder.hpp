#pragma once

#include "nebula_decoders/nebula_decoders_seyond/decoders/seyond_packet.hpp"
#include "nebula_decoders/nebula_decoders_seyond/decoders/seyond_scan_decoder.hpp"

#include <rclcpp/rclcpp.hpp>

#include "nebula_msgs/msg/nebula_packet.hpp"
#include "nebula_msgs/msg/nebula_packets.hpp"

namespace nebula
{
namespace drivers
{

template <typename SensorT>
class SeyondDecoder : public SeyondScanDecoder
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<const drivers::SeyondSensorConfiguration> sensor_configuration_;

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
  int last_phase_ = -1; // Dummy value to signal last_phase_ has not been set yet
  /// @brief The timestamp of the last completed scan in nanoseconds
  uint64_t output_scan_timestamp_ns_ = 0;
  /// @brief The timestamp of the scan currently in progress
  uint64_t decode_scan_timestamp_ns_ = 0;
  /// @brief Whether a full scan has been processed
  bool has_scanned_ = false;

  rclcpp::Logger logger_;

  /// @brief For each channel, its firing offset relative to the block in nanoseconds
  std::array<int, SensorT::packet_t::N_CHANNELS> channel_firing_offset_ns_;
  /// @brief For each return mode, the firing offset of each block relative to its packet in
  /// nanoseconds
  std::array<std::array<int, SensorT::packet_t::N_BLOCKS>, SensorT::packet_t::MAX_RETURNS>
    block_firing_offset_ns_;

  /// @brief Validates and parse NebulaPacket. Currently only checks size, not checksums etc.
  /// @param packet The incoming NebulaPacket
  /// @return Whether the packet was parsed successfully
  bool parsePacket(const std::vector<uint8_t> & packet)
  {
    if (packet.size() < sizeof(typename SensorT::packet_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch:" << packet.size() << " | Expected at least:"
                                         << sizeof(typename SensorT::packet_t));
      return false;
    }
    if (std::memcpy(&packet_, packet.data(), sizeof(typename SensorT::packet_t))) {
      // FIXME(mojomex) do validation?
      // RCLCPP_DEBUG(logger_, "Packet parsed successfully");
      return true;
    }

    RCLCPP_ERROR(logger_, "Packet memcopy failed");
    return false;
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param correction_data Calibration data for this decoder
  explicit SeyondDecoder(
    const std::shared_ptr<const SeyondSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const typename SensorT::angle_corrector_t::correction_data_t> & correction_data)
  : sensor_configuration_(sensor_configuration),
    angle_corrector_(correction_data),
    logger_(rclcpp::get_logger("SeyondDecoder"))
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO_STREAM(logger_, *sensor_configuration_);

    decode_pc_.reset(new NebulaPointCloud);
    output_pc_.reset(new NebulaPointCloud);

    decode_pc_->reserve(SensorT::MAX_SCAN_BUFFER_POINTS);
    output_pc_->reserve(SensorT::MAX_SCAN_BUFFER_POINTS);
  }

  int unpack(const std::vector<uint8_t> & packet) override
  {
    return -1;
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
