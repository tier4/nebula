#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace drivers
{
template <typename PacketT>
class HesaiDecoder : public HesaiScanDecoder
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<drivers::HesaiSensorConfiguration> sensor_configuration_;

  /// @brief Decodes azimuth/elevation angles given calibration/correction data
  typename PacketT::angle_decoder_t angle_corrector_;

  /// @brief The point cloud new points get added to
  NebulaPointCloudPtr decode_pc_;
  /// @brief The point cloud that is returned when a scan is complete
  NebulaPointCloudPtr output_pc_;

  /// @brief The last decoded packet
  PacketT packet_;
  /// @brief The last azimuth processed
  int last_phase_;
  /// @brief The timestamp of the last scan in nanoseconds
  uint64_t scan_timestamp_ns_;
  /// @brief Whether a full scan has been processed
  bool has_scanned_;

  /// @brief For each channel, its firing offset relative to the block in nanoseconds
  std::array<uint32_t, PacketT::N_CHANNELS> channel_firing_offset_ns_;
  /// @brief For each return mode, the firing offset of each block relative to its packet in
  /// nanosconds
  std::array<std::array<uint32_t, PacketT::N_BLOCKS>, PacketT::MAX_RETURNS> block_firing_offset_ns_;

  /// @brief Validates and parse PandarPacket. Currently only checks size, not checksums etc.
  /// @param pandar_packet The incoming PandarPacket
  /// @return Whether the packet was parsed successfully
  bool parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet);

  /// @brief Converts the given block(s) of the last incoming PandarPacket to point cloud and append
  /// the decoded points to @ref decode_pc_
  /// @param start_block_id The first block to convert
  /// @param n_blocks The number of blocks to convert (this is expected to align with the number of
  /// returns)
  void convertReturns(size_t start_block_id, size_t n_blocks);

  /// @brief Checks whether the last processed block was the last block of a scan
  /// @param current_phase The azimuth of the last processed block
  /// @return Whether the scan has completed
  bool checkScanCompleted(int current_phase);

  /// @brief Get the distance of the given unit in millimeters
  float getDistance(typename PacketT::body_t::block_t::unit_t & unit);

  /// @brief Get timestamp of point in nanoseconds, relative to scan timestamp. Includes firing time
  /// offset correction for channel and block
  /// @param packet_timestamp_ns The timestamp of the current PandarPacket in nanoseconds
  /// @param block_id The block index of the point
  /// @param channel_id The channel index of the point
  uint32_t getPointTimeRelative(uint64_t packet_timestamp_ns, size_t block_id, size_t channel_id);

  /// @brief Get the time offset of the given channel in nanoseconds, relative to the block
  uint32_t getChannelTimeOffset(size_t channel_id);

  /// @brief Get the time offset of the given block in nanoseconds, relative to the packet. The
  /// offset returned depends on the return mode of the current PandarPacket
  uint32_t getBlockTimeOffset(size_t block_id);

  /// @brief Get the return type of the given block
  /// @param block_id The index of the block in the current PandarPacket
  /// @return The block's return type
  ReturnType getReturnType(size_t block_id);

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
    const std::shared_ptr<HesaiCorrection> & correction_configuration);

  int unpack(const pandar_msgs::msg::PandarPacket & pandar_packet) override;

  bool hasScanned() override;

  std::tuple<drivers::NebulaPointCloudPtr, double> getPointcloud() override;
};

}  // namespace drivers
}  // namespace nebula
