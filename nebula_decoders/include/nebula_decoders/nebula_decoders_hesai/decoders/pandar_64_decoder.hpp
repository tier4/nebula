#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_64.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <array>

namespace nebula
{
namespace drivers
{
namespace pandar_64
{
/// @brief Hesai LiDAR decorder (Pandar64)
class Pandar64Decoder : public HesaiScanDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  explicit Pandar64Decoder(
    const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration);
  /// @brief Parsing and shaping PandarPacket
  /// @param pandar_packet
  void unpack(const pandar_msgs::msg::PandarPacket & pandar_packet) override;
  /// @brief Get the flag indicating whether one cycle is ready
  /// @return Readied
  bool hasScanned() override;
  /// @brief Get the constructed point cloud
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() override;

private:
  /// @brief Parsing PandarPacket based on packet structure
  /// @param pandar_packet
  /// @return Resulting flag
  bool parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet) override;
  /// @brief Constructing a point cloud of the target part
  /// @param block_id Target block
  /// @param unit_id Target unit
  /// @param return_type Corresponding return mode
  /// @return Point cloud
  drivers::NebulaPoint build_point(size_t block_id, size_t unit_id, uint8_t return_type);
  /// @brief Convert to point cloud
  /// @param block_id target block
  /// @return Point cloud
  drivers::NebulaPointCloudPtr convert(size_t block_id) override;
  /// @brief Convert to point cloud for dual return
  /// @param block_id target block
  /// @return Point cloud
  drivers::NebulaPointCloudPtr convert_dual(size_t block_id) override;

  std::array<float, LASER_COUNT> elevation_angle_{};
  std::array<float, LASER_COUNT> azimuth_offset_{};
  std::array<float, LASER_COUNT> elevation_angle_rad_{};
  std::array<float, LASER_COUNT> azimuth_offset_rad_{};
  std::array<float, LASER_COUNT> cos_elevation_angle_{};
  std::array<float, LASER_COUNT> sin_elevation_angle_{};

  std::array<float, MAX_AZIMUTH_STEPS> block_azimuth_rad_{};

  std::array<float, LASER_COUNT> firing_time_offset_{};

  std::array<float, BLOCKS_PER_PACKET> block_time_offset_single_{};
  std::array<float, BLOCKS_PER_PACKET> block_time_offset_dual_{};

  Packet packet_{};
};

}  // namespace pandar_64
}  // namespace drivers
}  // namespace nebula
