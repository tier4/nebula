#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_qt_128.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <array>

namespace nebula
{
namespace drivers
{
namespace pandar_qt_128
{
/// @brief Hesai LiDAR decoder (QT128)
class PandarQT128Decoder : public HesaiScanDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  explicit PandarQT128Decoder(
    const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration);
  /// @brief Parsing and shaping PandarPacket
  /// @param pandar_packet
  int unpack(const pandar_msgs::msg::PandarPacket & pandar_packet) override;
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
  /// @param dual_return Return mode is dual
  /// @param unix_second Packet time
  /// @return Point cloud
  drivers::NebulaPoint build_point(
    size_t block_id, size_t unit_id, bool dual_return, const double & unix_second);
  /// @brief Convert to point cloud
  /// @param block_id target block
  /// @return Point cloud
  drivers::NebulaPointCloudPtr convert(size_t block_id) override;
  /// @brief Convert to point cloud for dual return
  /// @param block_id target block
  /// @return Point cloud
  drivers::NebulaPointCloudPtr convert_dual(size_t block_id) override;

  /// @brief Checking packet_.return mode
  /// @return packet_.return mode is dual mode
  bool is_dual_return();

  std::array<float, LASER_COUNT> elevation_angle_{};
  std::array<float, LASER_COUNT> azimuth_offset_{};
  std::array<float, LASER_COUNT> elev_angle_rad_{};
  std::array<float, LASER_COUNT> cos_elevation_angle_{};
  std::array<float, LASER_COUNT> sin_elevation_angle_{};
  std::array<float, LASER_COUNT> elevation_angle_rad_{};
  std::array<float, LASER_COUNT> azimuth_offset_rad_{};

  std::array<float, MAX_AZIMUTH_STEPS> block_azimuth_rad_{};

  std::map<int, float> firing_time_offset1_{};
  std::map<int, float> firing_time_offset2_{};

  std::array<float, BLOCKS_PER_PACKET> block_time_offset_single_{};
  std::array<float, BLOCKS_PER_PACKET> block_time_offset_dual_{};

  uint8_t first_return_type_{};
  uint8_t second_return_type_{};

  Packet packet_{};
};

}  // namespace pandar_qt_128
}  // namespace drivers
}  // namespace nebula
