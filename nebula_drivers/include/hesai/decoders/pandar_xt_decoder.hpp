#pragma once

#include <array>

#include "hesai/decoders/pandar_xt.hpp"
#include "hesai_scan_decoder.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace drivers
{
namespace pandar_xt
{
/// @brief Hesai LiDAR decorder (XT32)
class PandarXTDecoder : public HesaiScanDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  explicit PandarXTDecoder(
    const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration);
  /// @brief Parsing and shaping PandarPacket
  /// @param pandar_packet
  void unpack(const pandar_msgs::msg::PandarPacket & raw_packet) override;
  /// @brief Get the flag indicating whether one cycle is ready
  /// @return Readied
  bool hasScanned() override;
  /// @brief Get the constructed point cloud
  /// @return Point cloud
  drivers::PointCloudXYZIRADTPtr get_pointcloud() override;

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
  drivers::PointXYZIRADT build_point(int block_id, int unit_id, uint8_t return_type);
  /// @brief Convert to point cloud
  /// @param block_id target block
  /// @return Point cloud
  drivers::PointCloudXYZIRADTPtr convert(size_t block_id) override;
  /// @brief Convert to point cloud for dual return
  /// @param block_id target block
  /// @return Point cloud
  drivers::PointCloudXYZIRADTPtr convert_dual(size_t block_id) override;

  std::array<float, LASER_COUNT> elev_angle_{};
  std::array<float, LASER_COUNT> azimuth_offset_{};

  std::array<float, LASER_COUNT> firing_offset_{};

  std::array<float, BLOCKS_PER_PACKET> block_offset_single_{};
  std::array<float, BLOCKS_PER_PACKET> block_offset_dual_{};

  Packet packet_{};
};

}  // namespace pandar_xt
}  // namespace drivers
}  // namespace nebula