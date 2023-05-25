#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xtm.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <array>

namespace nebula
{
namespace drivers
{
namespace pandar_xtm
{
const float blockXTMOffsetTriple[] = {
  5.632f - 50.0f * 1.0f, 5.632f - 50.0f * 1.0f, 5.632f - 50.0f * 1.0f, 5.632f - 50.0f * 0.0f,
  5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f};

const float blockXTMOffsetDual[] = {
  5.632f - 50.0f * 2.0f, 5.632f - 50.0f * 2.0f, 5.632f - 50.0f * 1.0f, 5.632f - 50.0f * 1.0f,
  5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f};
const float blockXTMOffsetSingle[] = {
  5.632f - 50.0f * 5.0f, 5.632f - 50.0f * 4.0f, 5.632f - 50.0f * 3.0f, 5.632f - 50.0f * 2.0f,
  5.632f - 50.0f * 1.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f};

const float laserXTMOffset[] = {
  2.856f * 0.0f + 0.368f,  2.856f * 1.0f + 0.368f,  2.856f * 2.0f + 0.368f,
  2.856f * 3.0f + 0.368f,  2.856f * 4.0f + 0.368f,  2.856f * 5.0f + 0.368f,
  2.856f * 6.0f + 0.368f,  2.856f * 7.0f + 0.368f,

  2.856f * 8.0f + 0.368f,  2.856f * 9.0f + 0.368f,  2.856f * 10.0f + 0.368f,
  2.856f * 11.0f + 0.368f, 2.856f * 12.0f + 0.368f, 2.856f * 13.0f + 0.368f,
  2.856f * 14.0f + 0.368f, 2.856f * 15.0f + 0.368f,

  2.856f * 0.0f + 0.368f,  2.856f * 1.0f + 0.368f,  2.856f * 2.0f + 0.368f,
  2.856f * 3.0f + 0.368f,  2.856f * 4.0f + 0.368f,  2.856f * 5.0f + 0.368f,
  2.856f * 6.0f + 0.368f,  2.856f * 7.0f + 0.368f,

  2.856f * 8.0f + 0.368f,  2.856f * 9.0f + 0.368f,  2.856f * 10.0f + 0.368f,
  2.856f * 11.0f + 0.368f, 2.856f * 12.0f + 0.368f, 2.856f * 13.0f + 0.368f,
  2.856f * 14.0f + 0.368f, 2.856f * 15.0f + 0.368f};

const uint16_t MAX_AZIMUTH_DEGREE_NUM = 36000;

/// @brief Hesai LiDAR decorder (XT32M)
class PandarXTMDecoder : public HesaiScanDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  explicit PandarXTMDecoder(
    const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration);
  /// @brief Parsing and shaping PandarPacket
  /// @param pandar_packet
  void unpack(const pandar_msgs::msg::PandarPacket & raw_packet) override;
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

#if defined(ROS_DISTRO_FOXY) || defined(ROS_DISTRO_GALACTIC)
  /// @brief Constructing a point cloud of the target part
  /// @param blockid Target block
  /// @param chLaserNumber Target laser
  /// @param cld Point cloud
  void CalcXTPointXYZIT(
    int blockid, char chLaserNumber, boost::shared_ptr<pcl::PointCloud<NebulaPoint>> cld);
#else
  /// @brief Constructing a point cloud of the target part
  /// @param blockid Target block
  /// @param chLaserNumber Target laser
  /// @param cld Point cloud
  void CalcXTPointXYZIT(
    int blockid, char chLaserNumber, std::shared_ptr<pcl::PointCloud<NebulaPoint>> cld);
#endif

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

  std::array<float, MAX_AZIMUTH_STEPS> block_azimuth_rad_{};

  std::array<float, BLOCKS_PER_PACKET> block_time_offset_single_return_{};
  std::array<float, BLOCKS_PER_PACKET> block_time_offset_dual_return_{};
  std::array<float, BLOCKS_PER_PACKET> block_time_offset_triple_return_{};

  std::vector<float> sin_elevation_angle_;
  std::vector<float> cos_elevation_angle_;

  std::vector<float> sin_azimuth_angle_;
  std::vector<float> cos_azimuth_angle_;

  Packet packet_{};

  uint16_t last_azimuth_;
  int start_angle_;
  double last_timestamp_;
};

}  // namespace pandar_xtm
}  // namespace drivers
}  // namespace nebula
