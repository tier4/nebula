#pragma once

#include <array>

#include "hesai/decoders/pandar_xtm.hpp"
#include "hesai_scan_decoder.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace drivers
{
namespace pandar_xtm
{
/*
const float pandarXTM_elev_angle_map[] = {
  19.5f, 18.2f, 16.9f, 15.6f, 14.3f, 13.0f, 11.7f, 10.4f, \
  9.1f,  7.8f,  6.5f,  5.2f,  3.9f,  2.6f,  1.3f, 0.0f,  \
  -1.3f, -2.6f, -3.9f, -5.2f, -6.5f, -7.8f, -9.1f, -10.4f, \
  -11.7f, -13.0f, -14.3f, -15.6f, -16.9f, -18.2f, -19.5f, -20.8f
};

const float pandarXTM_horizontal_azimuth_offset_map[] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};
*/
/*
const float pandarXTM_elev_angle_map[] = {
  19.407f, 18.106f, 16.801f, 15.512f, 14.214f, 12.917f, 11.661f, 10.362f, \
  9.066f,  7.769f,  6.475f,  5.181f,  3.885f,  2.595f,  1.299f, -0.005f,  \
  -1.310f, -2.605f, -3.905f, -5.204f, -6.496f, -7.792f, -9.087f, -10.386f, \
  -11.685f, -12.955f, -14.242f, -15.537f, -16.834f, -18.129f, -19.424f, -20.720f
};

const float pandarXTM_horizontal_azimuth_offset_map[] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};
*/

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
  /// @return Point cloud
  drivers::PointCloudXYZIRADTPtr get_pointcloud() override;

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
    int blockid, char chLaserNumber, boost::shared_ptr<pcl::PointCloud<PointXYZIRADT>> cld);
#else
  /// @brief Constructing a point cloud of the target part
  /// @param blockid Target block
  /// @param chLaserNumber Target laser
  /// @param cld Point cloud
  void CalcXTPointXYZIT(
    int blockid, char chLaserNumber, std::shared_ptr<pcl::PointCloud<PointXYZIRADT>> cld);
#endif

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

  //  std::array<float, LASER_COUNT> firing_offset_{};

  std::array<float, BLOCKS_PER_PACKET> block_offset_single_{};
  std::array<float, BLOCKS_PER_PACKET> block_offset_dual_{};
  std::array<float, BLOCKS_PER_PACKET> block_offset_triple_{};

  std::vector<float> m_sin_elevation_map_;
  std::vector<float> m_cos_elevation_map_;

  std::vector<float> m_sin_azimuth_map_;
  std::vector<float> m_cos_azimuth_map_;

  Packet packet_{};

  uint16_t last_azimuth_;
  int start_angle_;
  double last_timestamp_;
};

}  // namespace pandar_xtm
}  // namespace drivers
}  // namespace nebula