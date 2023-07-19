#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_at.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <array>

namespace nebula
{
namespace drivers
{
namespace pandar_at
{
const uint32_t channel_firing_ns[] = {
  0,     0,     8240,  4112,  4144,  8240,  0,     0,     12424, 4144,  4112,  8264,  12376,
  12376, 8264,  12424, 0,     0,     4112,  8240,  4144,  0,     0,     4144,  12424, 8264,
  4112,  12376, 12376, 12424, 8264,  848,   2504,  4976,  6616,  6616,  9112,  2504,  848,
  10768, 13280, 13280, 4976,  9112,  14928, 14928, 10768, 2504,  848,   6616,  4976,  9112,
  6616,  848,   2504,  13280, 10768, 4976,  13280, 14928, 9112,  10768, 14928, 13280, 848,
  9112,  13280, 2504,  4976,  848,   2504,  14928, 10768, 10768, 14928, 4976,  6616,  6616,
  9112,  848,   13280, 13280, 9112,  4976,  2504,  2504,  848,   10768, 14928, 14928, 10768,
  6616,  4976,  9112,  6616,  4112,  1242,  0,     4144,  0,     0,     12424, 0,     8264,
  4112,  4144,  8240,  8240,  8264,  12376, 12376, 12424, 4112,  4144,  0,     0,     0,
  0,     0,     12424, 8264,  8240,  4144,  8264,  8240,  12376, 12376, 8264};

const uint16_t MAX_AZIMUTH_DEGREE_NUM = 36000;

const uint16_t LIDAR_AZIMUTH_UNIT = 256;
const uint32_t MAX_AZI_LEN = 36000 * 256;

/// @brief Hesai LiDAR decoder (AT128)
class PandarATDecoder : public HesaiScanDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  /// @param correction_configuration Correction for this decoder
  explicit PandarATDecoder(
    const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration,
    const std::shared_ptr<drivers::HesaiCorrection> & correction_configuration);
  /// @brief Parsing and shaping PandarPacket
  /// @param pandar_packet
  int unpack(const pandar_msgs::msg::PandarPacket & raw_packet) override;
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
  /// @param block_id Target block
  /// @param chLaserNumber Target laser
  /// @param cld Point cloud
  void CalcXTPointXYZIT(
    int block_id, int chLaserNumber, boost::shared_ptr<pcl::PointCloud<NebulaPoint>> cld);
#else
  /// @brief Constructing a point cloud of the target part
  /// @param block_id Target block
  /// @param chLaserNumber Target laser
  /// @param cld Point cloud
  void CalcXTPointXYZIT(
    int block_id, int chLaserNumber, NebulaPointCloudPtr & cld);
#endif

  /// @brief Convert to point cloud
  /// @param block_id target block
  /// @return Point cloud
  drivers::NebulaPointCloudPtr convert(size_t block_id) override;
  void convert2(size_t block_id, NebulaPointCloudPtr & out_pc);
  /// @brief Convert to point cloud for dual return
  /// @param block_id target block
  /// @return Point cloud
  drivers::NebulaPointCloudPtr convert_dual(size_t block_id) override;

  std::array<float, LASER_COUNT> elev_angle_{};
  std::array<float, LASER_COUNT> azimuth_offset_{};

  //  std::array<float, LASER_COUNT> firing_offset_{};

  std::array<float, BLOCKS_PER_PACKET> block_offset_single_{};
  std::array<float, BLOCKS_PER_PACKET> block_offset_dual_{};
  std::array<float, BLOCKS_PER_PACKET> block_offset_triple_{};

  std::vector<float> m_sin_map_;
  std::vector<float> m_cos_map_;

  Packet packet_{};

  int last_azimuth_;
  int max_azimuth_;
  uint16_t last_field_;
  int start_angle_;
  double last_timestamp_;

  /// @brief Correction data for this decoder (Only AT)
  std::shared_ptr<drivers::HesaiCorrection> correction_configuration_;

  bool use_dat = true;
};

}  // namespace pandar_at
}  // namespace drivers
}  // namespace nebula
