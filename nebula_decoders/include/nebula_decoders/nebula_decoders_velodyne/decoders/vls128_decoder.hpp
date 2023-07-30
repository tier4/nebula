#pragma once

#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_scan_decoder.hpp"

#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

#include <array>

namespace nebula
{
namespace drivers
{
namespace vls128
{
/// @brief Velodyne LiDAR decoder (VLS128)
class Vls128Decoder : public VelodyneScanDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  explicit Vls128Decoder(
    const std::shared_ptr<drivers::VelodyneSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::VelodyneCalibrationConfiguration> & calibration_configuration);
  /// @brief Parsing and shaping VelodynePacket
  /// @param velodyne_packet
  void unpack(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) override;
  /// @brief Get the flag indicating whether one cycle is ready
  /// @return Readied
  bool hasScanned() override;
  /// @brief Calculation of points in each packet
  /// @return # of points
  int pointsPerPacket() override;
  /// @brief Get the constructed point cloud
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() override;
  /// @brief Resetting point cloud buffer
  /// @param n_pts # of points
  void reset_pointcloud(size_t n_pts) override;
  /// @brief Resetting overflowed point cloud buffer
  void reset_overflow() override;

private:
  /// @brief Parsing VelodynePacket based on packet structure
  /// @param velodyne_packet
  /// @return Resulting flag
  bool parsePacket(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) override;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];
  float rotation_radians_[ROTATION_MAX_UNITS];
  float vls_128_laser_azimuth_cache_[16];
  int phase_;
  int max_pts_;
  std::vector<std::vector<float>> timing_offsets_;
};

}  // namespace vls128
}  // namespace drivers
}  // namespace nebula
