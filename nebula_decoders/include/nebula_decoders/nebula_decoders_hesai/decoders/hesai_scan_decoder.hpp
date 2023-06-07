#ifndef NEBULA_WS_HESAI_SCAN_DECODER_HPP
#define NEBULA_WS_HESAI_SCAN_DECODER_HPP

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/point_types.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <tuple>

namespace nebula
{
namespace drivers
{
/// @brief Base class for Hesai LiDAR decoder
class HesaiScanDecoder
{
protected:
  /// @brief Decoded point cloud
  drivers::NebulaPointCloudPtr scan_pc_;
  /// @brief Point cloud overflowing from one cycle
  drivers::NebulaPointCloudPtr overflow_pc_;

  uint16_t scan_phase_{};
  int last_phase_{};
  bool has_scanned_{};
  double dual_return_distance_threshold_{};
  double scan_timestamp_{};

  /// @brief SensorConfiguration for this decoder
  std::shared_ptr<drivers::HesaiSensorConfiguration> sensor_configuration_;
  /// @brief Calibration for this decoder
  std::shared_ptr<drivers::HesaiCalibrationConfiguration> sensor_calibration_;

  /// @brief Converts an input degree value to the radian value
  /// @param degrees
  /// @return radians
  static inline float deg2rad(double degrees) { return degrees * M_PI / 180.0; }

public:
  HesaiScanDecoder(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder & operator=(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder(const HesaiScanDecoder & c) = delete;
  HesaiScanDecoder & operator=(const HesaiScanDecoder & c) = delete;

  virtual ~HesaiScanDecoder() = default;
  HesaiScanDecoder() = default;

  /// @brief Virtual function for parsing and shaping PandarPacket
  /// @param pandar_packet
  virtual int unpack(const pandar_msgs::msg::PandarPacket & pandar_packet) = 0;

  /// @brief Virtual function for parsing PandarPacket based on packet structure
  /// @param pandar_packet
  /// @return Resulting flag
  virtual bool parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet) = 0;

  /// @brief Virtual function for converting to point cloud
  /// @param block_id target block
  /// @return Point cloud
  virtual drivers::NebulaPointCloudPtr convert(size_t block_id) = 0;
  /// @brief Virtual function for converting to point cloud for dual return
  /// @param block_id target block
  /// @return Point cloud
  virtual drivers::NebulaPointCloudPtr convert_dual(size_t block_id) = 0;

  /// @brief Virtual function for getting the flag indicating whether one cycle is ready
  /// @return Readied
  virtual bool hasScanned() = 0;

  /// @brief Virtual function for getting the constructed point cloud
  /// @return tuple of Point cloud and timestamp
  virtual std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() = 0;
};
}  // namespace drivers
}  // namespace nebula
#endif  // NEBULA_WS_HESAI_SCAN_DECODER_HPP
