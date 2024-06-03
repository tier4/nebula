#ifndef NEBULA_WS_VELODYNE_SCAN_DECODER_HPP
#define NEBULA_WS_VELODYNE_SCAN_DECODER_HPP
#include "nebula_common/point_types.hpp"
#include "nebula_common/velodyne/velodyne_calibration_decoder.hpp"
#include "nebula_common/velodyne/velodyne_common.hpp"

#include <rclcpp/rclcpp.hpp>

#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <boost/format.hpp>

#include <pcl/point_cloud.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

namespace nebula
{
namespace drivers
{
/// @brief Base class for Velodyne LiDAR decoder
class VelodyneScanDecoder
{
protected:
  /// @brief Decoded point cloud
  drivers::NebulaPointCloudPtr scan_pc_;
  /// @brief Point cloud overflowing from one cycle
  drivers::NebulaPointCloudPtr overflow_pc_;

  uint16_t scan_phase_{};
  uint16_t last_phase_{};
  bool has_scanned_ = true;
  double dual_return_distance_threshold_{};  // Velodyne does this internally, this will not be
                                             // implemented here
  double scan_timestamp_{};

  /// @brief SensorConfiguration for this decoder
  std::shared_ptr<drivers::VelodyneSensorConfiguration> sensor_configuration_;
  /// @brief Calibration for this decoder
  std::shared_ptr<drivers::VelodyneCalibrationConfiguration> calibration_configuration_;

public:
  VelodyneScanDecoder(VelodyneScanDecoder && c) = delete;
  VelodyneScanDecoder & operator=(VelodyneScanDecoder && c) = delete;
  VelodyneScanDecoder(const VelodyneScanDecoder & c) = delete;
  VelodyneScanDecoder & operator=(const VelodyneScanDecoder & c) = delete;

  virtual ~VelodyneScanDecoder() = default;
  VelodyneScanDecoder() = default;

  /// @brief Virtual function for parsing and shaping VelodynePacket
  /// @param pandar_packet
  virtual void unpack(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) = 0;
  /// @brief Virtual function for parsing VelodynePacket based on packet structure
  /// @param pandar_packet
  /// @return Resulting flag
  virtual bool parsePacket(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) = 0;

  /// @brief Virtual function for getting the flag indicating whether one cycle is ready
  /// @return Readied
  virtual bool hasScanned() = 0;

  /// @brief Virtual function for getting the constructed point cloud
  /// @return tuple of Point cloud and timestamp
  virtual std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() = 0;
  /// @brief Resetting point cloud buffer
  /// @param n_pts # of points
  virtual void reset_pointcloud(size_t n_pts, double time_stamp) = 0;
  /// @brief Resetting overflowed point cloud buffer
  virtual void reset_overflow(double time_stamp) = 0;
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_WS_VELODYNE_SCAN_DECODER_HPP
