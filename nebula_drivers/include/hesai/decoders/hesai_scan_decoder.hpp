#ifndef NEBULA_WS_HESAI_SCAN_DECODER_HPP
#define NEBULA_WS_HESAI_SCAN_DECODER_HPP

#include "hesai/hesai_common.hpp"
#include "common/point_types.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace drivers
{
class HesaiScanDecoder
{
protected:
  drivers::PointCloudXYZIRADTPtr scan_pc_;
  drivers::PointCloudXYZIRADTPtr overflow_pc_;

  uint16_t scan_phase_{};
  int last_phase_{};
  bool has_scanned_{};
  double dual_return_distance_threshold_{};

  std::shared_ptr<drivers::HesaiSensorConfiguration> sensor_configuration_;
  std::shared_ptr<drivers::HesaiCalibrationConfiguration> sensor_calibration_;

  static inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

public:
  HesaiScanDecoder(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder & operator=(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder(const HesaiScanDecoder & c) = delete;
  HesaiScanDecoder & operator=(const HesaiScanDecoder & c) = delete;

  virtual ~HesaiScanDecoder() = default;
  HesaiScanDecoder() = default;

  virtual void unpack(const pandar_msgs::msg::PandarPacket & pandar_packet) = 0;
  virtual bool parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet) = 0;

  virtual drivers::PointCloudXYZIRADTPtr convert(size_t block_id) = 0;
  virtual drivers::PointCloudXYZIRADTPtr convert_dual(size_t block_id) = 0;

  virtual bool hasScanned() = 0;

  virtual drivers::PointCloudXYZIRADTPtr get_pointcloud() = 0;
};
}  // namespace drivers
}  // namespace nebula
#endif  // NEBULA_WS_HESAI_SCAN_DECODER_HPP
