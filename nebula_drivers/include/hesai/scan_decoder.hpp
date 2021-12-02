#ifndef NEBULA_WS_SCAN_DECODER_HPP
#define NEBULA_WS_SCAN_DECODER_HPP

#include "hesai/point_types.hpp"
#include "hesai/hesai_common.hpp"

#include "pandar_msgs/msg/pandar_scan.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"

#include <fstream>
#include <vector>

namespace nebula
{
namespace drivers
{
class HesaiScanDecoder
{
private:
  std::shared_ptr<drivers::HesaiSensorConfiguration> sensor_configuration_;
  std::shared_ptr<drivers::HesaiCalibrationConfiguration> sensor_calibration_;
  std::shared_ptr<drivers::HesaiCloudConfiguration> cloud_calibration_;
public:
  HesaiScanDecoder(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder & operator=(HesaiScanDecoder && c) = delete;
  HesaiScanDecoder(const HesaiScanDecoder & c) = delete;
  HesaiScanDecoder & operator=(const HesaiScanDecoder & c) = delete;

  virtual ~HesaiScanDecoder() = default;
  HesaiScanDecoder() = default;
  virtual void unpack(const pandar_msgs::msg::PandarScan & pandar_scan) = 0;

  virtual bool parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet) = 0;

  virtual drivers::PclPointCloudXYZIRADTPtr convert(const int block_id) = 0;
  virtual drivers::PclPointCloudXYZIRADTPtr convert_dual(const int block_id) = 0;

  virtual bool hasScanned() = 0;

  virtual drivers::PclPointCloudXYZIRADTPtr getPointcloud() = 0;
};
}  // namespace drivers
}  // namespace nebula
#endif  // NEBULA_WS_SCAN_DECODER_HPP
