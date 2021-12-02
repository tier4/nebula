#pragma once

#include "hesai/decoders/pandar_qt.hpp"
#include "hesai/point_types.hpp"

#include <hesai/scan_decoder.hpp>

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <array>

namespace nebula
{
namespace drivers
{
namespace pandar_qt
{
class PandarQTDecoder : HesaiScanDecoder
{
public:
  PandarQTDecoder();
  void unpack(const pandar_msgs::msg::PandarScan & pandar_scan) override;
  drivers::PclPointCloudXYZIRADTPtr build_point(int block_id, int unit_id, uint8_t return_type);
  bool hasScanned() override;
  drivers::PclPointCloudXYZIRADTPtr getPointcloud() override;

private:
  bool parsePacket(const pandar_msgs::msg::PandarPacket & pandar_scan) override;
  drivers::PclPointCloudXYZIRADTPtr convert(const int block_id) override;
  drivers::PclPointCloudXYZIRADTPtr convert_dual(const int block_id) override;

  std::array<float, UNIT_NUM> elev_angle_;
  std::array<float, UNIT_NUM> azimuth_offset_;

  std::array<float, UNIT_NUM> firing_offset_;
  std::array<float, BLOCK_NUM> block_offset_single_;
  std::array<float, BLOCK_NUM> block_offset_dual_;

  double dual_return_distance_threshold_;
  Packet packet_;

  drivers::PclPointCloudXYZIRADTPtr scan_pc_;
  drivers::PclPointCloudXYZIRADTPtr overflow_pc_;

  uint16_t scan_phase_;
  int last_phase_;
  bool has_scanned_;
};

}  // namespace pandar_qt
}
}
