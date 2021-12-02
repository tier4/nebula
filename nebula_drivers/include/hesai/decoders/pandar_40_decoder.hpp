#pragma once

#include <hesai/scan_decoder.hpp>
#include <hesai/decoders/pandar_40.hpp>
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <array>

namespace nebula
{
namespace drivers
{
namespace pandar_40
{
class Pandar40Decoder : HesaiScanDecoder
{
public:
  Pandar40Decoder();
  void unpack(const pandar_msgs::msg::PandarScan & pandar_scan) override;
  bool hasScanned() override;
  drivers::PclPointCloudXYZIRADTPtr getPointcloud() override;

private:

  bool parsePacket(const pandar_msgs::msg::PandarPacket& pandar_packet);
  drivers::PclPointCloudXYZIRADTPtr build_point(int block_id, int unit_id, uint8_t return_type);
  drivers::PclPointCloudXYZIRADTPtr convert(const int block_id);
  drivers::PclPointCloudXYZIRADTPtr convert_dual(const int block_id);

  std::array<float, LASER_COUNT> elev_angle_;
  std::array<float, LASER_COUNT> azimuth_offset_;

  std::array<float, LASER_COUNT> firing_offset_;
  std::array<float, BLOCKS_PER_PACKET> block_offset_single_;
  std::array<float, BLOCKS_PER_PACKET> block_offset_dual_;

  std::array<size_t, LASER_COUNT> firing_order_;

  double dual_return_distance_threshold_;
  Packet packet_;

  drivers::PclPointCloudXYZIRADTPtr scan_pc_;
  drivers::PclPointCloudXYZIRADTPtr overflow_pc_;

  uint16_t scan_phase_;
  int last_phase_;
  bool has_scanned_;
};

}  // namespace pandar40
}
}