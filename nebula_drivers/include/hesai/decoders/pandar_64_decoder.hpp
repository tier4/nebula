#pragma once

#include "hesai/decoders/pandar_64.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include "hesai/scan_decoder.hpp"

#include <array>

namespace nebula
{
namespace drivers
{
namespace pandar64
{
class Pandar64Decoder : HesaiScanDecoder
{
public:
  explicit Pandar64Decoder();
  void unpack(const pandar_msgs::msg::PandarScan & raw_packet) override;
  drivers::PclPointCloudXYZIRADTPtr build_point(int block_id, int unit_id, uint8_t return_type);
  bool hasScanned() override;
  drivers::PclPointCloudXYZIRADTPtr getPointcloud() override;

private:
  bool parsePacket(const pandar_msgs::msg::PandarPacket& pandar_scan);
  drivers::PclPointCloudXYZIRADTPtr convert(int block_id);
  drivers::PclPointCloudXYZIRADTPtr convert_dual(int block_id);

  std::array<float, UNIT_NUM> elev_angle_{};
  std::array<float, UNIT_NUM> azimuth_offset_{};

  std::array<float, UNIT_NUM> firing_offset_{};
  std::array<float, BLOCK_NUM> block_offset_single_{};
  std::array<float, BLOCK_NUM> block_offset_dual_{};

  double dual_return_distance_threshold_;
  Packet packet_{};

  drivers::PclPointCloudXYZIRADTPtr scan_pc_;
  drivers::PclPointCloudXYZIRADTPtr overflow_pc_;

  uint16_t scan_phase_;
  int last_phase_;
  bool has_scanned_;
};

}  // namespace pandar_qt
}
}