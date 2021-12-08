#pragma once

#include "hesai/decoders/pandar_xt.hpp"
#include "hesai_scan_decoder.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <array>

namespace nebula
{
namespace drivers
{
namespace pandar_xt
{
class PandarXTDecoder : public HesaiScanDecoder
{
public:
  explicit PandarXTDecoder(const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
                           const std::shared_ptr<drivers::HesaiCloudConfiguration> & cloud_configuration,
                           const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration);
  void unpack(const pandar_msgs::msg::PandarPacket & raw_packet) override;
  bool hasScanned() override;
  drivers::PointCloudXYZIRADTPtr get_pointcloud() override;

private:

  bool parsePacket(const pandar_msgs::msg::PandarPacket& pandar_packet) override;
  drivers::PointXYZIRADT build_point(int block_id, int unit_id, ReturnMode return_type);
  drivers::PointCloudXYZIRADTPtr convert(size_t block_id) override;
  drivers::PointCloudXYZIRADTPtr convert_dual(size_t block_id) override;

  std::array<float, LASER_COUNT> elev_angle_{};
  std::array<float, LASER_COUNT> azimuth_offset_{};

  std::array<float, LASER_COUNT> firing_offset_{};

  std::array<float, BLOCKS_PER_PACKET> block_offset_single_{};
  std::array<float, BLOCKS_PER_PACKET> block_offset_dual_{};

  Packet packet_{};

};

}  // namespace pandar_xt
}
}