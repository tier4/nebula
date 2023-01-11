#pragma once

#include <array>
#include <hesai/decoders/hesai_scan_decoder.hpp>
#include <hesai/decoders/pandar_40.hpp>

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace drivers
{
namespace pandar_40
{
class Pandar40Decoder : public HesaiScanDecoder
{
public:
  explicit Pandar40Decoder(
    const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration);
  void unpack(const pandar_msgs::msg::PandarPacket & pandar_packet) override;
  bool hasScanned() override;
  drivers::PointCloudXYZIRADTPtr get_pointcloud() override;

private:
  bool parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet) override;
  drivers::PointXYZIRADT build_point(size_t block_id, size_t unit_id, ReturnMode return_type);
  drivers::PointCloudXYZIRADTPtr convert(size_t block_id) override;
  drivers::PointCloudXYZIRADTPtr convert_dual(size_t block_id) override;

  std::array<float, LASER_COUNT> elev_angle_{};
  std::array<float, LASER_COUNT> azimuth_offset_{};

  std::array<float, LASER_COUNT> firing_offset_{};
  std::array<size_t, LASER_COUNT> firing_order_{};

  std::array<float, BLOCKS_PER_PACKET> block_offset_single_{};
  std::array<float, BLOCKS_PER_PACKET> block_offset_dual_{};

  Packet packet_{};
};

}  // namespace pandar_40
}  // namespace drivers
}  // namespace nebula