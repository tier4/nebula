#pragma once

#include <array>
#include <hesai/decoders/hesai_scan_decoder.hpp>
#include <hesai/decoders/pandar_128_e4x.hpp>

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

namespace nebula
{
namespace drivers
{
namespace pandar_128_e4x
{
class Pandar128E4XDecoder : public HesaiScanDecoder
{
public:
  explicit Pandar128E4XDecoder(
    const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration);
  void unpack(const pandar_msgs::msg::PandarPacket & pandar_packet) override;
  bool hasScanned() override;
  drivers::PointCloudXYZIRADTPtr get_pointcloud() override;

private:
  bool parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet) override;
  drivers::PointXYZIRADT build_point(
    const Block & block, const size_t & laser_id, const uint16_t & azimuth,
    const double & unix_second);
  inline drivers::PointCloudXYZIRADTPtr convert(size_t) override{};
  inline drivers::PointCloudXYZIRADTPtr convert_dual(size_t) override{};
  drivers::PointCloudXYZIRADTPtr convert();
  drivers::PointCloudXYZIRADTPtr convert_dual();

  std::array<float, LASER_COUNT> elev_angle_{};
  std::array<float, LASER_COUNT> elev_angle_rad_{};
  std::array<float, LASER_COUNT> cos_elev_angle_{};
  std::array<float, LASER_COUNT> sin_elev_angle_{};
  std::array<float, LASER_COUNT> azimuth_offset_{};

  Packet packet_{};
  //  PacketExtended packet_extended_{};
};

}  // namespace pandar_128_e4x
}  // namespace drivers
}  // namespace nebula