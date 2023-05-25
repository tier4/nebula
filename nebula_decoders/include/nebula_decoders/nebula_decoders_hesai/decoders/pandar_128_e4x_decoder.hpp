#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_128_e4x.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <array>

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
  std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() override;

private:
  bool parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet) override;
  drivers::NebulaPoint build_point(
    const Block & block, const size_t & laser_id, const uint16_t & azimuth,
    const uint32_t & unix_second, float & out_distance);
  drivers::NebulaPointCloudPtr convert(size_t) override;
  drivers::NebulaPointCloudPtr convert_dual(size_t) override;
  bool is_dual_return();
  static uint32_t get_epoch_from_datetime(const DateTime & date_time);

  std::array<float, LASER_COUNT> elevation_angle_{};
  std::array<float, LASER_COUNT> elevation_angle_rad_{};
  std::array<float, LASER_COUNT> cos_elevation_angle_{};
  std::array<float, LASER_COUNT> sin_elevation_angle_{};
  std::array<float, LASER_COUNT> azimuth_offset_{};
  std::array<float, LASER_COUNT> azimuth_offset_rad_{};

  std::array<float, MAX_AZIMUTH_STEPS> block_azimuth_rad_{};

  Packet packet_{};
  uint32_t current_unit_unix_second_{};
  uint8_t first_return_type_{};
  uint8_t second_return_type_{};

  //  PacketExtended packet_extended_{};
};

}  // namespace pandar_128_e4x
}  // namespace drivers
}  // namespace nebula
