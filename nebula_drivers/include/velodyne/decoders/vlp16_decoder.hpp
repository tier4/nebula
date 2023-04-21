#pragma once

#include <velodyne/decoders/velodyne_scan_decoder.hpp>

#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

#include <array>

namespace nebula
{
namespace drivers
{
namespace vlp16
{
class Vlp16Decoder : public VelodyneScanDecoder
{
public:
  explicit Vlp16Decoder(
    const std::shared_ptr<drivers::VelodyneSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::VelodyneCalibrationConfiguration> & calibration_configuration);
  void unpack(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) override;
  bool hasScanned() override;
  int pointsPerPacket() override;
  drivers::PointCloudXYZIRADTPtr get_pointcloud() override;
  void reset_pointcloud(size_t n_pts) override;
  void reset_overflow() override;

private:
  bool parsePacket(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) override;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];
  int phase_;
  int max_pts_;
};

}  // namespace vlp16
}  // namespace drivers
}  // namespace nebula
