#pragma once

#include "hesai/decoders/pandar_at.hpp"
#include "hesai_scan_decoder.hpp"

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <array>

namespace nebula
{
namespace drivers
{
namespace pandar_at
{
/*
const float pandarXTM_elev_angle_map[] = {
  19.5f, 18.2f, 16.9f, 15.6f, 14.3f, 13.0f, 11.7f, 10.4f, \
  9.1f,  7.8f,  6.5f,  5.2f,  3.9f,  2.6f,  1.3f, 0.0f,  \
  -1.3f, -2.6f, -3.9f, -5.2f, -6.5f, -7.8f, -9.1f, -10.4f, \
  -11.7f, -13.0f, -14.3f, -15.6f, -16.9f, -18.2f, -19.5f, -20.8f
};

const float pandarXTM_horizontal_azimuth_offset_map[] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};
*/
/*
const float pandarXTM_elev_angle_map[] = {
  19.407f, 18.106f, 16.801f, 15.512f, 14.214f, 12.917f, 11.661f, 10.362f, \
  9.066f,  7.769f,  6.475f,  5.181f,  3.885f,  2.595f,  1.299f, -0.005f,  \
  -1.310f, -2.605f, -3.905f, -5.204f, -6.496f, -7.792f, -9.087f, -10.386f, \
  -11.685f, -12.955f, -14.242f, -15.537f, -16.834f, -18.129f, -19.424f, -20.720f
};

const float pandarXTM_horizontal_azimuth_offset_map[] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};
*/

const float blockXTMOffsetTriple[] = {
  5.632f - 50.0f * 1.0f, 5.632f - 50.0f * 1.0f, 5.632f - 50.0f * 1.0f, 5.632f - 50.0f * 0.0f,
  5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f};

const float blockXTMOffsetDual[] = {
  5.632f - 50.0f * 2.0f, 5.632f - 50.0f * 2.0f, 5.632f - 50.0f * 1.0f, 5.632f - 50.0f * 1.0f,
  5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f};
const float blockXTMOffsetSingle[] = {
  5.632f - 50.0f * 5.0f, 5.632f - 50.0f * 4.0f, 5.632f - 50.0f * 3.0f, 5.632f - 50.0f * 2.0f,
  5.632f - 50.0f * 1.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f, 5.632f - 50.0f * 0.0f};

const float laserXTMOffset[] = {
  2.856f * 0.0f + 0.368f,  2.856f * 1.0f + 0.368f,  2.856f * 2.0f + 0.368f,
  2.856f * 3.0f + 0.368f,  2.856f * 4.0f + 0.368f,  2.856f * 5.0f + 0.368f,
  2.856f * 6.0f + 0.368f,  2.856f * 7.0f + 0.368f,

  2.856f * 8.0f + 0.368f,  2.856f * 9.0f + 0.368f,  2.856f * 10.0f + 0.368f,
  2.856f * 11.0f + 0.368f, 2.856f * 12.0f + 0.368f, 2.856f * 13.0f + 0.368f,
  2.856f * 14.0f + 0.368f, 2.856f * 15.0f + 0.368f,

  2.856f * 0.0f + 0.368f,  2.856f * 1.0f + 0.368f,  2.856f * 2.0f + 0.368f,
  2.856f * 3.0f + 0.368f,  2.856f * 4.0f + 0.368f,  2.856f * 5.0f + 0.368f,
  2.856f * 6.0f + 0.368f,  2.856f * 7.0f + 0.368f,

  2.856f * 8.0f + 0.368f,  2.856f * 9.0f + 0.368f,  2.856f * 10.0f + 0.368f,
  2.856f * 11.0f + 0.368f, 2.856f * 12.0f + 0.368f, 2.856f * 13.0f + 0.368f,
  2.856f * 14.0f + 0.368f, 2.856f * 15.0f + 0.368f};

const float elev_angle[] = {
  14.436f,  13.535f,  13.08f,   12.624f,  12.163f,  11.702f,  11.237f,  10.771f,  10.301f,
  9.83f,    9.355f,   8.88f,    8.401f,   7.921f,   7.437f,   6.954f,   6.467f,   5.98f,
  5.487f,   4.997f,   4.501f,   4.009f,   3.509f,   3.014f,   2.512f,   2.014f,   1.885f,
  1.761f,   1.637f,   1.511f,   1.386f,   1.258f,   1.13f,    1.009f,   0.88f,    0.756f,
  0.63f,    0.505f,   0.379f,   0.251f,   0.124f,   0.0f,     -0.129f,  -0.254f,  -0.38f,
  -0.506f,  -0.632f,  -0.76f,   -0.887f,  -1.012f,  -1.141f,  -1.266f,  -1.393f,  -1.519f,
  -1.646f,  -1.773f,  -1.901f,  -2.027f,  -2.155f,  -2.282f,  -2.409f,  -2.535f,  -2.662f,
  -2.789f,  -2.916f,  -3.044f,  -3.172f,  -3.299f,  -3.425f,  -3.552f,  -3.680f,  -3.806f,
  -3.933f,  -4.062f,  -4.190f,  -4.318f,  -4.444f,  -4.571f,  -4.698f,  -4.824f,  -4.951f,
  -5.081f,  -5.209f,  -5.336f,  -5.463f,  -5.589f,  -5.717f,  -5.843f,  -5.968f,  -6.099f,
  -6.607f,  -7.118f,  -7.624f,  -8.135f,  -8.64f,   -9.149f,  -9.652f,  -10.16f,  -10.664f,
  -11.17f,  -11.67f,  -12.174f, -12.672f, -13.173f, -13.668f, -14.166f, -14.658f, -15.154f,
  -15.643f, -16.135f, -16.62f,  -17.108f, -17.59f,  -18.073f, -18.548f, -19.031f, -19.501f,
  -19.981f, -20.445f, -20.92f,  -21.379f, -21.85f,  -22.304f, -22.77f,  -23.219f, -23.68f,
  -24.123f, -25.016f,
};

const float azimuth_offset[] = {
  3.257f,  3.263f,  -1.083f, 3.268f,  -1.086f, 3.273f,  -1.089f, 3.278f,  -1.092f, 3.283f,  -1.094f,
  3.288f,  -1.097f, 3.291f,  -1.1f,   1.1f,    -1.102f, 1.1f,    -3.306f, 1.102f,  -3.311f, 1.103f,
  -3.318f, 1.105f,  -3.324f, 1.106f,  7.72f,   5.535f,  3.325f,  -3.33f,  -1.114f, -5.538f, -7.726f,
  1.108f,  7.731f,  5.543f,  3.329f,  -3.336f, -1.116f, -5.547f, -7.738f, 1.108f,  7.743f,  5.551f,
  3.335f,  -3.342f, -1.119f, -5.555f, -7.75f,  1.11f,   7.757f,  5.56f,   3.34f,   -3.347f, -1.121f,
  -5.564f, -7.762f, 1.111f,  7.768f,  5.569f,  3.345f,  -3.353f, -1.123f, -5.573f, -7.775f, 1.113f,
  7.780f,  5.578f,  3.351f,  -3.358f, -1.125f, -5.582f, -7.787f, 1.115f,  7.792f,  5.586f,  3.356f,
  -3.363f, -1.126f, -5.591f, -7.799f, 1.117f,  7.804f,  5.595f,  3.36f,   -3.369f, -1.128f, -5.599f,
  -7.811f, 1.119f,  -3.374f, 1.12f,   -3.379f, 1.122f,  -3.383f, 3.381f,  -3.388f, 3.386f,  -1.135f,
  3.39f,   -1.137f, 3.395f,  -1.138f, 3.401f,  -1.139f, 3.406f,  -1.14f,  3.41f,   -1.141f, 3.416f,
  -1.142f, 1.14f,   -1.143f, 1.143f,  -3.426f, 1.146f,  -3.429f, 1.147f,  -3.433f, 1.15f,   -3.436f,
  1.152f,  -3.44f,  1.154f,  -3.443f, 1.157f,  -3.446f, -3.449f,
};

const uint16_t MAX_AZIMUTH_DEGREE_NUM = 36000;

const uint16_t LIDAR_AZIMUTH_UNIT = 256;
const uint32_t MAX_AZI_LEN = 36000 * 256;

class PandarATDecoder : public HesaiScanDecoder
{
public:
  explicit PandarATDecoder(
    const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration,
    const std::shared_ptr<drivers::HesaiCorrection> & correction_configuration);
  void unpack(const pandar_msgs::msg::PandarPacket & raw_packet) override;
  bool hasScanned() override;
  drivers::PointCloudXYZIRADTPtr get_pointcloud() override;

private:
  bool parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet) override;
  //  drivers::PointXYZIRADT build_point(int block_id, int unit_id, ReturnMode return_type);

#if defined(ROS_DISTRO_FOXY) || defined(ROS_DISTRO_GALACTIC)
  void CalcXTPointXYZIT(
    int blockid, int chLaserNumber, boost::shared_ptr<pcl::PointCloud<PointXYZIRADT>> cld);
#else
  void CalcXTPointXYZIT(
    int blockid, int chLaserNumber, std::shared_ptr<pcl::PointCloud<PointXYZIRADT>> cld);
#endif

  drivers::PointCloudXYZIRADTPtr convert(size_t block_id) override;
  drivers::PointCloudXYZIRADTPtr convert_dual(size_t block_id) override;

  std::array<float, LASER_COUNT> elev_angle_{};
  std::array<float, LASER_COUNT> azimuth_offset_{};

  //  std::array<float, LASER_COUNT> firing_offset_{};

  std::array<float, BLOCKS_PER_PACKET> block_offset_single_{};
  std::array<float, BLOCKS_PER_PACKET> block_offset_dual_{};
  std::array<float, BLOCKS_PER_PACKET> block_offset_triple_{};

  std::vector<float> m_sin_elevation_map_;
  std::vector<float> m_cos_elevation_map_;

  std::vector<float> m_sin_azimuth_map_;
  std::vector<float> m_cos_azimuth_map_;

  Packet packet_{};

  uint16_t last_azimuth_;
  int start_angle_;
  double last_timestamp_;

  std::shared_ptr<drivers::HesaiCorrection> correction_configuration_;

  bool use_dat = true;
};

}  // namespace pandar_at
}  // namespace drivers
}  // namespace nebula
