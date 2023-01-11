#include "hesai/decoders/pandar_at_decoder.hpp"

#include "hesai/decoders/pandar_at.hpp"
//#include <rclcpp/rclcpp.hpp>

namespace nebula
{
namespace drivers
{
namespace pandar_at
{
PandarATDecoder::PandarATDecoder(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration,
  const std::shared_ptr<drivers::HesaiCorrection> & correction_configuration)
{
  sensor_configuration_ = sensor_configuration;
  sensor_calibration_ = calibration_configuration;
  correction_configuration_ = correction_configuration;

  // TODO: add calibration data validation
  // if(calibration.elev_angle_map.size() != num_lasers_){
  //   // calibration data is not valid!
  // }
  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
  }
  /////////////////

  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
  }

  m_sin_elevation_map_.resize(MAX_AZI_LEN);
  m_cos_elevation_map_.resize(MAX_AZI_LEN);
  for (size_t i = 0; i < MAX_AZI_LEN; ++i) {
    m_sin_elevation_map_[i] = sinf(2 * i * M_PI / MAX_AZI_LEN);
    m_cos_elevation_map_[i] = cosf(2 * i * M_PI / MAX_AZI_LEN);
  }
  m_sin_azimuth_map_.resize(MAX_AZI_LEN);
  m_cos_azimuth_map_.resize(MAX_AZI_LEN);
  for (size_t i = 0; i < MAX_AZI_LEN; ++i) {
    m_sin_azimuth_map_[i] = sinf(2 * i * M_PI / MAX_AZI_LEN);
    m_cos_azimuth_map_[i] = cosf(2 * i * M_PI / MAX_AZI_LEN);
  }

  scan_phase_ = static_cast<uint16_t>(0 * 100.0f);
  dual_return_distance_threshold_ = sensor_configuration_->dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;

  scan_pc_.reset(new PointCloudXYZIRADT);
  overflow_pc_.reset(new PointCloudXYZIRADT);
}

bool PandarATDecoder::hasScanned() { return has_scanned_; }

drivers::PointCloudXYZIRADTPtr PandarATDecoder::get_pointcloud() { return scan_pc_; }

void PandarATDecoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    std::cout << "!parsePacket(pandar_packet)" << std::endl;
    return;
  }

  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new PointCloudXYZIRADT);
    has_scanned_ = false;
  }

  for (int block_id = 0; block_id < packet_.header.chBlockNumber; ++block_id) {
    int azimuthGap = 0;      /* To do */
    double timestampGap = 0; /* To do */
    int azimuth = packet_.blocks[block_id].azimuth;
    azimuth %= 12000;

    if (last_azimuth_ > azimuth) {
      azimuthGap = azimuth + (12000 - static_cast<int>(last_azimuth_));
    } else {
      azimuthGap = azimuth - static_cast<int>(last_azimuth_);
    }
    timestampGap = packet_.usec - last_timestamp_ + 0.001;

    auto block_pc = convert(block_id);
    if (
      (packet_.return_mode == STRONGEST_RETURN || packet_.return_mode == LAST_RETURN) ||
      (last_azimuth_ != azimuth &&  //            (azimuthGap / timestampGap) < 36000 * 100 ) {
       (azimuthGap / timestampGap) < 12000 * 100)) {
      /* for all the blocks */
      if (
        (last_azimuth_ > azimuth && start_angle_ <= azimuth) ||
        (last_azimuth_ < start_angle_ && start_angle_ <= azimuth)) {
        *overflow_pc_ += *block_pc;
        has_scanned_ = true;
      } else {
        *scan_pc_ += *block_pc;
      }
    } else {
      *scan_pc_ += *block_pc;
    }
    last_azimuth_ = azimuth;
    last_timestamp_ = packet_.usec;
  }
}

#if defined(ROS_DISTRO_FOXY) || defined(ROS_DISTRO_GALACTIC)
void PandarATDecoder::CalcXTPointXYZIT(
  int blockid, int chLaserNumber, boost::shared_ptr<pcl::PointCloud<PointXYZIRADT>> cld)
{
#else
void PandarATDecoder::CalcXTPointXYZIT(
  int blockid, int chLaserNumber, std::shared_ptr<pcl::PointCloud<PointXYZIRADT>> cld)
{
#endif
  Block * block = &packet_.blocks[blockid];

  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    Unit & unit = block->units[i];
    PointXYZIRADT point{};

    /* skip wrong points */
    if (unit.distance <= 0.1 || unit.distance > 180.0) {
      continue;
    }
    if (use_dat) {
      int Azimuth = static_cast<int>(block->azimuth * LIDAR_AZIMUTH_UNIT + block->fine_azimuth);
      int count = 0, field = 0;
      while (
        count < correction_configuration_->frameNumber &&
        (((Azimuth + MAX_AZI_LEN - correction_configuration_->startFrame[field]) % MAX_AZI_LEN +
          (correction_configuration_->endFrame[field] + MAX_AZI_LEN - Azimuth) % MAX_AZI_LEN) !=
         (correction_configuration_->endFrame[field] + MAX_AZI_LEN -
          correction_configuration_->startFrame[field]) %
           MAX_AZI_LEN)) {
        field = (field + 1) % correction_configuration_->frameNumber;
        count++;
      }
      auto elevation =
        correction_configuration_->elevation[i] +
        correction_configuration_->getElevationAdjustV3(i, Azimuth) * LIDAR_AZIMUTH_UNIT;
      elevation = (MAX_AZI_LEN + elevation) % MAX_AZI_LEN;
      auto azimuth = (Azimuth + MAX_AZI_LEN - correction_configuration_->startFrame[field]) * 2 -
                     correction_configuration_->azimuth[i] +
                     correction_configuration_->getAzimuthAdjustV3(i, Azimuth) * LIDAR_AZIMUTH_UNIT;
      azimuth = (MAX_AZI_LEN + azimuth) % MAX_AZI_LEN;

      {
        float xyDistance = unit.distance * m_cos_elevation_map_[elevation];
        point.x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
        point.y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
        point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[elevation]);
      }
    } else {
      int Azimuth = static_cast<int>(block->azimuth * LIDAR_AZIMUTH_UNIT + block->fine_azimuth);

      auto elevation = static_cast<int>(elev_angle_[i] * 100 * LIDAR_AZIMUTH_UNIT);
      elevation = (MAX_AZI_LEN + elevation) % MAX_AZI_LEN;
      auto azimuth = static_cast<int>(
        Azimuth + MAX_AZI_LEN - (azimuth_offset_[i] * 100 * LIDAR_AZIMUTH_UNIT) / 2);
      azimuth = (MAX_AZI_LEN + azimuth) % MAX_AZI_LEN;

      {
        float xyDistance = unit.distance * m_cos_elevation_map_[elevation];
        point.x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
        point.y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
        point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[elevation]);
      }
    }

    point.intensity = unit.intensity;
    double unix_second = packet_.unix_second;
    point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;
    point.return_type = packet_.return_mode;
    point.ring = i;
    cld->points.emplace_back(point);
  }
}

drivers::PointCloudXYZIRADTPtr PandarATDecoder::convert(size_t block_id)
{
  PointCloudXYZIRADTPtr block_pc(new PointCloudXYZIRADT);
  CalcXTPointXYZIT(block_id, static_cast<int>(packet_.header.chLaserNumber), block_pc);

  return block_pc;
}

drivers::PointCloudXYZIRADTPtr PandarATDecoder::convert_dual(size_t block_id)
{
  return convert(block_id);
}

bool PandarATDecoder::parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (pandar_packet.size != PACKET_SIZE) {
    std::cout << "pandar_packet.size != PACKET_SIZE" << std::endl;
    return false;
  }
  const uint8_t * buf = &pandar_packet.data[0];

  size_t index = 0;
  // Parse 12 Bytes Header
  packet_.header.sob = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
  packet_.header.chProtocolMajor = buf[index + 2] & 0xff;
  packet_.header.chProtocolMinor = buf[index + 3] & 0xff;
  packet_.header.chLaserNumber = static_cast<int>(buf[index + 6] & 0xff);
  packet_.header.chBlockNumber = buf[index + 7] & 0xff;
  packet_.header.chReturnType = buf[index + 8] & 0xff;
  packet_.header.chDisUnit = buf[index + 9] & 0xff;
  index += HEAD_SIZE;

  if (packet_.header.sob != 0xEEFF) {
    // Error Start of Packet!
    std::cout << "Error Start of Packet!" << std::endl;
    return false;
  }

  for (int8_t block = 0; block < packet_.header.chBlockNumber; block++) {
    packet_.blocks[block].azimuth = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    packet_.blocks[block].fine_azimuth = buf[index + 2] & 0xff;
    index += BLOCK_HEADER_AZIMUTH;

    for (int unit = 0; unit < packet_.header.chLaserNumber; unit++) {
      unsigned int unRange = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      packet_.blocks[block].units[unit].distance =
        (static_cast<float>(unRange * packet_.header.chDisUnit)) / 1000.f;
      packet_.blocks[block].units[unit].intensity = (buf[index + 2] & 0xff);
      packet_.blocks[block].units[unit].confidence = (buf[index + 3] & 0xff);
      index += UNIT_SIZE;
    }
  }
  index += CRC_SIZE;

  index += RESERVED1_SIZE;  // skip reserved bytes
  packet_.shutdown_flg = buf[index] & 0xff;
  index += HIGH_TEMP_SHUTDOWN_FLAG_SIZE;
  index += RESERVED2_SIZE;  // skip reserved bytes
  packet_.moter_speed = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

  index += MOTER_SPEED_SIZE;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);

  index += TIMESTAMP_SIZE;
  packet_.return_mode = buf[index] & 0xff;
  index += RETURN_SIZE;
  index += FACTORY_SIZE;

  if ((buf[index] & 0xff) != 0) {
    packet_.t.tm_year = (buf[index + 0] & 0xff);
    // in case of time error
    if (packet_.t.tm_year >= 200) {
      packet_.t.tm_year -= 100;
    }
    packet_.t.tm_mon = (buf[index + 1] & 0xff) - 1;
    packet_.t.tm_mday = buf[index + 2] & 0xff;
    packet_.t.tm_hour = buf[index + 3] & 0xff;
    packet_.t.tm_min = buf[index + 4] & 0xff;
    packet_.t.tm_sec = buf[index + 5] & 0xff;
    packet_.t.tm_isdst = 0;
    packet_.unix_second = static_cast<double>(mktime(&packet_.t));  // + m_iTimeZoneSecond);
  } else {
    uint32_t utc_time_big = (buf[index + 2] & 0xff) | (buf[index + 3] & 0xff) << 8 |
                            ((buf[index + 4] & 0xff) << 16) | ((buf[index + 5] & 0xff) << 24);
    packet_.unix_second = ((utc_time_big >> 24) & 0xff) | ((utc_time_big >> 8) & 0xff00) |
                          ((utc_time_big << 8) & 0xff0000) | ((utc_time_big << 24));
  }
  index += UTC_SIZE;
  index += SEQUENCE_SIZE;

  return true;
}
}  // namespace pandar_at
}  // namespace drivers
}  // namespace nebula