#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_at_decoder.hpp"

#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_at.hpp"

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

  m_elevation_rad_map_.resize(MAX_AZI_LEN);
  m_sin_elevation_map_.resize(MAX_AZI_LEN);
  m_cos_elevation_map_.resize(MAX_AZI_LEN);
  m_azimuth_rad_map_.resize(MAX_AZI_LEN);
  m_sin_azimuth_map_.resize(MAX_AZI_LEN);
  m_cos_azimuth_map_.resize(MAX_AZI_LEN);
  for (size_t i = 0; i < MAX_AZI_LEN; ++i) {
    m_elevation_rad_map_[i] = 2.f * i * M_PI / MAX_AZI_LEN;
    m_sin_elevation_map_[i] = sinf(m_elevation_rad_map_[i]);
    m_cos_elevation_map_[i] = cosf(m_elevation_rad_map_[i]);
    m_azimuth_rad_map_[i] = 2.f * i * M_PI / MAX_AZI_LEN;
    m_sin_azimuth_map_[i] = sinf(m_azimuth_rad_map_[i]);
    m_cos_azimuth_map_[i] = cosf(m_azimuth_rad_map_[i]);
  }

  dual_return_distance_threshold_ = sensor_configuration_->dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;
  scan_timestamp_ = -1;
  last_azimuth_ = -1;
  max_azimuth_ = -1;
  last_field_ = -1;

  scan_pc_.reset(new NebulaPointCloud);
  overflow_pc_.reset(new NebulaPointCloud);
}

bool PandarATDecoder::hasScanned() { return has_scanned_; }

std::tuple<drivers::NebulaPointCloudPtr, double> PandarATDecoder::get_pointcloud()
{
  return std::make_tuple(scan_pc_, scan_timestamp_);
}

int PandarATDecoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    std::cout << "!parsePacket(pandar_packet)" << std::endl;
    return -1;
  }

  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new NebulaPointCloud); //MAX reallocates memory, slow?
    has_scanned_ = false;
  }

  for (int block_id = 0; block_id < packet_.header.chBlockNumber; ++block_id) {
    int azimuth = static_cast<int>(
      packet_.blocks[block_id].azimuth * LIDAR_AZIMUTH_UNIT +
      packet_.blocks[block_id].fine_azimuth);

    //*
    int count = 0, field = 0;
    while (count < correction_configuration_->frameNumber &&
           (((azimuth + MAX_AZI_LEN - correction_configuration_->startFrame[field]) % MAX_AZI_LEN +
             (correction_configuration_->endFrame[field] + MAX_AZI_LEN - azimuth) % MAX_AZI_LEN) !=
            (correction_configuration_->endFrame[field] + MAX_AZI_LEN -
             correction_configuration_->startFrame[field]) %
              MAX_AZI_LEN)) {
      field = (field + 1) % correction_configuration_->frameNumber;
      count++;
    }
    if (last_field_ != field) {
      if (max_azimuth_ < azimuth) {
        max_azimuth_ = azimuth;
      }
      scan_timestamp_ = packet_.unix_second + static_cast<double>(packet_.usec) / 1000000.f;
      auto block_pc = convert(block_id);
      *overflow_pc_ += *block_pc;
      has_scanned_ = true;
    } else {
      auto block_pc = convert(block_id);
      *scan_pc_ += *block_pc;
    }
    last_azimuth_ = azimuth;
    last_field_ = field;
  }
  return last_azimuth_;
}

#if defined(ROS_DISTRO_FOXY) || defined(ROS_DISTRO_GALACTIC)
void PandarATDecoder::CalcXTPointXYZIT(
  int block_id, int chLaserNumber, boost::shared_ptr<pcl::PointCloud<NebulaPoint>> cld)
{
#else
void PandarATDecoder::CalcXTPointXYZIT(
  int block_id, int chLaserNumber, std::shared_ptr<pcl::PointCloud<NebulaPoint>> cld)
{
#endif
  Block * block = &packet_.blocks[block_id];
  Block * another_block = &packet_.blocks[(block_id + 1) % 2];

  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    Unit & unit = block->units[i];
    Unit & another_unit = another_block->units[i];
    NebulaPoint point{};

    /* skip wrong points */
    if (unit.distance < MIN_RANGE || unit.distance > MAX_RANGE) {
      continue;
    }

    point.intensity = unit.intensity;
    auto another_intensity = another_unit.intensity;

    bool identical_flg = false;
    if (point.intensity == another_intensity && unit.distance == another_unit.distance) {
      if (0 < block_id) {
        continue;
      }
      identical_flg = true;
    }

    int Azimuth = static_cast<int>(block->azimuth * LIDAR_AZIMUTH_UNIT + block->fine_azimuth);
    int count = 0, field = 0;
    while (count < correction_configuration_->frameNumber &&
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
    point.azimuth = 2.f * azimuth * M_PI / MAX_AZI_LEN;
    point.distance = unit.distance;
    point.elevation = 2.f * elevation * M_PI / MAX_AZI_LEN;
    {
      float xyDistance = unit.distance * m_cos_elevation_map_[elevation];
      point.x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
      point.y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
      point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[elevation]);
    }
    if (scan_timestamp_ < 0) {  // invalid timestamp
      scan_timestamp_ = packet_.unix_second + static_cast<double>(packet_.usec) / 1000000.;
    }

    point.time_stamp = static_cast<uint32_t>(
      (packet_.unix_second +
      static_cast<double>(packet_.usec) / 1000000. -
      scan_timestamp_)*1000000000);

    switch (packet_.return_mode) {
      case STRONGEST_RETURN:
        point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::STRONGEST);
        break;

      case LAST_RETURN:
        point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::LAST);
        break;

      case DUAL_RETURN:
        if (identical_flg) {
          point.return_type = static_cast<uint8_t>(
            nebula::drivers::ReturnType::IDENTICAL);  // not present in the manual, but it always
                                                      // seems to be this pattern
        } else if (block_id == 0) {
          if (point.intensity < another_intensity) {
            point.return_type =
              static_cast<uint8_t>(nebula::drivers::ReturnType::LAST_WEAK);  // Last return
          } else {
            point.return_type = static_cast<uint8_t>(
              nebula::drivers::ReturnType::STRONGEST);  // Last and strongest return
          }
        } else {
          if (point.intensity > another_intensity) {
            point.return_type =
              static_cast<uint8_t>(nebula::drivers::ReturnType::STRONGEST);  // Strongest return
          } else {
            point.return_type = static_cast<uint8_t>(
              nebula::drivers::ReturnType::SECOND_STRONGEST);  // Second strongest return
          }
        }
        break;

      default:
        point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::UNKNOWN);
        break;
    }
    point.channel = i;
    cld->points.emplace_back(point);
  }
}

drivers::NebulaPointCloudPtr PandarATDecoder::convert(size_t block_id)
{
  NebulaPointCloudPtr block_pc(new NebulaPointCloud);
  CalcXTPointXYZIT(block_id, static_cast<int>(packet_.header.chLaserNumber), block_pc);

  return block_pc;
}

drivers::NebulaPointCloudPtr PandarATDecoder::convert_dual(size_t block_id)
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
  packet_.motor_speed = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

  index += MOTOR_SPEED_SIZE;

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
  //  index += UTC_SIZE;
  //  index += SEQUENCE_SIZE;

  return true;
}
}  // namespace pandar_at
}  // namespace drivers
}  // namespace nebula
