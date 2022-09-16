#include "hesai/decoders/pandar_at_decoder.hpp"

#include "hesai/decoders/pandar_at.hpp"

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

  /*
  for (size_t unit = 0; unit < LASER_COUNT; ++unit) {
    firing_offset_[unit] = 1.512f * static_cast<float>(unit) + 0.28f;
  }

  for (size_t block = 0; block < BLOCKS_PER_PACKET; ++block) {
    block_offset_single_[block] =
      3.28f - 50.00f * static_cast<float>(BLOCKS_PER_PACKET - block - 1);
    block_offset_dual_[block] =
      3.28f - 50.00f * (static_cast<float>(BLOCKS_PER_PACKET - block - 1) / 2.f);
  }

  // TODO: add calibration data validation
  // if(calibration.elev_angle_map.size() != num_lasers_){
  //   // calibration data is not valid!
  // }
  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
  }
  */
  /////////////////

  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
//    elev_angle_[laser] = elev_angle[laser];
//    azimuth_offset_[laser] = azimuth_offset[laser];
  }


  /*
  m_sin_elevation_map_.resize(LASER_COUNT);
  m_cos_elevation_map_.resize(LASER_COUNT);
  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    m_sin_elevation_map_[laser] = sinf(deg2rad(elev_angle_[laser]));
    m_cos_elevation_map_[laser] = cosf(deg2rad(elev_angle_[laser]));
  }
  */
  m_sin_elevation_map_.resize(MAX_AZI_LEN);
  m_cos_elevation_map_.resize(MAX_AZI_LEN);
  for (size_t i = 0; i < MAX_AZI_LEN; ++i) {
    m_sin_elevation_map_[i] = sinf(2 * i * M_PI / MAX_AZI_LEN);
    m_cos_elevation_map_[i] = cosf(2 * i * M_PI / MAX_AZI_LEN);
  }
  /*
  m_sin_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);
  m_cos_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);
  for(int i = 0; i < MAX_AZIMUTH_DEGREE_NUM; ++i) {
    m_sin_azimuth_map_[i] = sinf(i * M_PI / 18000);
    m_cos_azimuth_map_[i] = cosf(i * M_PI / 18000);
  }
  */
  m_sin_azimuth_map_.resize(MAX_AZI_LEN);
  m_cos_azimuth_map_.resize(MAX_AZI_LEN);
  for (size_t i = 0; i < MAX_AZI_LEN; ++i) {
    m_sin_azimuth_map_[i] = sinf(2 * i * M_PI / MAX_AZI_LEN);
    m_cos_azimuth_map_[i] = cosf(2 * i * M_PI / MAX_AZI_LEN);
  }

//  scan_phase_ = static_cast<uint16_t>(sensor_configuration_->scan_phase * 100.0f);
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
//  std::cout << "parsePacket(pandar_packet)" << std::endl;

  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new PointCloudXYZIRADT);
    has_scanned_ = false;
  }

  /*
  bool dual_return =
    (packet_.return_mode != FIRST_RETURN && packet_.return_mode != STRONGEST_RETURN &&
     packet_.return_mode != LAST_RETURN);
  auto step = dual_return ? 2 : 1;

  for (size_t block_id = 0; block_id < BLOCKS_PER_PACKET; block_id += step) {
    auto block_pc = dual_return ? convert_dual(block_id) : convert(block_id);
    int current_phase =
      (static_cast<int>(packet_.blocks[block_id].azimuth) - scan_phase_ + 36000) % 36000;
    if (current_phase > last_phase_ && !has_scanned_) {
      *scan_pc_ += *block_pc;
    } else {
      *overflow_pc_ += *block_pc;
      has_scanned_ = true;
    }
    last_phase_ = current_phase;
  }
  */
// std::cout << "packet_.header.chBlockNumber = " << packet_.header.chBlockNumber << std::endl;
  for (int block_id = 0; block_id < packet_.header.chBlockNumber; ++block_id) {
    int azimuthGap = 0; /* To do */
    double timestampGap = 0; /* To do */
    if(last_azimuth_ > packet_.blocks[block_id].azimuth) {
      azimuthGap = static_cast<int>(packet_.blocks[block_id].azimuth) + (36000 - static_cast<int>(last_azimuth_));
    } else {
      azimuthGap = static_cast<int>(packet_.blocks[block_id].azimuth) - static_cast<int>(last_azimuth_);
    }
    timestampGap = packet_.usec - last_timestamp_ + 0.001;


    int Azimuth = static_cast<int>(packet_.blocks[block_id].azimuth * LIDAR_AZIMUTH_UNIT + packet_.blocks[block_id].fine_azimuth);
    /*
    std::cout << "azimuth: " << azimuth << std::endl;
    std::cout << "correction_configuration_->frameNumber: " << correction_configuration_->frameNumber << std::endl;
    std::cout << "correction_configuration_->frameNumber: " << static_cast<int>(correction_configuration_->frameNumber) << std::endl;
    */
    int count = 0, field = 0;
    while ( count < correction_configuration_->frameNumber
        && (
        ((Azimuth + MAX_AZI_LEN  - correction_configuration_->startFrame[field]) % MAX_AZI_LEN  + (correction_configuration_->endFrame[field] + MAX_AZI_LEN - Azimuth) % MAX_AZI_LEN )
          != (correction_configuration_->endFrame[field] + MAX_AZI_LEN  - correction_configuration_->startFrame[field]) % MAX_AZI_LEN  )
    ) {
      field = (field + 1) % correction_configuration_->frameNumber;
      count++;
//      std::cout << "field: " << field << std::endl;
//      std::cout << "count: " << count << std::endl;
    }
//    std::cout << "field: " << field << std::endl;
//    std::cout << "count: " << count << std::endl;
    if (count >= correction_configuration_->frameNumber)
      continue;


    auto block_pc = convert(block_id);
    if (last_azimuth_ != packet_.blocks[block_id].azimuth && \
            (azimuthGap / timestampGap) < 36000 * 100 ) {
      /* for all the blocks */
      if ((last_azimuth_ > packet_.blocks[block_id].azimuth &&
           start_angle_ <= packet_.blocks[block_id].azimuth) ||
          (last_azimuth_ < start_angle_ &&
           start_angle_ <= packet_.blocks[block_id].azimuth)) {
          *overflow_pc_ += *block_pc;
          has_scanned_ = true;
      }
    } else {
      *scan_pc_ += *block_pc;
      //printf("last_azimuth_:%d pkt.blocks[block_id].azimuth:%d  *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[block_id].azimuth, azimuthGap);
    }
//    CalcXTPointXYZIT(block_id, packet_.header.chLaserNumber, scan_pc_);
    last_azimuth_ = packet_.blocks[block_id].azimuth;
    last_timestamp_ = packet_.usec;
  }
}


#if defined(ROS_DISTRO_FOXY) || defined(ROS_DISTRO_GALACTIC)
void PandarATDecoder::CalcXTPointXYZIT(int blockid, \
    int chLaserNumber, boost::shared_ptr<pcl::PointCloud<PointXYZIRADT>> cld) {
#else
void PandarATDecoder::CalcXTPointXYZIT(int blockid, \
    int chLaserNumber, std::shared_ptr<pcl::PointCloud<PointXYZIRADT>> cld) {
#endif
  Block *block = &packet_.blocks[blockid];

//  std::cout << "chLaserNumber =" << chLaserNumber << std::endl;
//  std::cout << "chLaserNumber =" << static_cast<int>(chLaserNumber) << std::endl;
  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    Unit &unit = block->units[i];
    PointXYZIRADT point{};

    /* skip wrong points */
//    std::cout << "unit.distance =" << unit.distance << std::endl;
//    if (unit.distance <= 0.1 || unit.distance > 200.0) {
//    if (unit.distance <= 0.1 || unit.distance > 300.0) {
    if (unit.distance <= 0.1 || unit.distance > 180.0) {
//      std::cout << "unit.distance <= 0.1 || unit.distance > 300.0" << std::endl;
      continue;
    }
/*
//    int azimuth = static_cast<int>(azimuth_offset_[i] * 100 + block->azimuth);
//    int azimuth = static_cast<int>(azimuth_offset_[i] * 100 + block->azimuth + block->fine_azimuth * 100 / 256.0);
//    int azimuth = static_cast<int>(block->azimuth + block->fine_azimuth / 256.0 - azimuth_offset_[i] * 100 / 2.0);
    int azimuth = static_cast<int>(block->azimuth + block->fine_azimuth / 256.0 + azimuth_offset_[i] * 100);
//    int azimuth = static_cast<int>(block->azimuth  + azimuth_offset_[i] * 100);
    if(azimuth < 0)
      azimuth += 36000;
    if(azimuth >= 36000)
      azimuth -= 36000;
*/
    if(use_dat){
      int Azimuth = static_cast<int>(block->azimuth * LIDAR_AZIMUTH_UNIT + block->fine_azimuth);
      int count = 0, field = 0;
      while ( count < correction_configuration_->frameNumber
          && (
          ((Azimuth + MAX_AZI_LEN  - correction_configuration_->startFrame[field]) % MAX_AZI_LEN  + (correction_configuration_->endFrame[field] + MAX_AZI_LEN - Azimuth) % MAX_AZI_LEN )
            != (correction_configuration_->endFrame[field] + MAX_AZI_LEN  - correction_configuration_->startFrame[field]) % MAX_AZI_LEN  )
      ) {
        field = (field + 1) % correction_configuration_->frameNumber;
        count++;
      }
      auto elevation = correction_configuration_->elevation[i] + correction_configuration_->getElevationAdjustV3(i, Azimuth) * LIDAR_AZIMUTH_UNIT;
      elevation = (MAX_AZI_LEN + elevation) % MAX_AZI_LEN;      
      auto azimuth = (Azimuth + MAX_AZI_LEN  - correction_configuration_->startFrame[field]) * 2
        - correction_configuration_->azimuth[i]
        + correction_configuration_->getAzimuthAdjustV3(i, Azimuth) * LIDAR_AZIMUTH_UNIT;
      azimuth = (MAX_AZI_LEN  + azimuth) % MAX_AZI_LEN;
      
      {
  //      float xyDistance = unit.distance * m_cos_elevation_map_[i];
        float xyDistance = unit.distance * m_cos_elevation_map_[elevation];
        point.x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
        point.y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
  //      point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);
        point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[elevation]);
  //      std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
      }
    }else{
      int Azimuth = static_cast<int>(block->azimuth * LIDAR_AZIMUTH_UNIT + block->fine_azimuth);

      auto elevation = static_cast<int>(elev_angle_[i] * 100 * LIDAR_AZIMUTH_UNIT);
      elevation = (MAX_AZI_LEN + elevation) % MAX_AZI_LEN;      
      auto azimuth = static_cast<int>(Azimuth + MAX_AZI_LEN  - (azimuth_offset_[i] * 100 * LIDAR_AZIMUTH_UNIT) / 2);
      azimuth = (MAX_AZI_LEN  + azimuth) % MAX_AZI_LEN;
      
      {
  //      float xyDistance = unit.distance * m_cos_elevation_map_[i];
        float xyDistance = unit.distance * m_cos_elevation_map_[elevation];
        point.x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
        point.y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
  //      point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);
        point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[elevation]);
  //      std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
      }
    }

    point.intensity = unit.intensity;

//    double unix_second = static_cast<double>(timegm(&packet_.t));  // sensor-time (ppt/gps)
    double unix_second = packet_.unix_second;
    point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;
    /*
    point.time_stamp += (static_cast<double>(blockXTMOffsetSingle[i] + laserXTMOffset[i]) / 1000000.0f);

    if (packet_.return_mode == 0x3d){
      point.time_stamp =
        point.time_stamp + (static_cast<double>(blockXTMOffsetTriple[blockid] +
          laserXTMOffset[i]) /
                           1000000.0f);
    }
    else if (packet_.return_mode == 0x39 || packet_.return_mode == 0x3b || packet_.return_mode == 0x3c) {
      point.time_stamp =
        point.time_stamp + (static_cast<double>(blockXTMOffsetDual[blockid] +
          laserXTMOffset[i]) /
                           1000000.0f);
    } else {
      point.time_stamp = point.time_stamp + \
          (static_cast<double>(blockXTMOffsetSingle[blockid] + laserXTMOffset[i]) / \
          1000000.0f);
    }
    */

    point.return_type = packet_.return_mode;
    point.ring = i;
//    std::cout << point.x << "," << point.y << "," << point.z << std::endl;
    cld->points.emplace_back(point);
  }
}

/*

drivers::PointXYZIRADT PandarXTDecoder::build_point(
  int block_id, int unit_id, ReturnMode return_type)
{
  const auto & block = packet_.blocks[block_id];
  const auto & unit = block.units[unit_id];
  auto unix_second = static_cast<double>(timegm(&packet_.t));
  bool dual_return = (packet_.return_mode == DUAL_RETURN);
  PointXYZIRADT point{};

  double xyDistance = unit.distance * cosf(deg2rad(elev_angle_[unit_id]));

  point.x = static_cast<float>(
    xyDistance *
    sinf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
  point.y = static_cast<float>(
    xyDistance *
    cosf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
  point.z = static_cast<float>(unit.distance * sinf(deg2rad(elev_angle_[unit_id])));

  point.intensity = unit.intensity;
  point.distance = static_cast<float>(unit.distance);
  point.ring = unit_id;
  point.azimuth = static_cast<float>(block.azimuth) + std::round(azimuth_offset_[unit_id] * 100.0f);
  point.return_type = drivers::ReturnModeToInt(return_type);
  point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;
  point.time_stamp +=
    dual_return
      ? (static_cast<double>(block_offset_dual_[block_id] + firing_offset_[unit_id]) / 1000000.0f)
      : (static_cast<double>(block_offset_single_[block_id] + firing_offset_[unit_id]) /
         1000000.0f);

  return point;
}

drivers::PointCloudXYZIRADTPtr PandarXTDecoder::convert(size_t block_id)
{
  PointCloudXYZIRADTPtr block_pc(new PointCloudXYZIRADT);

  auto unix_second = static_cast<double>(timegm(&packet_.t));

  const auto & block = packet_.blocks[block_id];
  for (size_t unit_id = 0; unit_id < LASER_COUNT; ++unit_id) {
    const auto & unit = block.units[unit_id];
    // skip invalid points
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }

    block_pc->points.emplace_back(build_point(
      block_id, unit_id,
      (packet_.return_mode == STRONGEST_RETURN) ? drivers::ReturnMode::SINGLE_STRONGEST
                                                : drivers::ReturnMode::SINGLE_LAST));
  }
  return block_pc;
}

drivers::PointCloudXYZIRADTPtr PandarXTDecoder::convert_dual(size_t block_id)
{
  PointCloudXYZIRADTPtr block_pc(new PointCloudXYZIRADT);

  auto unix_second = static_cast<double>(timegm(&packet_.t));  // sensor-time (ppt/gps)

  auto head =
    block_id + ((sensor_configuration_->return_mode == drivers::ReturnMode::SINGLE_FIRST) ? 1 : 0);
  auto tail =
    block_id + ((sensor_configuration_->return_mode == drivers::ReturnMode::SINGLE_LAST) ? 1 : 2);

  for (size_t unit_id = 0; unit_id < LASER_COUNT; ++unit_id) {
    for (size_t i = head; i < tail; ++i) {
      PointXYZIRADT point{};
      const auto & block = packet_.blocks[i];
      const auto & unit = block.units[unit_id];
      // skip invalid points
      if (unit.distance <= 0.1 || unit.distance > 200.0) {
        continue;
      }
      double xyDistance = unit.distance * cosf(deg2rad(elev_angle_[unit_id]));

      point.x = static_cast<float>(
        xyDistance *
        sinf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
      point.y = static_cast<float>(
        xyDistance *
        cosf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
      point.z = static_cast<float>(unit.distance * sinf(deg2rad(elev_angle_[unit_id])));

      point.intensity = unit.intensity;
      point.distance = unit.distance;
      point.ring = unit_id;
      point.azimuth = block.azimuth + std::round(azimuth_offset_[unit_id] * 100.0f);

      point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;

      point.time_stamp +=
        (static_cast<double>(block_offset_dual_[block_id] + firing_offset_[unit_id]) / 1000000.0f);

      block_pc->points.emplace_back(point);
    }
  }
  return block_pc;
}
*/


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

  /*
  for(int i=0;i<HEAD_SIZE;i++){
    std::cout << static_cast<int>(buf[0 + i]) << ", ";
  }
  std::cout << std::endl;
  
  for(int i=0;i<HEAD_SIZE;i++){
    std::cout << static_cast<int>(buf[0 + i] & 0xff) << ", ";
  }
  std::cout << std::endl;
  */

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
  /*
  std::cout << "packet_.header.chProtocolMajor=" << packet_.header.chProtocolMajor << std::endl;
  std::cout << "int packet_.header.chProtocolMajor=" << static_cast<int>(packet_.header.chProtocolMajor) << std::endl;
  std::cout << "packet_.header.chProtocolMinor=" << packet_.header.chProtocolMinor << std::endl;
  std::cout << "int packet_.header.chProtocolMinor=" << static_cast<int>(packet_.header.chProtocolMinor) << std::endl;
//[hesai_driver_ros_wrapper_node-1] int packet_.header.chProtocolMajor=4
//[hesai_driver_ros_wrapper_node-1] int packet_.header.chProtocolMinor=3
  */
//  std::cout << "packet_.header.chBlockNumber=" << packet_.header.chBlockNumber << std::endl;
//  std::cout << "int packet_.header.chBlockNumber=" << static_cast<int>(packet_.header.chBlockNumber) << std::endl;
//  std::cout << "packet_.header.chLaserNumber=" << packet_.header.chLaserNumber << std::endl;
//  std::cout << "int packet_.header.chLaserNumber=" << static_cast<int>(packet_.header.chLaserNumber) << std::endl;


  for (int8_t block = 0; block < packet_.header.chBlockNumber; block++) {
    packet_.blocks[block].azimuth = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    packet_.blocks[block].fine_azimuth = buf[index + 2] & 0xff;
    index += BLOCK_HEADER_AZIMUTH;
//    std::cout << "packet_.blocks[" << static_cast<int>(block) << "].azimuth=" << packet_.blocks[block].azimuth << std::endl;
//    std::cout << "packet_.blocks[" << static_cast<int>(block) << "].fine_azimuth=" << packet_.blocks[block].fine_azimuth << std::endl;

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
  /*
  std::cout << "index=" << index << std::endl;
  for(int i=0;i<40;i++){
    std::cout << static_cast<int>(buf[index + i]) << ", ";
  }
  */

  index += RESERVED1_SIZE;  // skip reserved bytes
  packet_.shutdown_flg = buf[index] & 0xff;
  index += RESERVED2_SIZE;  // skip reserved bytes
  packet_.moter_speed = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));

  index += MOTER_SPEED_SIZE;
  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);

  index += TIMESTAMP_SIZE;
  packet_.return_mode = buf[index] & 0xff;
  index += RETURN_SIZE;
  index += FACTORY_SIZE;
  /*
  packet_.t.tm_year = (buf[index + 0] & 0xff) + 100;
  packet_.t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet_.t.tm_mday = buf[index + 2] & 0xff;
  packet_.t.tm_hour = buf[index + 3] & 0xff;
  packet_.t.tm_min = buf[index + 4] & 0xff;
  packet_.t.tm_sec = buf[index + 5] & 0xff;
  packet_.t.tm_isdst = 0;
  // in case of time error
  if (packet_.t.tm_year >= 200) {
    packet_.t.tm_year -= 100;
  }
  */
  if((buf[index] & 0xff) != 0){
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
    packet_.unix_second = static_cast<double>(mktime(&packet_.t));// + m_iTimeZoneSecond);
  }else{
    uint32_t utc_time_big = (buf[index + 2] & 0xff) | (buf[index + 3] & 0xff) << 8 |
                  ((buf[index + 4] & 0xff) << 16) | ((buf[index + 5] & 0xff) << 24);
    packet_.unix_second = ((utc_time_big >> 24) & 0xff) |
                    ((utc_time_big >> 8) & 0xff00) |
                    ((utc_time_big << 8) & 0xff0000) |
                    ((utc_time_big << 24));
  }
  /*
  std::cout << "packet_.t.tm_year=" << packet_.t.tm_year << std::endl;
  std::cout << "packet_.t.tm_mon=" << packet_.t.tm_mon << std::endl;
  std::cout << "packet_.t.tm_mday=" << packet_.t.tm_mday << std::endl;
  std::cout << "packet_.t.tm_hour=" << packet_.t.tm_hour << std::endl;
  std::cout << "packet_.t.tm_min=" << packet_.t.tm_min << std::endl;
  std::cout << "packet_.t.tm_sec=" << packet_.t.tm_sec << std::endl;
  for(int i=0;i<UTC_SIZE;i++){
    std::cout << static_cast<int>(buf[index + i]) << ", ";
  }
  std::cout << std::endl;
  */

  index += UTC_SIZE;
  /*
  for(int i=0;i<SEQUENCE_SIZE;i++){
    std::cout << static_cast<int>(buf[index + i]) << ", ";
  }
  std::cout << std::endl;
  */
  index += SEQUENCE_SIZE;


  return true;
}
}  // namespace pandar_at
}  // namespace drivers
}  // namespace nebula