#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_qt_128_decoder.hpp"

#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_qt_128.hpp"

#include <cmath>

namespace nebula
{
namespace drivers
{
namespace pandar_qt_128
{
PandarQT128Decoder::PandarQT128Decoder(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  sensor_calibration_ = calibration_configuration;

  {
    std::string sbuf;
    std::stringstream ss(PandarQT128_TL1);
    while (std::getline(ss, sbuf, '\n')) {
      int id;
      float val;
      sscanf(sbuf.c_str(), "%d,%f", &id, &val);
      firing_time_offset1_[id] = val;
    }
  }
  {
    std::string sbuf;
    std::stringstream ss(PandarQT128_TL2);
    while (std::getline(ss, sbuf, '\n')) {
      int id;
      float val;
      sscanf(sbuf.c_str(), "%d,%f", &id, &val);
      firing_time_offset2_[id] = val;
    }
  }
  for (size_t block = 0; block < BLOCKS_PER_PACKET; ++block) {
    block_time_offset_single_[block] = 9.00f + 111.11f * static_cast<float>(block);
    block_time_offset_dual_[block] = 9.00f;
  }

  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elevation_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
    elevation_angle_rad_[laser] = deg2rad(elevation_angle_[laser]);
    azimuth_offset_rad_[laser] = deg2rad(azimuth_offset_[laser]);
    cos_elevation_angle_[laser] = cosf(elevation_angle_rad_[laser]);
    sin_elevation_angle_[laser] = sinf(elevation_angle_rad_[laser]);
  }
  for (uint32_t i = 0; i < MAX_AZIMUTH_STEPS; i++) {  // precalculate sensor azimuth, unit 0.01 deg
    block_azimuth_rad_[i] = deg2rad(i / 100.);
  }

  scan_phase_ = static_cast<uint16_t>(sensor_configuration_->scan_phase * 100.0f);
  dual_return_distance_threshold_ = sensor_configuration_->dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;
  first_timestamp_tmp = std::numeric_limits<uint32_t>::max();
  first_timestamp_ = first_timestamp_tmp;

  scan_pc_.reset(new NebulaPointCloud);
  scan_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
  overflow_pc_.reset(new NebulaPointCloud);
  overflow_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
}

bool PandarQT128Decoder::hasScanned() { return has_scanned_; }

std::tuple<drivers::NebulaPointCloudPtr, double> PandarQT128Decoder::get_pointcloud()
{
  return std::make_tuple(scan_pc_, first_timestamp_);
}

void PandarQT128Decoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    return;
  }

  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    first_timestamp_ = first_timestamp_tmp;
    first_timestamp_tmp = std::numeric_limits<uint32_t>::max();
    overflow_pc_.reset(new NebulaPointCloud);
    overflow_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
    has_scanned_ = false;
  }

  bool dual_return = is_dual_return();

  drivers::NebulaPointCloudPtr block_pc(new NebulaPointCloud);
  int current_phase;
  int cnt2;
  if (dual_return) {
    for (size_t block_id = 0; block_id < BLOCKS_PER_PACKET; block_id += 2) {
      auto block1_pt = convert(block_id);
      auto block2_pt = convert(block_id + 1);
      size_t block1size = block1_pt->points.size();
      cnt2 = 0;
      for (size_t i = 0; i < block1size; i++) {
        if (
          fabsf(
            packet_.blocks[block_id + 1].units[i].distance -
            packet_.blocks[block_id].units[i].distance) > dual_return_distance_threshold_) {
          block_pc->points.emplace_back(block1_pt->points[i]);
          block_pc->points.emplace_back(block2_pt->points[i]);
          cnt2++;
        } else {
          block1_pt->points[i].return_type = static_cast<uint8_t>(ReturnType::IDENTICAL);
          block_pc->points.emplace_back(block1_pt->points[i]);
        }
      }
      current_phase =
        (static_cast<int>(packet_.blocks[block_id + 1].azimuth) - scan_phase_ + 36000) % 36000;
    }
  } else  // single
  {
    for (size_t block_id = 0; block_id < BLOCKS_PER_PACKET; block_id++) {
      block_pc = convert(block_id);
      *block_pc += *block_pc;
      current_phase =
        (static_cast<int>(packet_.blocks[block_id].azimuth) - scan_phase_ + 36000) % 36000;
    }
  }
  if (current_phase > last_phase_ && !has_scanned_) {
    *scan_pc_ += *block_pc;
  } else {
    *overflow_pc_ += *block_pc;
    has_scanned_ = true;
  }
  last_phase_ = current_phase;
}

drivers::NebulaPoint PandarQT128Decoder::build_point(
  size_t block_id, size_t unit_id, bool dual_return, const uint32_t & unix_second)
{
  const auto & block = packet_.blocks[block_id];
  const auto & unit = block.units[unit_id];
  bool first_flg = false;
  if (unix_second < first_timestamp_tmp) {
    first_timestamp_tmp = unix_second;
    first_flg = true;
  }
  NebulaPoint point{};
  float xyDistance = unit.distance * cos_elevation_angle_[unit_id];

  point.x = xyDistance * sinf(azimuth_offset_rad_[unit_id] + block_azimuth_rad_[block.azimuth]);
  point.y = xyDistance * cosf(azimuth_offset_rad_[unit_id] + block_azimuth_rad_[block.azimuth]);
  point.z = unit.distance * sin_elevation_angle_[unit_id];

  point.intensity = unit.intensity;
  point.channel = unit_id;
  point.azimuth = block_azimuth_rad_[block_id] + azimuth_offset_rad_[unit_id];
  point.elevation = elevation_angle_rad_[unit_id];
  if (dual_return && block_id == 1) {
    point.return_type = second_return_type_;
  } else {
    point.return_type = first_return_type_;
  }

  point.time_stamp = (static_cast<double>(packet_.usec)) / 1000000.0;
  if (!first_flg) {
    point.time_stamp += unix_second - first_timestamp_tmp;
  }
  if (0 < packet_.mode_flag) {
    point.time_stamp +=
      dual_return
        ? (static_cast<double>(block_time_offset_dual_[block_id] + firing_time_offset1_[unit_id]) /
           1000000.0f)
        : (static_cast<double>(
             block_time_offset_single_[block_id] + firing_time_offset1_[unit_id]) /
           1000000.0f);
  } else {
    point.time_stamp +=
      dual_return
        ? (static_cast<double>(block_time_offset_dual_[block_id] + firing_time_offset2_[unit_id]) /
           1000000.0f)
        : (static_cast<double>(
             block_time_offset_single_[block_id] + firing_time_offset2_[unit_id]) /
           1000000.0f);
  }

  return point;
}

drivers::NebulaPointCloudPtr PandarQT128Decoder::convert(size_t block_id)
{
  NebulaPointCloudPtr block_pc(new NebulaPointCloud);

  bool dual_return = is_dual_return();
  auto unix_second = static_cast<double>(timegm(&packet_.t));
  for (size_t unit_id = 0; unit_id < LASER_COUNT; ++unit_id) {
    auto distance = packet_.blocks[dual_return ? 0 : block_id].units[unit_id].distance;
    if (distance < MIN_RANGE || MAX_RANGE < distance) {
      continue;
    }

    block_pc->points.emplace_back(build_point(block_id, unit_id, dual_return, unix_second));
  }
  return block_pc;
}

drivers::NebulaPointCloudPtr PandarQT128Decoder::convert_dual(size_t block_id)
{
  return convert(block_id);
}

bool PandarQT128Decoder::is_dual_return()
{
  switch (packet_.return_mode) {
    case DUAL_LAST_STRONGEST_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::LAST);
      second_return_type_ = static_cast<uint8_t>(ReturnType::STRONGEST);
      return true;
    case DUAL_FIRST_LAST_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::FIRST);
      second_return_type_ = static_cast<uint8_t>(ReturnType::LAST);
      return true;
    case DUAL_FIRST_STRONGEST_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::FIRST);
      second_return_type_ = static_cast<uint8_t>(ReturnType::STRONGEST);
      return true;
    case DUAL_STRONGEST_2ndSTRONGEST_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::STRONGEST);
      second_return_type_ = static_cast<uint8_t>(ReturnType::SECOND_STRONGEST);
      return true;
    case DUAL_FIRST_SECOND_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::FIRST);
      second_return_type_ = static_cast<uint8_t>(ReturnType::SECOND);
      return true;
    case SINGLE_FIRST_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::FIRST);
      break;
    case SINGLE_SECOND_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::SECOND);
      break;
    case SINGLE_STRONGEST_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::STRONGEST);
      break;
    case SINGLE_LAST_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::LAST);
      break;
    default:
      first_return_type_ = static_cast<uint8_t>(ReturnType::UNKNOWN);
      break;
  }
  return false;
}

bool PandarQT128Decoder::parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (pandar_packet.size != PACKET_SIZE && pandar_packet.size != PACKET_WITHOUT_UDPSEQ_CRC_SIZE) {
    return false;
  }
  const uint8_t * buf = &pandar_packet.data[0];

  int index = 0;
  // Parse 12 Bytes Header
  packet_.header.sob = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
  packet_.header.chProtocolMajor = buf[index + 2] & 0xff;
  packet_.header.chProtocolMinor = buf[index + 3] & 0xff;
  packet_.header.chLaserNumber = buf[index + 6] & 0xff;
  packet_.header.chBlockNumber = buf[index + 7] & 0xff;
  packet_.header.chReturnType = buf[index + 8] & 0xff;  // First Block Return (Reserved)
  packet_.header.chDisUnit = buf[index + 9] & 0xff;
  index += HEAD_SIZE;

  if (packet_.header.sob != 0xEEFF) {
    return false;
  }

  for (size_t block = 0; block < static_cast<size_t>(packet_.header.chBlockNumber); block++) {
    packet_.blocks[block].azimuth = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    index += BLOCK_HEADER_AZIMUTH;

    for (size_t unit = 0; unit < packet_.header.chLaserNumber; unit++) {
      unsigned int unRange = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      packet_.blocks[block].units[unit].distance =
        (static_cast<float>(unRange * packet_.header.chDisUnit)) / 1000.f;
      packet_.blocks[block].units[unit].intensity = (buf[index + 2] & 0xff);
      packet_.blocks[block].units[unit].confidence = (buf[index + 3] & 0xff);
      index += UNIT_SIZE;
    }
  }

  index += SKIP_SIZE;
  packet_.mode_flag = buf[index] & 0x01;  // Mode Flag
  index += MODE_FLAG_SIZE;
  index += RESERVED3_SIZE;
  packet_.return_mode = buf[index] & 0xff;  // Return Mode
  index += RETURN_MODE_SIZE;

  packet_.t.tm_year = (buf[index + 0] & 0xff) + 100;
  packet_.t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet_.t.tm_mday = buf[index + 2] & 0xff;
  packet_.t.tm_hour = buf[index + 3] & 0xff;
  packet_.t.tm_min = buf[index + 4] & 0xff;
  packet_.t.tm_sec = buf[index + 5] & 0xff;
  packet_.t.tm_isdst = 0;
  index += UTC_SIZE;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
  index += TIMESTAMP_SIZE;

  // in case of time error
  if (packet_.t.tm_year >= 200) {
    packet_.t.tm_year -= 100;
  }

  return true;
}
}  // namespace pandar_qt_128
}  // namespace drivers
}  // namespace nebula
