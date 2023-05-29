#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xt_decoder.hpp"

#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xt.hpp"

namespace nebula
{
namespace drivers
{
namespace pandar_xt
{
PandarXTDecoder::PandarXTDecoder(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  sensor_calibration_ = calibration_configuration;

  for (size_t unit = 0; unit < LASER_COUNT; ++unit) {
    firing_time_offset_[unit] = 1.512f * static_cast<float>(unit) + 0.28f;
  }

  for (size_t block = 0; block < BLOCKS_PER_PACKET; ++block) {
    block_time_offset_single_return_[block] =
      3.28f - 50.00f * static_cast<float>(BLOCKS_PER_PACKET - block - 1);
    block_offset_dual_return_[block] =
      3.28f - 50.00f * (static_cast<float>(BLOCKS_PER_PACKET - block - 1) / 2.f);
  }

  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elevation_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
    elevation_angle_rad_[laser] = deg2rad(elevation_angle_[laser]);
    azimuth_offset_rad_[laser] = deg2rad(azimuth_offset_[laser]);
  }
  for (uint32_t i = 0; i < MAX_AZIMUTH_STEPS; i++) {  // precalculate sensor azimuth, unit 0.01 deg
    block_azimuth_rad_[i] = deg2rad(i / 100.);
  }

  scan_phase_ = static_cast<uint16_t>(sensor_configuration_->scan_phase * 100.0f);
  dual_return_distance_threshold_ = sensor_configuration_->dual_return_distance_threshold;
  last_phase_ = 0;
  has_scanned_ = false;
  scan_timestamp_ = -1;

  scan_pc_.reset(new NebulaPointCloud);
  scan_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
  overflow_pc_.reset(new NebulaPointCloud);
  overflow_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
}

bool PandarXTDecoder::hasScanned() { return has_scanned_; }

std::tuple<drivers::NebulaPointCloudPtr, double> PandarXTDecoder::get_pointcloud()
{
  return std::make_tuple(scan_pc_, scan_timestamp_);
}

void PandarXTDecoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    return;
  }

  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    auto unix_second = static_cast<double>(timegm(&packet_.t));  // sensor-time (ppt/gps)
    scan_timestamp_ = unix_second + static_cast<double>(packet_.usec) / 1000000.f;
    overflow_pc_.reset(new NebulaPointCloud);
    overflow_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
    has_scanned_ = false;
  }

  bool dual_return = packet_.return_mode == DUAL_RETURN;
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
}

drivers::NebulaPoint PandarXTDecoder::build_point(int block_id, int unit_id, uint8_t return_type)
{
  const auto & block = packet_.blocks[block_id];
  const auto & unit = block.units[unit_id];
  auto unix_second = static_cast<double>(timegm(&packet_.t));
  NebulaPoint point{};

  float xyDistance = unit.distance * cosf(elevation_angle_rad_[unit_id]);

  point.x = xyDistance * sinf(azimuth_offset_rad_[unit_id] + block_azimuth_rad_[block.azimuth]);
  point.y = xyDistance * cosf(azimuth_offset_rad_[unit_id] + block_azimuth_rad_[block.azimuth]);
  point.z = unit.distance * sinf(elevation_angle_rad_[unit_id]);

  point.intensity = unit.intensity;
  point.channel = unit_id;
  point.azimuth = block_azimuth_rad_[block_id] + azimuth_offset_rad_[unit_id];
  point.distance = unit.distance;
  point.elevation = elevation_angle_rad_[unit_id];
  point.return_type = return_type;
  if (scan_timestamp_ < 0) {  // invalid timestamp
    scan_timestamp_ = unix_second + static_cast<double>(packet_.usec) / 1000000.f;
  }
  auto offset =
    (static_cast<double>(block_offset_dual_return_[block_id] + firing_time_offset_[unit_id]) /
     1000000.0f);
  auto point_stamp =
    (unix_second + offset + static_cast<double>(packet_.usec) / 1000000.f - scan_timestamp_);
  if (point_stamp < 0) {
    point.time_stamp = 0;
  } else {
    point.time_stamp = static_cast<uint32_t>(point_stamp * 10e9);
  }
  return point;
}

drivers::NebulaPointCloudPtr PandarXTDecoder::convert(size_t block_id)
{
  NebulaPointCloudPtr block_pc(new NebulaPointCloud);

  const auto & block = packet_.blocks[block_id];
  for (size_t unit_id = 0; unit_id < LASER_COUNT; ++unit_id) {
    const auto & unit = block.units[unit_id];
    // skip invalid points
    if (unit.distance < MIN_RANGE || unit.distance > MAX_RANGE) {
      continue;
    }

    switch (packet_.return_mode) {
      case STRONGEST_RETURN:
        block_pc->points.emplace_back(
          build_point(block_id, unit_id, static_cast<uint8_t>(drivers::ReturnType::STRONGEST)));
        break;

      case LAST_RETURN:
        block_pc->points.emplace_back(
          build_point(block_id, unit_id, static_cast<uint8_t>(drivers::ReturnType::LAST)));
        break;

      default:
        block_pc->points.emplace_back(
          build_point(block_id, unit_id, static_cast<uint8_t>(drivers::ReturnType::UNKNOWN)));
        break;
    }
  }
  return block_pc;
}

drivers::NebulaPointCloudPtr PandarXTDecoder::convert_dual(size_t block_id)
{
  NebulaPointCloudPtr block_pc(new NebulaPointCloud);

  auto unix_second = static_cast<double>(timegm(&packet_.t));  // sensor-time (ppt/gps)

  auto head =
    block_id + ((sensor_configuration_->return_mode == drivers::ReturnMode::FIRST) ? 1 : 0);
  auto tail =
    block_id + ((sensor_configuration_->return_mode == drivers::ReturnMode::LAST) ? 1 : 2);

  for (size_t unit_id = 0; unit_id < LASER_COUNT; ++unit_id) {
    for (size_t i = head; i < tail; ++i) {
      NebulaPoint point{};
      const auto & block = packet_.blocks[i];
      const auto & unit = block.units[unit_id];
      const auto & another_block = packet_.blocks[(i + 1) % 2];
      const auto & another_unit = another_block.units[unit_id];
      // skip invalid points
      if (unit.distance <= MIN_RANGE || unit.distance > MAX_RANGE) {
        continue;
      }
      point.intensity = unit.intensity;
      auto another_intensity = another_unit.intensity;
      bool identical_flg = false;
      if (point.intensity == another_intensity && unit.distance == another_unit.distance) {
        identical_flg = true;
      }

      float xyDistance = unit.distance * cosf(elevation_angle_rad_[unit_id]);

      point.x = xyDistance * sinf(azimuth_offset_rad_[unit_id] + block_azimuth_rad_[block.azimuth]);
      point.y = xyDistance * cosf(azimuth_offset_rad_[unit_id] + block_azimuth_rad_[block.azimuth]);
      point.z = unit.distance * sinf(elevation_angle_rad_[unit_id]);
      point.channel = unit_id;
      point.azimuth = block_azimuth_rad_[block.azimuth] + azimuth_offset_rad_[unit_id];
      point.distance = unit.distance;

      if (scan_timestamp_ < 0) {  // invalid timestamp
        scan_timestamp_ = unix_second + static_cast<double>(packet_.usec) / 1000000.f;
      }
      auto offset =
        (static_cast<double>(block_offset_dual_return_[block_id] + firing_time_offset_[unit_id]) /
         1000000.0f);
      auto point_stamp =
        (unix_second + offset + static_cast<double>(packet_.usec) / 1000000.f - scan_timestamp_);
      if (point_stamp < 0) {
        point.time_stamp = 0;
      } else {
        point.time_stamp = static_cast<uint32_t>(point_stamp * 10e9);
      }

      if (identical_flg) {
        point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::IDENTICAL);
      } else if (i % 2 == 0) {
        if (point.intensity < another_intensity) {
          point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::LAST_WEAK);
        } else {
          point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::STRONGEST);
        }
      } else {
        if (point.intensity > another_intensity) {
          point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::STRONGEST);
        } else {
          point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::SECOND_STRONGEST);
        }
      }

      block_pc->points.emplace_back(point);
    }
  }
  return block_pc;
}

bool PandarXTDecoder::parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (pandar_packet.size != PACKET_SIZE) {
    return false;
  }
  const uint8_t * buf = &pandar_packet.data[0];

  size_t index = 0;
  // Parse 12 Bytes Header
  packet_.header.sob = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
  packet_.header.chProtocolMajor = buf[index + 2] & 0xff;
  packet_.header.chProtocolMinor = buf[index + 3] & 0xff;
  packet_.header.chLaserNumber = buf[index + 6] & 0xff;
  packet_.header.chBlockNumber = buf[index + 7] & 0xff;
  packet_.header.chReturnType = buf[index + 8] & 0xff;
  packet_.header.chDisUnit = buf[index + 9] & 0xff;
  index += HEAD_SIZE;

  if (packet_.header.sob != 0xEEFF) {
    // Error Start of Packet!
    return false;
  }

  for (int8_t block = 0; block < packet_.header.chBlockNumber; block++) {
    packet_.blocks[block].azimuth = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
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

  index += RESERVED_SIZE;  // skip reserved bytes
  packet_.return_mode = buf[index] & 0xff;

  index += RETURN_SIZE;
  index += ENGINE_VELOCITY;

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

  index += UTC_SIZE;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
  index += TIMESTAMP_SIZE;
  index += FACTORY_SIZE;

  return true;
}
}  // namespace pandar_xt
}  // namespace drivers
}  // namespace nebula
