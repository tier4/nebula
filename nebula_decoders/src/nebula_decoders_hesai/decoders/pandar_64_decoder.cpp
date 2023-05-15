#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_64_decoder.hpp"

#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_64.hpp"

#include <cmath>

namespace nebula
{
namespace drivers
{
namespace pandar_64
{
Pandar64Decoder::Pandar64Decoder(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  sensor_calibration_ = calibration_configuration;

  firing_time_offset_ = {
    23.18,  21.876, 20.572, 19.268, 17.964, 16.66,  11.444, 46.796, 7.532,  36.956, 50.732,
    54.668, 40.892, 44.828, 31.052, 34.988, 48.764, 52.7,   38.924, 42.86,  29.084, 33.02,
    46.796, 25.148, 36.956, 50.732, 27.116, 40.892, 44.828, 31.052, 34.988, 48.764, 25.148,
    38.924, 42.86,  29.084, 33.02,  52.7,   6.228,  54.668, 15.356, 27.116, 10.14,  23.18,
    4.924,  21.876, 14.052, 17.964, 8.836,  19.268, 3.62,   20.572, 12.748, 16.66,  7.532,
    11.444, 6.228,  15.356, 10.14,  4.924,  3.62,   14.052, 8.836,  12.748};

  for (size_t block = 0; block < BLOCKS_PER_PACKET; ++block) {
    block_time_offset_single_[block] =
      55.56f * static_cast<float>(BLOCKS_PER_PACKET - block - 1) + 28.58f;
    block_time_offset_dual_[block] =
      55.56f * (static_cast<float>(BLOCKS_PER_PACKET - block - 1) / 2.f) + 28.58f;
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

bool Pandar64Decoder::hasScanned() { return has_scanned_; }

std::tuple<drivers::NebulaPointCloudPtr, double> Pandar64Decoder::get_pointcloud()
{
  return std::make_tuple(scan_pc_, first_timestamp_);
}

void Pandar64Decoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
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

  bool dual_return = (packet_.return_mode == DUAL_RETURN);
  auto step = dual_return ? 2 : 1;

  if (!dual_return) {
    if (
      (packet_.return_mode == STRONGEST_RETURN &&
       sensor_configuration_->return_mode != drivers::ReturnMode::STRONGEST) ||
      (packet_.return_mode == LAST_RETURN &&
       sensor_configuration_->return_mode != drivers::ReturnMode::LAST)) {
      // sensor config, driver mismatched
    }
  }
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

drivers::NebulaPoint Pandar64Decoder::build_point(
  size_t block_id, size_t unit_id, uint8_t return_type)
{
  const auto & block = packet_.blocks[block_id];
  const auto & unit = block.units[unit_id];
  auto unix_second = static_cast<double>(timegm(&packet_.t));
  if (unix_second < first_timestamp_tmp) {
    first_timestamp_tmp = unix_second;
  }
  bool dual_return = (packet_.return_mode == DUAL_RETURN);
  NebulaPoint point{};

  float xyDistance = unit.distance * cos_elevation_angle_[unit_id];

  point.x = xyDistance * sinf(azimuth_offset_rad_[unit_id] + block_azimuth_rad_[block.azimuth]);
  point.y = xyDistance * cosf(azimuth_offset_rad_[unit_id] + block_azimuth_rad_[block.azimuth]);
  point.z = unit.distance * sin_elevation_angle_[unit_id];

  point.intensity = unit.intensity;
  point.channel = unit_id;
  point.azimuth = block_azimuth_rad_[block_id] + azimuth_offset_rad_[unit_id];
  point.elevation = elevation_angle_rad_[unit_id];
  point.return_type = return_type;
  point.time_stamp = (static_cast<double>(packet_.usec)) / 1000000.0;
  point.time_stamp +=
    dual_return
      ? (static_cast<double>(block_time_offset_dual_[block_id] + firing_time_offset_[unit_id]) /
         1000000.0f)
      : (static_cast<double>(block_time_offset_single_[block_id] + firing_time_offset_[unit_id]) /
         1000000.0f);

  return point;
}

drivers::NebulaPointCloudPtr Pandar64Decoder::convert(size_t block_id)
{
  NebulaPointCloudPtr block_pc(new NebulaPointCloud);

  const auto & block = packet_.blocks[block_id];
  for (size_t unit_id = 0; unit_id < LASER_COUNT; ++unit_id) {
    const auto & unit = block.units[unit_id];
    // skip invalid points
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }
    block_pc->points.emplace_back(build_point(
      block_id, unit_id,
      (packet_.return_mode == STRONGEST_RETURN)
        ? static_cast<uint8_t>(
            drivers::ReturnType::STRONGEST)  // drivers::ReturnMode::SINGLE_STRONGEST
        : static_cast<uint8_t>(drivers::ReturnType::LAST)));  // drivers::ReturnMode::SINGLE_LAST
  }
  return block_pc;
}

drivers::NebulaPointCloudPtr Pandar64Decoder::convert_dual(size_t block_id)
{
  //     Under the Dual Return mode, the ranging data from each firing is stored in two adjacent
  //     blocks:
  //   · The even number block is the first return
  //   · The odd number block is the last return
  //   · The Azimuth changes every two blocks
  //   · Important note: Hesai datasheet block numbering starts from 0, not 1, so odd/even are
  //   reversed here
  NebulaPointCloudPtr block_pc(new NebulaPointCloud);

  size_t even_block_id = block_id;
  size_t odd_block_id = block_id + 1;
  const auto & even_block = packet_.blocks[even_block_id];
  const auto & odd_block = packet_.blocks[odd_block_id];

  for (size_t unit_id = 0; unit_id < LASER_COUNT; ++unit_id) {
    const auto & even_unit = even_block.units[unit_id];
    const auto & odd_unit = odd_block.units[unit_id];

    bool even_usable = !(even_unit.distance < MIN_RANGE || even_unit.distance > MAX_RANGE);
    bool odd_usable = !(odd_unit.distance < MIN_RANGE || odd_unit.distance > MAX_RANGE);

    // If the two returns are too close, only return the last one
    if (
      (abs(even_unit.distance - odd_unit.distance) < dual_return_distance_threshold_) &&
      odd_usable) {
      block_pc->points.emplace_back(
        build_point(odd_block_id, unit_id, static_cast<uint8_t>(drivers::ReturnType::IDENTICAL)));
    } else {
      if (even_usable) {
        block_pc->points.emplace_back(
          build_point(even_block_id, unit_id, static_cast<uint8_t>(drivers::ReturnType::FIRST)));
      }
      if (odd_usable) {
        block_pc->points.emplace_back(
          build_point(odd_block_id, unit_id, static_cast<uint8_t>(drivers::ReturnType::LAST)));
      }
    }
  }
  return block_pc;
}

bool Pandar64Decoder::parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (pandar_packet.size != PACKET_SIZE && pandar_packet.size != PACKET_WITHOUT_UDPSEQ_SIZE) {
    return false;
  }
  const uint8_t * buf = &pandar_packet.data[0];

  size_t index = 0;
  // Parse 12 Bytes Header
  packet_.header.sob = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
  packet_.header.chLaserNumber = buf[index + 2] & 0xff;
  packet_.header.chBlockNumber = buf[index + 3] & 0xff;
  packet_.header.chReturnType = buf[index + 4] & 0xff;
  packet_.header.chDisUnit = buf[index + 5] & 0xff;
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
      index += UNIT_SIZE;
    }  // end fot laser
  }    // end for block

  index += RESERVED_SIZE;  // skip reserved bytes
  index += ENGINE_VELOCITY;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
  index += TIMESTAMP_SIZE;

  packet_.return_mode = buf[index] & 0xff;

  index += RETURN_SIZE;
  index += FACTORY_SIZE;

  packet_.t.tm_year = (buf[index + 0] & 0xff) + 100;
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

  index += UTC_SIZE;

  return true;
}  // parsePacket
}  // namespace pandar_64
}  // namespace drivers
}  // namespace nebula
