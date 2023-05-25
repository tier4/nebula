#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_40_decoder.hpp"

#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_40.hpp"

#include <cmath>
#include <utility>

namespace nebula
{
namespace drivers
{
namespace pandar_40
{
Pandar40Decoder::Pandar40Decoder(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  sensor_calibration_ = calibration_configuration;

  vertical_laser_firing_order_ = {7,  19, 14, 26, 6,  18, 4,  32, 36, 0,  10, 22, 17, 29,
                                  9,  21, 5,  33, 37, 1,  13, 25, 20, 30, 12, 8,  24, 34,
                                  38, 2,  16, 28, 23, 31, 15, 11, 27, 35, 39, 3};

  firing_time_offset_ = {42.22, 28.47, 16.04, 3.62,  45.49, 31.74, 47.46, 54.67, 20.62, 33.71,
                         40.91, 8.19,  20.62, 27.16, 50.73, 8.19,  14.74, 36.98, 45.49, 52.7,
                         23.89, 31.74, 38.95, 11.47, 18.65, 25.19, 48.76, 6.23,  12.77, 35.01,
                         21.92, 9.5,   43.52, 29.77, 17.35, 4.92,  42.22, 28.47, 16.04, 3.62};

  for (size_t block = 0; block < BLOCKS_PER_PACKET; ++block) {
    block_time_offset_single_return_[block] =
      55.56f * static_cast<float>(BLOCKS_PER_PACKET - block - 1) + 28.58f;
    block_time_offset_dual_return_[block] =
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
  scan_timestamp_ = std::numeric_limits<uint32_t>::max();

  scan_pc_.reset(new NebulaPointCloud);
  scan_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
  overflow_pc_.reset(new NebulaPointCloud);
  overflow_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
}

bool Pandar40Decoder::hasScanned() { return has_scanned_; }

std::tuple<drivers::NebulaPointCloudPtr, double> Pandar40Decoder::get_pointcloud()
{
  return std::make_tuple(scan_pc_, scan_timestamp_);
}

void Pandar40Decoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    return;
  }

  if (has_scanned_) {
    scan_timestamp_ = std::numeric_limits<uint32_t>::max();
    scan_pc_ = overflow_pc_;
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

drivers::NebulaPoint Pandar40Decoder::build_point(
  size_t block_id, size_t unit_id, uint8_t return_type)
{
  const auto & block = packet_.blocks[block_id];
  const auto & unit = block.units[unit_id];
  auto unix_second = static_cast<double>(timegm(&packet_.t));

  bool dual_return = (packet_.return_mode == DUAL_RETURN);
  NebulaPoint point{};

  float xyDistance = unit.distance * cos_elevation_angle_[unit_id];

  point.x = xyDistance * sinf(azimuth_offset_rad_[unit_id] + block_azimuth_rad_[block.azimuth]);
  point.y = xyDistance * cosf(azimuth_offset_rad_[unit_id] + block_azimuth_rad_[block.azimuth]);
  point.z = unit.distance * sin_elevation_angle_[unit_id];

  point.intensity = unit.intensity;
  point.channel = unit_id;
  point.azimuth = block_azimuth_rad_[packet_.blocks[block_id].azimuth];
  point.elevation = elevation_angle_rad_[unit_id];
  point.return_type = return_type;

  if (std::numeric_limits<uint32_t>::max() == scan_timestamp_) {  // invalid timestamp use current
                                                                  // block stamp
    scan_timestamp_ = unix_second + static_cast<double>(packet_.usec) / 1000000.f;
  }
  if (!block.azimuth) {  // initial azimuth, set as initial
    scan_timestamp_ = unix_second + static_cast<double>(packet_.usec) / 1000000.f;
  }
  auto offset = dual_return
                  ? (static_cast<double>(
                       block_time_offset_dual_return_[block_id] + firing_time_offset_[unit_id]) /
                     1000000.0f)
                  : (static_cast<double>(
                       block_time_offset_single_return_[block_id] + firing_time_offset_[unit_id]) /
                     1000000.0f);
  point.time_stamp = static_cast<uint32_t>(
    (unix_second + offset + static_cast<double>(packet_.usec) / 1000000.f - scan_timestamp_) *
    10e9);

  return point;
}

drivers::NebulaPointCloudPtr Pandar40Decoder::convert(size_t block_id)
{
  drivers::NebulaPointCloudPtr block_pc(new NebulaPointCloud);

  for (auto unit_id : vertical_laser_firing_order_) {
    block_pc->points.emplace_back(build_point(
      block_id, unit_id,
      (packet_.return_mode == STRONGEST_RETURN)
        ? static_cast<uint8_t>(
            drivers::ReturnType::STRONGEST)  // drivers::ReturnMode::SINGLE_STRONGEST
        : static_cast<uint8_t>(drivers::ReturnType::LAST)));  // drivers::ReturnMode::SINGLE_LAST
  }
  return block_pc;
}

drivers::NebulaPointCloudPtr Pandar40Decoder::convert_dual(size_t block_id)
{
  //   Under the Dual Return mode, the measurements from each round of firing are stored in two
  //   adjacent blocks:
  // · The even number block is the last return, and the odd number block is the strongest return
  // · If the last and strongest returns coincide, the second strongest return will be placed in the
  // odd number block · The Azimuth changes every two blocks · Important note: Hesai datasheet block
  // numbering starts from 0, not 1, so odd/even are reversed here
  drivers::NebulaPointCloudPtr block_pc(new NebulaPointCloud);

  size_t even_block_id = block_id;
  size_t odd_block_id = block_id + 1;
  const auto & even_block = packet_.blocks[even_block_id];
  const auto & odd_block = packet_.blocks[odd_block_id];
  //  auto sensor_return_mode = sensor_configuration_->return_mode;

  for (auto unit_id : vertical_laser_firing_order_) {
    const auto & even_unit = even_block.units[unit_id];
    const auto & odd_unit = odd_block.units[unit_id];

    bool even_usable =
      (even_unit.distance < MIN_RANGE || even_unit.distance > MAX_RANGE) ? false : true;
    bool odd_usable =
      (odd_unit.distance < MIN_RANGE || odd_unit.distance > MAX_RANGE) ? false : true;

    // maybe always dual return mode in convert_dual
    // If the two returns are too close, only return the last one
    if (
      (abs(even_unit.distance - odd_unit.distance) < dual_return_distance_threshold_) &&
      even_usable) {
      block_pc->emplace_back(build_point(
        even_block_id, unit_id,
        static_cast<uint8_t>(drivers::ReturnType::IDENTICAL)));  // drivers::ReturnMode::DUAL_ONLY
    } else if (even_unit.intensity >= odd_unit.intensity) {
      // Strongest return is in even block when it is also the last
      if (odd_usable) {
        block_pc->emplace_back(build_point(
          odd_block_id, unit_id,
          static_cast<uint8_t>(
            drivers::ReturnType::FIRST_WEAK)));  // drivers::ReturnMode::DUAL_WEAK_FIRST
      }
      if (even_usable) {
        block_pc->emplace_back(build_point(
          even_block_id, unit_id,
          static_cast<uint8_t>(
            drivers::ReturnType::STRONGEST)));  // drivers::ReturnMode::DUAL_STRONGEST_LAST
      }
    } else {
      // Normally, strongest return is in odd block and last return is in even block
      if (odd_usable) {
        block_pc->emplace_back(build_point(
          odd_block_id, unit_id,
          static_cast<uint8_t>(
            drivers::ReturnType::STRONGEST)));  // drivers::ReturnMode::DUAL_STRONGEST_FIRST
      }
      if (even_usable) {
        block_pc->emplace_back(build_point(
          even_block_id, unit_id,
          static_cast<uint8_t>(
            drivers::ReturnType::LAST_WEAK)));  // drivers::ReturnMode::DUAL_WEAK_LAST
      }
    }
    //    }
  }
  return block_pc;
}

bool Pandar40Decoder::parsePacket(const pandar_msgs::msg::PandarPacket & raw_packet)
{
  if (raw_packet.size != PACKET_SIZE && raw_packet.size != PACKET_SIZE + SEQ_NUM_SIZE) {
    // packet size mismatch !
    return false;
  }

  // auto buf = raw_packet.data;
  const uint8_t * buf = &raw_packet.data[0];

  int index = 0;
  for (size_t i = 0; i < BLOCKS_PER_PACKET; i++) {
    Block & block = packet_.blocks[i];

    block.sob = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    block.azimuth = (buf[index + 2] & 0xff) | ((buf[index + 3] & 0xff) << 8);
    index += SOB_ANGLE_SIZE;

    for (size_t j = 0; j < LASER_COUNT; j++) {
      Unit & unit = block.units[j];
      uint32_t range = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      unit.distance = (static_cast<float>(range)) * LASER_RETURN_TO_DISTANCE_RATE;
      unit.intensity = (buf[index + 2] & 0xff);

      index += RAW_MEASURE_SIZE;
    }
  }

  index += RESERVE_SIZE;

  index += REVOLUTION_SIZE;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
  packet_.usec %= 1000000;

  index += TIMESTAMP_SIZE;
  packet_.return_mode = buf[index] & 0xff;

  index += FACTORY_INFO_SIZE + RETURN_SIZE;

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

  return true;
}

}  // namespace pandar_40
}  // namespace drivers
}  // namespace nebula
