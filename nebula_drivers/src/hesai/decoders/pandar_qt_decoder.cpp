#include "hesai/decoders/pandar_qt_decoder.hpp"

#include <cmath>

#include "hesai/decoders/pandar_qt.hpp"

namespace nebula
{
namespace drivers
{
namespace pandar_qt
{
PandarQTDecoder::PandarQTDecoder(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  sensor_calibration_ = calibration_configuration;

  firing_offset_ = {
    12.31,  14.37,  16.43,  18.49,  20.54,  22.6,   24.66,  26.71,  29.16,  31.22,  33.28,
    35.34,  37.39,  39.45,  41.5,   43.56,  46.61,  48.67,  50.73,  52.78,  54.84,  56.9,
    58.95,  61.01,  63.45,  65.52,  67.58,  69.63,  71.69,  73.74,  75.8,   77.86,  80.9,
    82.97,  85.02,  87.08,  89.14,  91.19,  93.25,  95.3,   97.75,  99.82,  101.87, 103.93,
    105.98, 108.04, 110.1,  112.15, 115.2,  117.26, 119.32, 121.38, 123.43, 125.49, 127.54,
    129.6,  132.05, 134.11, 136.17, 138.22, 140.28, 142.34, 144.39, 146.45,
  };

  for (size_t block = 0; block < BLOCKS_PER_PACKET; ++block) {
    block_offset_single_[block] = 25.71f + 500.00f / 3.0f * static_cast<float>(block);
    block_offset_dual_[block] = 25.71f + 500.00f / 3.0f * (static_cast<float>(block) / 2.f);
  }

  // TODO: add calibration data validation
  // if(calibration.elev_angle_map.size() != num_lasers_){
  //   // calibration data is not valid!
  // }
  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
  }

  scan_phase_ = static_cast<uint16_t>(sensor_configuration_->scan_phase * 100.0f);
  dual_return_distance_threshold_ = sensor_configuration_->dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;

  scan_pc_.reset(new PointCloudXYZIRADT);
  overflow_pc_.reset(new PointCloudXYZIRADT);
}

bool PandarQTDecoder::hasScanned() { return has_scanned_; }

drivers::PointCloudXYZIRADTPtr PandarQTDecoder::get_pointcloud() { return scan_pc_; }

void PandarQTDecoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    return;
  }

  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new PointCloudXYZIRADT);
    has_scanned_ = false;
  }

  bool dual_return = (packet_.return_mode == DUAL_RETURN);
  auto step = dual_return ? 2 : 1;

  if (!dual_return) {
    if (
      (packet_.return_mode == FIRST_RETURN && sensor_configuration_->return_mode != drivers::ReturnMode::FIRST) ||
      (packet_.return_mode == LAST_RETURN && sensor_configuration_->return_mode != drivers::ReturnMode::LAST)) {
      //sensor config, driver mismatched
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

drivers::PointXYZIRADT PandarQTDecoder::build_point(
  size_t block_id, size_t unit_id, drivers::ReturnMode return_type)
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
  point.distance = unit.distance;
  point.ring = unit_id;
  point.azimuth = block.azimuth + std::round(azimuth_offset_[unit_id] * 100.0f);
  point.return_type = drivers::ReturnModeToInt(return_type);;
  point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;
  point.time_stamp +=
    dual_return
      ? (static_cast<double>(block_offset_dual_[block_id] + firing_offset_[unit_id]) /
      1000000.0f) : (static_cast<double>(block_offset_single_[block_id] +
      firing_offset_[unit_id]) /
         1000000.0f);

  return point;
}

drivers::PointCloudXYZIRADTPtr PandarQTDecoder::convert(size_t block_id)
{
  PointCloudXYZIRADTPtr block_pc(new PointCloudXYZIRADT);

  const auto & block = packet_.blocks[block_id];
  for (size_t unit_id = 0; unit_id < LASER_COUNT; ++unit_id) {
    const auto & unit = block.units[unit_id];
    // skip invalid points
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }

    block_pc->points.emplace_back(build_point(
      block_id, unit_id,
      (packet_.return_mode == FIRST_RETURN) ? drivers::ReturnMode::SINGLE_FIRST
                                                : drivers::ReturnMode::SINGLE_LAST));
  }
 return block_pc;
}

drivers::PointCloudXYZIRADTPtr PandarQTDecoder::convert_dual(size_t block_id)
{
  //   Under the Dual Return mode, the ranging data from each firing is stored in two adjacent
  //   blocks:
  // · The even number block is the first return
  // · The odd number block is the last return
  // · The Azimuth changes every two blocks
  // · Important note: Hesai datasheet block numbering starts from 0, not 1, so odd/even are
  // reversed here
  PointCloudXYZIRADTPtr block_pc(new PointCloudXYZIRADT);

  size_t even_block_id = block_id;
  size_t odd_block_id = block_id + 1;
  const auto & even_block = packet_.blocks[even_block_id];
  const auto & odd_block = packet_.blocks[odd_block_id];

  for (size_t unit_id = 0; unit_id < LASER_COUNT; ++unit_id) {
    const auto & even_unit = even_block.units[unit_id];
    const auto & odd_unit = odd_block.units[unit_id];

    bool even_usable = !(even_unit.distance <= 0.1 || even_unit.distance > 200.0);
    bool odd_usable = !(odd_unit.distance <= 0.1 || odd_unit.distance > 200.0);

//    if (sensor_configuration_->return_mode == drivers::ReturnMode::SINGLE_FIRST && even_usable) {
    if (sensor_configuration_->return_mode == drivers::ReturnMode::FIRST && even_usable) {
      // First return is in even block
      block_pc->push_back(build_point(even_block_id, unit_id, drivers::ReturnMode::SINGLE_FIRST));
//    } else if (sensor_configuration_->return_mode == drivers::ReturnMode::SINGLE_LAST && even_usable) {
    } else if (sensor_configuration_->return_mode == drivers::ReturnMode::LAST && even_usable) {
      // Last return is in odd block
      block_pc->push_back(build_point(odd_block_id, unit_id, drivers::ReturnMode::SINGLE_LAST));
//    } else if (sensor_configuration_->return_mode == drivers::ReturnMode::DUAL_ONLY) {
    } else if (sensor_configuration_->return_mode == drivers::ReturnMode::DUAL) {
      // If the two returns are too close, only return the last one
      if (
        (abs(even_unit.distance - odd_unit.distance) < dual_return_distance_threshold_) &&
        odd_usable) {
        block_pc->push_back(build_point(odd_block_id, unit_id, drivers::ReturnMode::DUAL_ONLY));
      } else {
        if (even_usable) {
          block_pc->push_back(build_point(even_block_id, unit_id, drivers::ReturnMode::DUAL_FIRST));
        }
        if (odd_usable) {
          block_pc->push_back(build_point(odd_block_id, unit_id, drivers::ReturnMode::DUAL_LAST));
        }
      }
    }
  }
  return block_pc;
}

bool PandarQTDecoder::parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (pandar_packet.size != PACKET_SIZE && pandar_packet.size != PACKET_WITHOUT_UDPSEQ_SIZE) {
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

  for (size_t block = 0; block < static_cast<size_t>(packet_.header.chBlockNumber); block++) {
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
  index += ENGINE_VELOCITY;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
  index += TIMESTAMP_SIZE;

  packet_.return_mode = buf[index] & 0xff;

  index += RETURN_SIZE;
  index += FACTORY_SIZE;

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

  return true;
}
}  // namespace pandar_qt
}  // namespace drivers
}  // namespace nebula