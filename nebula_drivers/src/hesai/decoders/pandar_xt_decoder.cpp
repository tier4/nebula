#include "hesai/decoders/pandar_xt_decoder.hpp"
#include "hesai/decoders/pandar_xt.hpp"

namespace
{
static inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }
}  // namespace

namespace nebula
{
namespace drivers
{
namespace pandar_xt
{
PandarXTDecoder::PandarXTDecoder()
{
//  for (size_t unit = 0; unit < UNIT_NUM; ++unit) {
//    firing_offset_[unit] = 1.512 * unit + 0.28;
//  }
//
//  for (size_t block = 0; block < BLOCK_NUM; ++block) {
//    block_offset_single_[block] = 3.28f - 50.00f * (BLOCK_NUM - block - 1);
//    block_offset_dual_[block] = 3.28f - 50.00f * ((BLOCK_NUM - block - 1) / 2);
//  }
//
//  // TODO: add calibration data validation
//  // if(calibration.elev_angle_map.size() != num_lasers_){
//  //   // calibration data is not valid!
//  // }
//  for (size_t laser = 0; laser < UNIT_NUM; ++laser) {
//    elev_angle_[laser] = calibration.elev_angle_map[laser];
//    azimuth_offset_[laser] = calibration.azimuth_offset_map[laser];
//  }
//
//  scan_phase_ = static_cast<uint16_t>(scan_phase * 100.0f);
//  return_mode_ = return_mode;
//
//  last_phase_ = 0;
//  has_scanned_ = false;
//
//  scan_pc_.reset(new pcl::PointCloud<PclPointCloudXYZIRADTPtr>);
//  overflow_pc_.reset(new pcl::PointCloud<PclPointCloudXYZIRADTPtr>);
}

bool PandarXTDecoder::hasScanned() { return has_scanned_; }

nebula::drivers::PclPointCloudXYZIRADTPtr PandarXTDecoder::getPointcloud() { return scan_pc_; }

void PandarXTDecoder::unpack(const pandar_msgs::msg::PandarScan & pandar_scan)
{
//  if (!parsePacket(raw_packet)) {
//    return;
//  }
//
//  if (has_scanned_) {
//    scan_pc_ = overflow_pc_;
//    overflow_pc_.reset(new pcl::PointCloud<PclPointCloudXYZIRADTPtr>);
//    has_scanned_ = false;
//  }
//
//  bool dual_return =
//    (packet_.return_mode != FIRST_RETURN && packet_.return_mode != STRONGEST_RETURN &&
//     packet_.return_mode != LAST_RETURN);
//  auto step = dual_return ? 2 : 1;
//
//  for (size_t block_id = 0; block_id < BLOCK_NUM; block_id += step) {
//    auto block_pc = dual_return ? convert_dual(block_id) : convert(block_id);
//    int current_phase =
//      (static_cast<int>(packet_.blocks[block_id].azimuth) - scan_phase_ + 36000) % 36000;
//    if (current_phase > last_phase_ && !has_scanned_) {
//      *scan_pc_ += *block_pc;
//    } else {
//      *overflow_pc_ += *block_pc;
//      has_scanned_ = true;
//    }
//    last_phase_ = current_phase;
//  }
//  return;
}

nebula::drivers::PclPointCloudXYZIRADTPtr PandarXTDecoder::convert(const int block_id)
{
  nebula::drivers::PclPointCloudXYZIRADTPtr block_pc(new pcl::PointCloud<PclPointCloudXYZIRADTPtr>);

  // double unix_second = raw_packet.header.stamp.toSec() // system-time (packet receive time)
//  double unix_second = static_cast<double>(timegm(&packet_.t));  // sensor-time (ppt/gps)
//
//  const auto & block = packet_.blocks[block_id];
//  for (size_t unit_id = 0; unit_id < UNIT_NUM; ++unit_id) {
//    PclPointCloudXYZIRADTPtr point;
//    const auto & unit = block.units[unit_id];
//    // skip invalid points
//    if (unit.distance <= 0.1 || unit.distance > 200.0) {
//      continue;
//    }
//    double xyDistance = unit.distance * cosf(deg2rad(elev_angle_[unit_id]));
//
//    point.x = static_cast<float>(
//      xyDistance *
//      sinf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
//    point.y = static_cast<float>(
//      xyDistance *
//      cosf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
//    point.z = static_cast<float>(unit.distance * sinf(deg2rad(elev_angle_[unit_id])));
//
//    point.intensity = unit.intensity;
//    point.distance = unit.distance;
//    point.ring = unit_id;
//    point.azimuth = block.azimuth + round(azimuth_offset_[unit_id] * 100.0f);
//
//    point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;
//
//    point.time_stamp +=
//      (static_cast<double>(block_offset_single_[block_id] + firing_offset_[unit_id]) / 1000000.0f);
//
//    block_pc->push_back(point);
//  }
  return block_pc;
}

nebula::drivers::PclPointCloudXYZIRADTPtr PandarXTDecoder::convert_dual(const int block_id)
{
  nebula::drivers::PclPointCloudXYZIRADTPtr block_pc(new pcl::PointCloud<PclPointCloudXYZIRADTPtr>);

  // double unix_second = raw_packet.header.stamp.toSec() // system-time (packet receive time)
//  double unix_second = static_cast<double>(timegm(&packet_.t));  // sensor-time (ppt/gps)
//
//  auto head = block_id + ((return_mode_ == ReturnMode::FIRST) ? 1 : 0);
//  auto tail = block_id + ((return_mode_ == ReturnMode::LAST) ? 1 : 2);
//
//  for (size_t unit_id = 0; unit_id < UNIT_NUM; ++unit_id) {
//    for (int i = head; i < tail; ++i) {
//      PclPointCloudXYZIRADTPtr point;
//      const auto & block = packet_.blocks[i];
//      const auto & unit = block.units[unit_id];
//      // skip invalid points
//      if (unit.distance <= 0.1 || unit.distance > 200.0) {
//        continue;
//      }
//      double xyDistance = unit.distance * cosf(deg2rad(elev_angle_[unit_id]));
//
//      point.x = static_cast<float>(
//        xyDistance *
//        sinf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
//      point.y = static_cast<float>(
//        xyDistance *
//        cosf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
//      point.z = static_cast<float>(unit.distance * sinf(deg2rad(elev_angle_[unit_id])));
//
//      point.intensity = unit.intensity;
//      point.distance = unit.distance;
//      point.ring = unit_id;
//      point.azimuth = block.azimuth + round(azimuth_offset_[unit_id] * 100.0f);
//
//      point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;
//
//      point.time_stamp +=
//        (static_cast<double>(block_offset_dual_[block_id] + firing_offset_[unit_id]) / 1000000.0f);
//
//      block_pc->push_back(point);
//    }
//  }
  return block_pc;
}

bool PandarXTDecoder::parsePacket(const pandar_msgs::msg::PandarPacket & raw_packet)
{
//  if (raw_packet.size != PACKET_SIZE) {
//    return false;
//  }
//  const uint8_t * buf = &raw_packet.data[0];
//
//  size_t index = 0;
//  // Parse 12 Bytes Header
//  packet_.header.sob = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
//  packet_.header.chProtocolMajor = buf[index + 2] & 0xff;
//  packet_.header.chProtocolMinor = buf[index + 3] & 0xff;
//  packet_.header.chLaserNumber = buf[index + 6] & 0xff;
//  packet_.header.chBlockNumber = buf[index + 7] & 0xff;
//  packet_.header.chReturnType = buf[index + 8] & 0xff;
//  packet_.header.chDisUnit = buf[index + 9] & 0xff;
//  index += HEAD_SIZE;
//
//  if (packet_.header.sob != 0xEEFF) {
//    // Error Start of Packet!
//    return false;
//  }
//
//  for (int8_t block = 0; block < packet_.header.chBlockNumber; block++) {
//    packet_.blocks[block].azimuth = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
//    index += BLOCK_HEADER_AZIMUTH;
//
//    for (int unit = 0; unit < packet_.header.chLaserNumber; unit++) {
//      unsigned int unRange = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
//
//      packet_.blocks[block].units[unit].distance =
//        (static_cast<double>(unRange * packet_.header.chDisUnit)) / (double)1000;
//      packet_.blocks[block].units[unit].intensity = (buf[index + 2] & 0xff);
//      packet_.blocks[block].units[unit].confidence = (buf[index + 3] & 0xff);
//      index += UNIT_SIZE;
//    }
//  }
//
//  index += RESERVED_SIZE;  // skip reserved bytes
//  packet_.return_mode = buf[index] & 0xff;
//
//  index += RETURN_SIZE;
//  index += ENGINE_VELOCITY;
//
//  packet_.t.tm_year = (buf[index + 0] & 0xff) + 100;
//  packet_.t.tm_mon = (buf[index + 1] & 0xff) - 1;
//  packet_.t.tm_mday = buf[index + 2] & 0xff;
//  packet_.t.tm_hour = buf[index + 3] & 0xff;
//  packet_.t.tm_min = buf[index + 4] & 0xff;
//  packet_.t.tm_sec = buf[index + 5] & 0xff;
//  packet_.t.tm_isdst = 0;
//  // in case of time error
//  if (packet_.t.tm_year >= 200) {
//    packet_.t.tm_year -= 100;
//  }
//
//  index += UTC_SIZE;
//
//  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
//                 ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
//  index += TIMESTAMP_SIZE;
//  index += FACTORY_SIZE;

  return true;
}
}  // namespace pandar_xt
}  // namespace drivers
}  // namespace nebula