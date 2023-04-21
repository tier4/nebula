#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_128_e4x_decoder.hpp"

#include <cmath>
#include <utility>

#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_128_e4x.hpp"

namespace nebula
{
namespace drivers
{
namespace pandar_128_e4x
{
Pandar128E4XDecoder::Pandar128E4XDecoder(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  sensor_calibration_ = calibration_configuration;

  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
  }

  size_t i = 0;
  for (const auto & angle : elev_angle_) {
    auto rads = deg2rad(angle);
    elev_angle_rad_[i] = rads;
    cos_elev_angle_[i] = cosf(rads);
    sin_elev_angle_[i++] = sinf(rads);
  }

  scan_phase_ = static_cast<uint16_t>(sensor_configuration_->scan_phase * 100.0f);
  dual_return_distance_threshold_ = sensor_configuration_->dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;
  first_timestamp_ = 0;

  scan_pc_.reset(new NebulaPointCloud);
  scan_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
  overflow_pc_.reset(new NebulaPointCloud);
  overflow_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
}

bool Pandar128E4XDecoder::hasScanned() { return has_scanned_; }

std::tuple<drivers::NebulaPointCloudPtr, double> Pandar128E4XDecoder::get_pointcloud()
{
  return std::make_tuple(scan_pc_, first_timestamp_);
}

bool Pandar128E4XDecoder::parsePacket(const pandar_msgs::msg::PandarPacket & raw_packet)
{
  if (raw_packet.size != sizeof(Packet)) {
    std::cerr << "Packet size mismatch:" << raw_packet.size << "| Expected:" << sizeof(Packet)
              << std::endl;
    return false;
  }
  if (std::memcpy(&packet_, raw_packet.data.data(), sizeof(Packet))) {
    return true;
  }
  std::cerr << "Invalid SOF " << std::hex << packet_.header.SOP << " Packet" << std::endl;
  return false;
}

bool Pandar128E4XDecoder::is_dual_return()
{
  switch (packet_.tail.return_mode) {
    case DUAL_LAST_STRONGEST_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::LAST);
      second_return_type_ = static_cast<uint8_t>(ReturnType::STRONGEST);
      return true;
    case DUAL_LAST_FIRST_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::LAST);
      second_return_type_ = static_cast<uint8_t>(ReturnType::FIRST);
      return true;
    case DUAL_FIRST_STRONGEST_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::FIRST);
      second_return_type_ = static_cast<uint8_t>(ReturnType::STRONGEST);
      return true;
    case SINGLE_FIRST_RETURN:
      first_return_type_ = static_cast<uint8_t>(ReturnType::FIRST);
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

void Pandar128E4XDecoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    return;
  }
  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new NebulaPointCloud);
    overflow_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
    has_scanned_ = false;
    first_timestamp_ = current_unit_unix_second_;
  }

  bool dual_return = is_dual_return();

  NebulaPointCloudPtr block_pc = dual_return ? convert_dual(UNUSED_INT) : convert(UNUSED_INT);

  int current_phase = (static_cast<int>(packet_.body.azimuth_1) - scan_phase_ + 36000) % 36000;
  if (current_phase > last_phase_ && !has_scanned_) {
    *scan_pc_ += *block_pc;
  } else {
    *overflow_pc_ += *block_pc;
    has_scanned_ = true;
  }
  last_phase_ = current_phase;
}

drivers::NebulaPoint Pandar128E4XDecoder::build_point(
  const Block & block, const size_t & laser_id, const uint16_t & azimuth,
  const uint32_t & unix_second, float & out_distance)
{
  NebulaPoint point{};

  float xyDistance = static_cast<float>(block.distance) * DISTANCE_UNIT * cos_elev_angle_[laser_id];

  //TODO: Create HASH TABLE to accelerate deg2rad
  point.x =
    (xyDistance * sinf(deg2rad(azimuth_offset_[laser_id] + (static_cast<float>(azimuth)) / 100.0)));
  point.y =
    (xyDistance * cosf(deg2rad(azimuth_offset_[laser_id] + (static_cast<float>(azimuth)) / 100.0)));
  point.z = static_cast<float>(block.distance * DISTANCE_UNIT * sin_elev_angle_[laser_id]);

  point.intensity = block.reflectivity;
  point.channel = laser_id;
  point.azimuth = static_cast<float>(azimuth / 100.0f) + azimuth_offset_[laser_id];
  point.time_stamp = unix_second + packet_.tail.timestamp_us - first_timestamp_;
  out_distance = xyDistance;
  return point;
}

uint32_t Pandar128E4XDecoder::get_epoch_from_datetime(const DateTime & date_time)
{
  struct tm t = {};
  t.tm_year = date_time.year;
  t.tm_mon = date_time.month - 1;
  t.tm_mday = date_time.day;
  t.tm_hour = date_time.hour;
  t.tm_min = date_time.minute;
  t.tm_sec = date_time.second;
  t.tm_isdst = 0;
  return timegm(&t);
}

drivers::NebulaPointCloudPtr Pandar128E4XDecoder::convert([[maybe_unused]] size_t)
{
  drivers::NebulaPointCloudPtr block_pc(new NebulaPointCloud);
  block_pc->reserve(LASER_COUNT * NUM_BLOCKS);
  current_unit_unix_second_ = get_epoch_from_datetime(packet_.tail.date_time);

  for (size_t i = 0; i < LASER_COUNT; i++) {
    auto distance = packet_.body.block_01[i].distance * DISTANCE_UNIT;
    if (distance < MIN_RANGE || distance > MAX_RANGE) continue;
    distance = packet_.body.block_02[i].distance * DISTANCE_UNIT;
    if (distance < MIN_RANGE || distance > MAX_RANGE) continue;
    float pt_distance;
    auto block1_pt = build_point(
      packet_.body.block_01[i], i, packet_.body.azimuth_1, current_unit_unix_second_, pt_distance);
    block1_pt.return_type = first_return_type_;
    auto block2_pt = build_point(
      packet_.body.block_02[i], i, packet_.body.azimuth_2, current_unit_unix_second_, pt_distance);
    block2_pt.return_type = first_return_type_;
    block_pc->points.emplace_back(block1_pt);
    block_pc->points.emplace_back(block2_pt);
  }

  return block_pc;
}

drivers::NebulaPointCloudPtr Pandar128E4XDecoder::convert_dual([[maybe_unused]] size_t)
{
  drivers::NebulaPointCloudPtr block_pc(new NebulaPointCloud);
  block_pc->reserve(LASER_COUNT * 2);
  current_unit_unix_second_ = get_epoch_from_datetime(packet_.tail.date_time);

  for (size_t i = 0; i < LASER_COUNT; i++) {
    auto distance = packet_.body.block_01[i].distance * DISTANCE_UNIT;
    if (distance < MIN_RANGE || distance > MAX_RANGE) continue;
    float pt1_distance, pt2_distance;
    auto block1_pt = build_point(
      packet_.body.block_01[i], i, packet_.body.azimuth_1, current_unit_unix_second_, pt1_distance);
    auto block2_pt = build_point(
      packet_.body.block_02[i], i, packet_.body.azimuth_2, current_unit_unix_second_, pt2_distance);
    block1_pt.return_type = first_return_type_;
    block_pc->points.emplace_back(block1_pt);
    if (fabsf(pt1_distance - pt2_distance) > dual_return_distance_threshold_) {
      block2_pt.return_type = second_return_type_;
      block_pc->points.emplace_back(block2_pt);
    }
  }

  return block_pc;
}

}  // namespace pandar_128_e4x
}  // namespace drivers
}  // namespace nebula