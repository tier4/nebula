#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_decoder.hpp"

namespace nebula
{
namespace drivers
{

template <typename PacketT>
bool HesaiDecoder<PacketT>::parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (pandar_packet.size != sizeof(PacketT)) {
    std::cerr << "Packet size mismatch:" << pandar_packet.size << " | Expected:" << sizeof(PacketT)
              << std::endl;
    return false;
  }
  if (std::memcpy(&packet_, pandar_packet.data.data(), sizeof(PacketT))) {
    // FIXME(mojomex) do validation?
    return true;
  }

  return false;
}

template <typename PacketT>
void HesaiDecoder<PacketT>::convertReturns(size_t start_block_id, size_t n_blocks)
{
  uint64_t timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
  for (size_t block_id = start_block_id; block_id < start_block_id + n_blocks; ++block_id) {
    for (size_t channel_id = 0; channel_id < PacketT::N_CHANNELS; ++channel_id) {
      auto & block = packet_.body.blocks[block_id];
      auto & unit = block.units[channel_id];

      auto distance = getDistance(unit);
      if (
        distance < sensor_configuration_->min_range ||
        distance > sensor_configuration_->max_range) {
        continue;
      }

      NebulaPoint point;
      point.distance = distance;
      point.intensity = unit.reflectivity;
      point.time_stamp = getPointTimeRelative(timestamp_ns, block_id, channel_id);
      point.return_type = static_cast<uint8_t>(getReturnType(block_id));
      point.channel = channel_id;

      uint32_t raw_azimuth = block.get_azimuth();
      auto [azimuth_idx, elevation_idx, azimuth_rad, elevation_rad] =
        angle_corrector_.getCorrectedAzimuthAndElevation(raw_azimuth, channel_id);
      float xyDistance = point.distance * angle_corrector_.cos_map_[elevation_idx];
      point.x = xyDistance * angle_corrector_.sin_map_[azimuth_idx];
      point.y = xyDistance * angle_corrector_.cos_map_[azimuth_idx];
      point.z = unit.distance * angle_corrector_.sin_map_[elevation_idx];
      point.azimuth = azimuth_rad;
      point.elevation = elevation_rad;

      decode_pc_->emplace_back(point);
    }
  }
}

template <typename PacketT>
bool HesaiDecoder<PacketT>::checkScanCompleted(int current_phase)
{
  return current_phase > last_phase_ && !has_scanned_;
}

template <typename PacketT>
float HesaiDecoder<PacketT>::getDistance(typename PacketT::body_t::block_t::unit_t & unit)
{
  return unit.distance * hesai_packet::get_dis_unit(packet_);
}

template <typename PacketT>
uint32_t HesaiDecoder<PacketT>::getPointTimeRelative(
  uint64_t packet_timestamp_ns, size_t block_id, size_t channel_id)
{
  auto point_to_packet_offset_ns = getChannelTimeOffset(channel_id) + getBlockTimeOffset(block_id);
  auto packet_to_scan_offset_ns = static_cast<uint32_t>(packet_timestamp_ns - scan_timestamp_ns_);
  return packet_to_scan_offset_ns + point_to_packet_offset_ns;
}

template <typename PacketT>
uint32_t HesaiDecoder<PacketT>::getChannelTimeOffset(size_t channel_id)
{
  return channel_firing_offset_ns_[channel_id];
}

template <typename PacketT>
uint32_t HesaiDecoder<PacketT>::getBlockTimeOffset(size_t block_id)
{
  int n_returns = hesai_packet::get_n_returns(packet_.tail.return_mode);
  return block_firing_offset_ns_[n_returns][block_id];
}

template <typename PacketT>
ReturnType HesaiDecoder<PacketT>::getReturnType(size_t block_id)
{
  // FIXME(mojomex) deal with coinciding returns in dual/triple return mode
  // FIXME(mojomex) deal with reversed order of DUAL_FIRST_LAST in some sensors
  switch (packet_.tail.return_mode) {
    case hesai_packet::return_mode::SINGLE_FIRST:
      return ReturnType::FIRST;
    case hesai_packet::return_mode::SINGLE_SECOND:
      return ReturnType::SECOND;
    case hesai_packet::return_mode::SINGLE_STRONGEST:
      return ReturnType::STRONGEST;
    case hesai_packet::return_mode::SINGLE_LAST:
      return ReturnType::LAST;
    case hesai_packet::return_mode::DUAL_LAST_STRONGEST:
      return block_id % 2 == 0 ? ReturnType::LAST : ReturnType::STRONGEST;
    case hesai_packet::return_mode::DUAL_FIRST_SECOND:
      return block_id % 2 == 0 ? ReturnType::FIRST : ReturnType::SECOND;
    case hesai_packet::return_mode::DUAL_FIRST_LAST:
      return block_id % 2 == 0 ? ReturnType::FIRST : ReturnType::LAST;
    case hesai_packet::return_mode::DUAL_FIRST_STRONGEST:
      return block_id % 2 == 0 ? ReturnType::FIRST : ReturnType::STRONGEST;
    case hesai_packet::return_mode::TRIPLE_FIRST_LAST_STRONGEST:
      switch (block_id % 3) {
        case 0:
          return ReturnType::FIRST;
        case 1:
          return ReturnType::LAST;
        case 2:
          return ReturnType::STRONGEST;
        default:
          return ReturnType::UNKNOWN;
      }
    case hesai_packet::return_mode::DUAL_STRONGEST_SECONDSTRONGEST:
      return block_id % 2 == 0 ? ReturnType::STRONGEST : ReturnType::SECOND_STRONGEST;
    default:
      return ReturnType::UNKNOWN;
  }
}

template <typename PacketT>
HesaiDecoder<PacketT>::HesaiDecoder(
  const std::shared_ptr<HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<HesaiCalibrationConfiguration> & calibration_configuration,
  const std::shared_ptr<HesaiCorrection> & correction_configuration)
: sensor_configuration_(sensor_configuration),
  angle_corrector_(calibration_configuration, correction_configuration)
{
  decode_pc_.reset(new NebulaPointCloud);
  output_pc_.reset(new NebulaPointCloud);
}

template <typename PacketT>
int HesaiDecoder<PacketT>::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    return -1;
  }

  // At the start of a scan, set the timestamp to its absolute time since epoch.
  // Point timestamps in the scan are relative to this value.
  if (has_scanned_ || scan_timestamp_ns_ == 0) {
    scan_timestamp_ns_ = hesai_packet::get_timestamp_ns(packet_);
  }

  if (has_scanned_) {
    has_scanned_ = false;
  }

  const size_t n_returns = hesai_packet::get_n_returns(packet_.tail.return_mode);
  int current_azimuth;
  for (size_t block_id = 0; block_id < PacketT::N_BLOCKS; block_id += n_returns) {
    // FIXME(mojomex) respect scan phase
    current_azimuth =
      packet_.body.blocks[block_id].get_azimuth() -
      static_cast<int>(sensor_configuration_->scan_phase * PacketT::DEGREE_SUBDIVISIONS);
    if (checkScanCompleted(current_azimuth)) {
      std::swap(decode_pc_, output_pc_);
      decode_pc_->points.clear();
      has_scanned_ = true;
    }

    convertReturns(block_id, n_returns);
  }

  last_phase_ = current_azimuth;
  return last_phase_;
}

template <typename PacketT>
bool HesaiDecoder<PacketT>::hasScanned()
{
  return has_scanned_;
}

template <typename PacketT>
std::tuple<drivers::NebulaPointCloudPtr, double> HesaiDecoder<PacketT>::getPointcloud()
{
  return std::make_pair(output_pc_, scan_timestamp_ns_);
}

// Explicit template instantiation to prevent linker errors
template class HesaiDecoder<hesai_packet::Packet128E3X>;
template class HesaiDecoder<hesai_packet::PacketXT32>;
template class HesaiDecoder<hesai_packet::PacketXT32M2X>;
template class HesaiDecoder<hesai_packet::PacketQT128C2X>;
template class HesaiDecoder<hesai_packet::PacketAT128E2X>;
template class HesaiDecoder<hesai_packet::PacketQT64>;
template class HesaiDecoder<hesai_packet::Packet64>;
template class HesaiDecoder<hesai_packet::Packet40P>;

}  // namespace drivers
}  // namespace nebula
