#pragma once

#include "nebula_decoders/nebula_decoders_common/sensor.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/angles.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/channel.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/distance.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/intensity.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/return_mode.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/scan_completion.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/timestamp.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/validity.hpp"
#include "nebula_decoders/nebula_decoders_common/util.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/sensor_info.hpp"

#include "boost/endian/buffers.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

using namespace boost::endian;
using namespace nebula::drivers::sensor_mixins;

namespace nebula
{
namespace drivers
{
namespace robosense_packet
{
namespace m1
{
#pragma pack(push, 1)

struct Header
{
  big_uint32_buf_t header_id;
  big_uint16_buf_t sequence_number;
  big_uint16_buf_t protocol_version;
  uint8_t wave_mode;
  uint8_t time_sync_mode;
  Timestamp timestamp;
  uint8_t reserved_1[10];
  uint8_t lidar_type;
  uint8_t mems_tmp;
};

template <typename UnitT, size_t UnitN>
struct M1Block
{
  uint8_t time_offset;
  uint8_t return_seq;
  UnitT units[UnitN];
  typedef UnitT unit_t;
};

struct M1Unit
{
  big_uint16_buf_t distance;
  big_uint16_buf_t elevation;
  big_uint16_buf_t azimuth;
  big_uint8_buf_t reflectivity;
  big_uint16_buf_t reserved;
};

struct Packet : public PacketBase<25, 5, 2, 100>
{
  typedef Body<M1Block<M1Unit, Packet::N_CHANNELS>, Packet::N_BLOCKS> body_t;
  Header header;
  body_t body;
  big_uint24_buf_t tail;
};

struct InfoPacket
{
  uint8_t header[8];
  uint8_t reserved_1;
  uint8_t frame_rate;

  IpAddress lidar_ip;
  IpAddress dest_pc_ip;
  MacAddress mac_addr;
  big_uint16_buf_t pc_dest_msop_port;
  big_uint16_buf_t pc_dest_difop_port;

  big_uint16_buf_t h_fov_start;
  big_uint16_buf_t h_fov_end;
  big_uint16_buf_t v_fov_start;
  big_uint16_buf_t v_fov_end;

  FirmwareVersion mb_programmable_logic_fw_version;
  FirmwareVersion mb_programming_system_fw_version;

  SerialNumber serial_number;

  uint8_t return_mode;

  uint8_t time_sync_mode;
  uint8_t sync_status;
  Timestamp time;

  uint8_t operating_status[31];
  uint8_t diagnostics_reserved[29];

  uint8_t reserved_2[60];
  uint8_t reserved_3[71];
};

#pragma pack(pop)
}  // namespace m1
}  // namespace robosense_packet

using namespace sensor_mixins;

class M1 : public SensorBase<robosense_packet::m1::Packet>,
           public PointTimestampMixin<robosense_packet::m1::Packet>,
           public robosense_packet::RobosensePacketTimestampMixin<robosense_packet::m1::Packet>,
           public ReturnModeMixin<robosense_packet::m1::Packet>,
           public AnglesInUnitMixin<robosense_packet::m1::Packet>,
           public BasicReflectivityMixin<robosense_packet::m1::Packet>,
           public DistanceMixin<robosense_packet::m1::Packet>,
           public ChannelIsUnitMixin<robosense_packet::m1::Packet>,
           public NonZeroDistanceIsValidMixin<robosense_packet::m1::Packet>,
           public ScanCompletionMixin<robosense_packet::m1::Packet>,
           public AngleCorrectorMixin<robosense_packet::m1::Packet>
{
private:
  uint32_t last_sequence_number_{};

  std::array<float, 65536> sin_{};
  std::array<float, 65536> cos_{};

  float internalAngleToRad(int32_t angle) const
  {
    return static_cast<float>(angle - 32768) / 100.0f * M_PI / 180.0f;
  }

public:
  static constexpr float MIN_RANGE = 0.2f;
  static constexpr float MAX_RANGE = 150.f;
  static constexpr size_t MAX_SCAN_BUFFER_POINTS = 1152000;

  static constexpr std::array<bool, 3> RETURN_GROUP_STRIDE{1, 0, 0};

  M1()
  {
    for (size_t i = 0; i < 65536; ++i) {
      sin_[i] = std::sin(internalAngleToRad(i));
      cos_[i] = std::cos(internalAngleToRad(i));
    }
  }

  size_t getDecodeGroupIndex(const uint8_t * const raw_packet) const override
  {
    constexpr size_t header_offset = offsetof(robosense_packet::m1::Packet, header);
    constexpr size_t wave_mode_offset = offsetof(robosense_packet::m1::Header, wave_mode);
    constexpr size_t seq_num_offset = offsetof(robosense_packet::m1::Header, sequence_number);
    // In single return mode, the return index is always 0
    if (raw_packet[header_offset + wave_mode_offset] != 0x00) {
      return 0;
    }

    // From the datasheet: in dual return mode, the odd-numbered packets store the first return
    // [...] the even numbered packets store the second return
    bool is_odd = raw_packet[header_offset + seq_num_offset] % 2 == 1;
    return is_odd ? 0 : 1;
  }

  bool checkScanCompleted(const packet_t & packet, const size_t /* block_id */) override
  {
    const uint32_t current_sequence_number = getFieldValue(packet.header.sequence_number);
    bool completed = current_sequence_number < last_sequence_number_;
    last_sequence_number_ = current_sequence_number;
    return completed;
  }

  int32_t getPacketRelativeTimestamp(
    const packet_t & packet, const size_t block_id, const size_t /* channel_id */,
    const ReturnMode /* return_mode */) const override
  {
    const auto * block = getBlock(packet, block_id);
    return static_cast<int32_t>(getFieldValue(block->time_offset)) * 1000;
  };

  int32_t getEarliestPointTimeOffsetForScan(
    const packet_t & packet, const size_t block_id, const ReturnMode return_mode) const override
  {
    int32_t t_min = std::numeric_limits<int>::max();
    for (size_t i = block_id; i < packet_t::N_BLOCKS; ++i) {
      t_min = std::min(t_min, getPacketRelativeTimestamp(packet, i, 0, return_mode));
    }

    return t_min;
  }

  double getDistanceUnit(const packet_t & /* packet */) const override { return 0.005; }

  /// @brief Get the distance value of the given unit in meters.
  double getDistance(
    const packet_t & packet, const size_t block_id, const size_t unit_id) const override
  {
    const auto * unit = getUnit(packet, block_id, unit_id);
    return getFieldValue(unit->distance) * getDistanceUnit(packet);
  }

  sensor_mixins::CorrectedAngleData getCorrectedAngleData(
    int32_t raw_azimuth, int32_t raw_elevation) const override
  {
    sensor_mixins::CorrectedAngleData data;
    data.azimuth_rad = internalAngleToRad(raw_azimuth);
    data.elevation_rad = internalAngleToRad(raw_elevation);
    data.sin_azimuth = sin_[raw_azimuth];
    data.cos_azimuth = cos_[raw_azimuth];
    data.sin_elevation = sin_[raw_elevation];
    data.cos_elevation = cos_[raw_elevation];
    return data;
  }

  ReturnMode getReturnMode(
    const packet_t & packet, const SensorConfigurationBase & /* config */) const override
  {
    switch (getFieldValue(packet.header.wave_mode)) {
      case 0x00:
        return ReturnMode::DUAL;
      case 0x04:
        return ReturnMode::SINGLE_STRONGEST;
      case 0x05:
        return ReturnMode::SINGLE_LAST;
      case 0x06:
        return ReturnMode::SINGLE_FIRST;
      default:
        return ReturnMode::UNKNOWN;
    }
  }
};

class M1Info : public SensorInfoBase<robosense_packet::m1::InfoPacket>
{
private:
  static constexpr uint8_t SYNC_MODE_GPS_FLAG = 0x00;
  static constexpr uint8_t SYNC_MODE_E2E_FLAG = 0x01;
  static constexpr uint8_t SYNC_MODE_P2P_FLAG = 0x02;
  static constexpr uint8_t SYNC_MODE_GPTP_FLAG = 0x03;

  static constexpr uint8_t SYNC_STATUS_UNSUCCESSFUL_FLAG = 0x00;
  static constexpr uint8_t SYNC_STATUS_SUCCESSFUL_FLAG = 0x01;
  static constexpr uint8_t SYNC_STATUS_CLOCK_DISCONNECTED_FLAG = 0x02;

public:
  typedef typename robosense_packet::m1::InfoPacket packet_t;

  ::std::map<std::string, std::string> getSensorInfo(
    const robosense_packet::m1::InfoPacket & info_packet) const override
  {
    std::map<std::string, std::string> sensor_info;

    sensor_info["lidar_ip"] = info_packet.lidar_ip.to_string();
    sensor_info["dest_pc_ip"] = info_packet.dest_pc_ip.to_string();
    sensor_info["mac_addr"] = info_packet.mac_addr.to_string();
    sensor_info["pc_dest_msop_port"] = std::to_string(info_packet.pc_dest_msop_port.value());
    sensor_info["pc_dest_difop_port"] = std::to_string(info_packet.pc_dest_difop_port.value());
    sensor_info["h_fov_start"] = robosense_packet::get_float_value(info_packet.h_fov_start.value());
    sensor_info["h_fov_end"] = robosense_packet::get_float_value(info_packet.h_fov_end.value());
    sensor_info["mb_programmable_logic_fw_version"] =
      info_packet.mb_programmable_logic_fw_version.to_string();
    sensor_info["mb_programming_system_fw_version"] =
      info_packet.mb_programming_system_fw_version.to_string();
    sensor_info["serial_number"] = info_packet.serial_number.to_string();

    switch (getReturnMode(info_packet)) {
      case ReturnMode::DUAL:
        sensor_info["return_mode"] = "dual";
        break;
      case ReturnMode::SINGLE_STRONGEST:
        sensor_info["return_mode"] = "strongest";
        break;
      case ReturnMode::SINGLE_LAST:
        sensor_info["return_mode"] = "last";
        break;
      case ReturnMode::SINGLE_FIRST:
        sensor_info["return_mode"] = "first";
        break;
      default:
        sensor_info["return_mode"] = "n/a";
        break;
    }

    switch (info_packet.time_sync_mode) {
      case SYNC_MODE_GPS_FLAG:
        sensor_info["time_sync_mode"] = "gps";
        break;
      case SYNC_MODE_E2E_FLAG:
        sensor_info["time_sync_mode"] = "e2e";
        break;
      case SYNC_MODE_P2P_FLAG:
        sensor_info["time_sync_mode"] = "p2p";
        break;
      case SYNC_MODE_GPTP_FLAG:
        sensor_info["time_sync_mode"] = "gptp";
        break;
      default:
        sensor_info["time_sync_mode"] = "n/a";
        break;
    }

    switch (info_packet.sync_status) {
      case SYNC_STATUS_UNSUCCESSFUL_FLAG:
        sensor_info["sync_status"] = "not_synced";
        break;
      case SYNC_STATUS_SUCCESSFUL_FLAG:
        sensor_info["sync_status"] = "synced";
        break;
      case SYNC_STATUS_CLOCK_DISCONNECTED_FLAG:
        sensor_info["sync_status"] = "clock_disconnected";
        break;
      default:
        sensor_info["sync_status"] = "n/a";
        break;
    }

    sensor_info["time"] = std::to_string(info_packet.time.get_time_in_ns());

    return sensor_info;
  }

  std::optional<RobosenseCalibrationConfiguration> getSensorCalibration(
    const robosense_packet::m1::InfoPacket & /* info_packet */) const override
  {
    return {};  // M1 has no calibration
  }

  bool getSyncStatus(const robosense_packet::m1::InfoPacket & /* info_packet */) const override
  {
    return false;  // TODO(mojomex)
  }
};

}  // namespace drivers
}  // namespace nebula