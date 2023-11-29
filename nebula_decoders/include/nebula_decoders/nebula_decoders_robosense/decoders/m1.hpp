#pragma once

#include "nebula_decoders/nebula_decoders_common/sensor.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/angles.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/channel.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/distance.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/intensity.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/return_mode.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/timestamp.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/validity.hpp"
#include "nebula_decoders/nebula_decoders_common/util.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"

#include "boost/endian/buffers.hpp"

#include <cstddef>
#include <cstdint>

using namespace boost::endian;
using namespace nebula::drivers::point_accessors;

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
  big_uint16_buf_t radius;
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
  big_uint48_buf_t tail;
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

  uint8_t wave_mode;

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

class M1 : public SensorBase<M1>,
           public PointTimestampMixin<M1>,
           public ReturnModeMixin<M1>,
           public AnglesInUnitMixin<M1>,
           public BasicReflectivityMixin<M1>,
           public DistanceMixin<M1>,
           public ChannelIsUnitMixin<M1>,
           public NonZeroDistanceIsValidMixin<M1>
{
public:
  typedef robosense_packet::m1::Packet packet_t;

  static constexpr float MIN_RANGE = 0.2f;
  static constexpr float MAX_RANGE = 150.f;
  static constexpr size_t MAX_SCAN_BUFFER_POINTS = 1152000;

  static constexpr std::array<bool, 3> RETURN_GROUP_STRIDE{1, 0, 0};

  int32_t getPacketRelativeTimestamp(
    const packet_t & packet, const size_t block_id, const size_t channel_id,
    const ReturnMode return_mode) override
  {
    const auto * block = getBlock(packet, block_id);
    return static_cast<int32_t>(getFieldValue(block->time_offset)) * 1000;
  };

  double getDistanceUnit(const packet_t & packet) override
  {
    return 0.005;
  }

  /// @brief Get the distance value of the given unit in meters.
  double getDistance(const packet_t & packet, const size_t block_id, const size_t unit_id) override
  {
    const auto * unit = getUnit(packet, block_id, unit_id);
    return getFieldValue(unit->radius) * getDistanceUnit(packet);
  }

  ReturnMode getReturnMode(const packet_t & packet, const SensorConfigurationBase & /* config */) override
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

  std::map<std::string, std::string> getSensorInfo(
    const robosense_packet::m1::InfoPacket & info_packet)
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

    switch (info_packet.wave_mode) {
      case 0:
        sensor_info["return_mode"] = "dual";
        break;
      case 4:
        sensor_info["return_mode"] = "strongest";
        break;
      case 5:
        sensor_info["return_mode"] = "last";
        break;
      case 6:
        sensor_info["return_mode"] = "first";
        break;
    }

    switch (info_packet.time_sync_mode) {
      case 0:
        sensor_info["time_sync_mode"] = "gps";
        break;
      case 1:
        sensor_info["time_sync_mode"] = "e2e";
        break;
      case 2:
        sensor_info["time_sync_mode"] = "p2p";
        break;
      case 3:
        sensor_info["time_sync_mode"] = "gptp";
        break;
    }

    switch (info_packet.sync_status) {
      case 0:
        sensor_info["sync_status"] = "time_sync_invalid";
        break;
      case 1:
        sensor_info["sync_status"] = "gps_time_sync_successful";
        break;
      case 2:
        sensor_info["sync_status"] = "ptp_time_sync_successful";
        break;
    }

    sensor_info["time"] = std::to_string(info_packet.time.get_time_in_ns());

    return sensor_info;
  }
};

}  // namespace drivers
}  // namespace nebula