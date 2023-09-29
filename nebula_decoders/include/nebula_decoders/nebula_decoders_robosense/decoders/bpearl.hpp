#pragma once

#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_sensor.hpp"

#include "boost/endian/buffers.hpp"

#include <cstddef>
#include <cstdint>

namespace nebula
{
namespace drivers
{
namespace robosense_packet
{
namespace bpearl
{
#pragma pack(push, 1)

struct Timestamp
{
  boost::endian::big_uint8_buf_t year;
  boost::endian::big_uint8_buf_t month;
  boost::endian::big_uint8_buf_t day;
  boost::endian::big_uint8_buf_t hour;
  boost::endian::big_uint8_buf_t minute;
  boost::endian::big_uint8_buf_t second;
  boost::endian::big_uint16_buf_t millisecond;
  boost::endian::big_uint16_buf_t nanosecond;

  uint64_t get_time_in_ns() const
  {
    // Constants for conversion
    constexpr uint64_t nanoseconds_per_second = 1'000'000'000;
    constexpr uint64_t nanoseconds_per_millisecond = 1'000'000;
    constexpr uint64_t nanoseconds_per_minute = 60 * nanoseconds_per_second;
    constexpr uint64_t nanoseconds_per_hour = 60 * nanoseconds_per_minute;
    constexpr uint64_t nanoseconds_per_day = 24 * nanoseconds_per_hour;
    constexpr uint64_t nanoseconds_per_month = 30 * nanoseconds_per_day;
    constexpr uint64_t nanoseconds_per_year = 12 * nanoseconds_per_month;

    // Calculate the nanoseconds from each component
    uint64_t year_ns = year.value() * nanoseconds_per_year;
    uint64_t month_ns = month.value() * nanoseconds_per_month;
    uint64_t day_ns = day.value() * nanoseconds_per_day;
    uint64_t hour_ns = hour.value() * nanoseconds_per_hour;
    uint64_t minute_ns = minute.value() * nanoseconds_per_minute;
    uint64_t second_ns = second.value() * nanoseconds_per_second;
    uint64_t millisecond_ns = millisecond.value() * nanoseconds_per_millisecond;
    uint64_t nanosecond_ns = nanosecond.value();

    // Calculate the total time in nanoseconds
    uint64_t total_ns = year_ns + month_ns + day_ns + hour_ns + minute_ns + second_ns +
                        millisecond_ns + nanosecond_ns;

    return total_ns;
  }
};

struct Header
{
  boost::endian::big_uint64_buf_t header_id;
  boost::endian::big_uint32_buf_t checksum;
  boost::endian::big_uint32_buf_t packet_count;
  boost::endian::big_uint32_buf_t reserved_first;
  Timestamp timestamp;
  boost::endian::big_uint8_buf_t lidar_model;
  uint8_t reserved_second[7];
  boost::endian::big_uint16_buf_t temperature;
  boost::endian::big_uint16_buf_t top_board_temperature;
};

struct Packet : public PacketBase<12, 32, 2, 100>
{
  typedef Body<Block<Unit, Packet::N_CHANNELS>, Packet::N_BLOCKS> body_t;
  Header header;
  body_t body;
  boost::endian::big_uint48_buf_t tail;
};

struct OperatingStatus
{
  boost::endian::big_uint48_buf_t reserved;
  boost::endian::big_uint16_buf_t v_dat_0v5;
  boost::endian::big_uint16_buf_t v_dat_12v;
  boost::endian::big_uint16_buf_t v_dat_5v;
  boost::endian::big_uint16_buf_t v_dat_1v25;
  boost::endian::big_uint16_buf_t v_dat_0v;
  boost::endian::big_uint16_buf_t v_dat_1v;
};

struct FaultDiagnosis
{
  uint8_t reserved_first[10];
  boost::endian::big_uint8_buf_t cksum_st;
  boost::endian::big_uint16_buf_t manc_err1;
  boost::endian::big_uint16_buf_t manc_err2;
  boost::endian::big_uint8_buf_t gps_st;
  boost::endian::big_uint16_buf_t temperature1;
  boost::endian::big_uint16_buf_t temperature2;
  boost::endian::big_uint16_buf_t temperature3;
  boost::endian::big_uint16_buf_t temperature4;
  boost::endian::big_uint16_buf_t temperature5;
  uint8_t reserved_second[5];
  boost::endian::big_uint8_buf_t r_rpm1;
  boost::endian::big_uint8_buf_t r_rpm2;
  uint8_t reserved_third[7];
};

struct InfoPacket : public InfoPacketBase
{
  boost::endian::big_uint64_buf_t header;
  boost::endian::big_uint16_buf_t motor_speed;
  Ethernet ethernet;
  FovSetting fov_setting;
  boost::endian::big_uint16_buf_t tcp_msop_port;
  boost::endian::big_uint16_buf_t phase_lock;
  boost::endian::big_uint40_buf_t top_firmware_version;
  boost::endian::big_uint40_buf_t bottom_firmware_version;
  boost::endian::big_uint40_buf_t bottom_software_version;
  boost::endian::big_uint40_buf_t motor_firmware_version;
  uint8_t reserved_first[230];
  boost::endian::big_uint16_buf_t reverse_zero_angle_offset;
  boost::endian::big_uint48_buf_t serial_number;
  boost::endian::big_uint16_buf_t zero_angle_offset;
  boost::endian::big_uint8_buf_t return_mode;
  boost::endian::big_uint8_buf_t time_sync_mode;
  boost::endian::big_uint8_buf_t sync_status;
  Timestamp time;
  OperatingStatus operating_status;
  uint8_t reserved_second[6];
  boost::endian::big_uint8_buf_t rotation_direction;
  boost::endian::big_uint32_buf_t elapsed_time_flag;
  FaultDiagnosis fault_diagnosis;
  boost::endian::big_uint8_buf_t gprmc[86];
  CorrectedVerticalAngle corrected_vertical;
  CorrectedHorizontalAngle corrected_horizontal;
  uint8_t reserved_fourth[586];
  boost::endian::big_uint16_buf_t tail;
};

#pragma pack(pop)
}  // namespace bpearl

/// @brief Get the distance unit of the given @ref Bpearl packet in meters.
/// @return 0.005m (0.5cm)
template <>
double get_dis_unit<bpearl::Packet>(const bpearl::Packet & /* packet */)
{
  return 0.005;
}

}  // namespace robosense_packet

class Bpearl
: public RobosenseSensor<robosense_packet::bpearl::Packet, robosense_packet::bpearl::InfoPacket>
{
private:
  static constexpr int firing_time_offset_ns_single_[12][32]{};

  static constexpr int firing_time_offset_ns_dual_[12][32]{};

public:
  static constexpr float MIN_RANGE = 0.2f;
  static constexpr float MAX_RANGE = 150.f;
  static constexpr size_t MAX_SCAN_BUFFER_POINTS = 230400;  ///// !! Calculate this

  int getPacketRelativePointTimeOffset(
    uint32_t block_id, uint32_t channel_id,
    const std::shared_ptr<RobosenseSensorConfiguration> & sensor_configuration) override
  {
    if (sensor_configuration->return_mode == ReturnMode::DUAL)
      return firing_time_offset_ns_dual_[block_id][channel_id];
    else
      return firing_time_offset_ns_single_[block_id][channel_id];
  }

  std::map<std::string, std::string> getSensorInfo(
    const robosense_packet::bpearl::InfoPacket & info_packet)
  {
    std::map<std::string, std::string> sensor_info;
    sensor_info["motor_speed"] = std::to_string(info_packet.motor_speed.value());
    sensor_info["lidar_ip"] = info_packet.ethernet.lidar_ip.to_string();
    sensor_info["dest_pc_ip"] = info_packet.ethernet.dest_pc_ip.to_string();
    sensor_info["mac_addr"] = std::to_string(info_packet.ethernet.mac_addr.value());
    sensor_info["lidar_out_msop_port"] =
      std::to_string(info_packet.ethernet.lidar_out_msop_port.value());
    sensor_info["lidar_out_difop_port"] =
      std::to_string(info_packet.ethernet.lidar_out_difop_port.value());
    sensor_info["fov_start"] = std::to_string(info_packet.fov_setting.fov_start.value());
    sensor_info["fov_end"] = std::to_string(info_packet.fov_setting.fov_end.value());
    sensor_info["tcp_msop_port"] = std::to_string(info_packet.tcp_msop_port.value());
    sensor_info["phase_lock"] = std::to_string(info_packet.phase_lock.value());
    sensor_info["top_firmware_version"] = std::to_string(info_packet.top_firmware_version.value());
    sensor_info["bottom_firmware_version"] =
      std::to_string(info_packet.bottom_firmware_version.value());
    sensor_info["bottom_software_version"] =
      std::to_string(info_packet.bottom_software_version.value());
    sensor_info["motor_firmware_version"] =
      std::to_string(info_packet.motor_firmware_version.value());
    sensor_info["reverse_zero_angle_offset"] =
      std::to_string(info_packet.reverse_zero_angle_offset.value());
    sensor_info["serial_number"] = std::to_string(info_packet.serial_number.value());
    sensor_info["zero_angle_offset"] = std::to_string(info_packet.zero_angle_offset.value());

    if (info_packet.return_mode.value() == 0x00) {
      sensor_info["return_mode"] = "dual";
    } else if (info_packet.return_mode.value() == 0x01) {
      sensor_info["return_mode"] = "strongest";
    } else if (info_packet.return_mode.value() == 0x02) {
      sensor_info["return_mode"] = "last";
    }

    sensor_info["time_sync_mode"] = std::to_string(info_packet.time_sync_mode.value());
    sensor_info["sync_status"] = std::to_string(info_packet.sync_status.value());
    sensor_info["time"] = std::to_string(info_packet.time.get_time_in_ns());
    sensor_info["v_dat_0v5"] = std::to_string(info_packet.operating_status.v_dat_0v5.value());
    sensor_info["v_dat_12v"] = std::to_string(info_packet.operating_status.v_dat_12v.value());
    sensor_info["v_dat_5v"] = std::to_string(info_packet.operating_status.v_dat_5v.value());
    sensor_info["v_dat_1v25"] = std::to_string(info_packet.operating_status.v_dat_1v25.value());
    sensor_info["v_dat_0v"] = std::to_string(info_packet.operating_status.v_dat_0v.value());
    sensor_info["v_dat_1v"] = std::to_string(info_packet.operating_status.v_dat_1v.value());
    sensor_info["rotation_direction"] = std::to_string(info_packet.rotation_direction.value());
    sensor_info["elapsed_time_flag"] = std::to_string(info_packet.elapsed_time_flag.value());
    sensor_info["cksum_st"] = std::to_string(info_packet.fault_diagnosis.cksum_st.value());
    sensor_info["manc_err1"] = std::to_string(info_packet.fault_diagnosis.manc_err1.value());
    sensor_info["manc_err2"] = std::to_string(info_packet.fault_diagnosis.manc_err2.value());
    sensor_info["gps_st"] = std::to_string(info_packet.fault_diagnosis.gps_st.value());
    sensor_info["temperature1"] = std::to_string(info_packet.fault_diagnosis.temperature1.value());
    sensor_info["temperature2"] = std::to_string(info_packet.fault_diagnosis.temperature2.value());
    sensor_info["temperature3"] = std::to_string(info_packet.fault_diagnosis.temperature3.value());
    sensor_info["temperature4"] = std::to_string(info_packet.fault_diagnosis.temperature4.value());
    sensor_info["temperature5"] = std::to_string(info_packet.fault_diagnosis.temperature5.value());
    sensor_info["r_rpm1"] = std::to_string(info_packet.fault_diagnosis.r_rpm1.value());
    sensor_info["r_rpm2"] = std::to_string(info_packet.fault_diagnosis.r_rpm2.value());
    //        sensor_info["gprmc"] = std::to_string(info_packet.gprmc.getGprmc());

    return sensor_info;
  }
};
}  // namespace drivers
}  // namespace nebula