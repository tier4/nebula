#pragma once

#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_sensor.hpp"

#include "boost/endian/buffers.hpp"

#include <bitset>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace nebula
{
namespace drivers
{
namespace robosense_packet
{
namespace bpearl_v3
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
  boost::endian::big_uint16_buf_t microsecond;

  [[nodiscard]] uint64_t get_time_in_ns() const
  {
    std::tm tm{};
    tm.tm_year = year.value() + 100;
    tm.tm_mon = month.value() - 1;  // starts from 0 in C
    tm.tm_mday = day.value();
    tm.tm_hour = hour.value();
    tm.tm_min = minute.value();
    tm.tm_sec = second.value();
    const uint64_t time = timegm(&tm) * 1000000000ULL + millisecond.value() * 1000000ULL +
                          microsecond.value() * 1000ULL;
    return time;
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
  uint8_t reserved_first[11];
  boost::endian::big_uint16_buf_t manc_err1;
  boost::endian::big_uint16_buf_t manc_err2;
  boost::endian::big_uint8_buf_t gps_st;
  boost::endian::big_uint8_buf_t temperature1;
  boost::endian::big_uint8_buf_t temperature2;
  uint8_t reserved_second[2];
  boost::endian::big_uint8_buf_t temperature3;
  boost::endian::big_uint8_buf_t temperature4;
  boost::endian::big_uint8_buf_t temperature5;
  boost::endian::big_uint8_buf_t temperature6;
  uint8_t reserved_third[7];
  boost::endian::big_uint8_buf_t r_rpm1;
  boost::endian::big_uint8_buf_t r_rpm2;
  uint8_t reserved_fourth[7];
};

struct InfoPacket
{
  boost::endian::big_uint64_buf_t header;
  boost::endian::big_uint16_buf_t motor_speed;
  Ethernet ethernet;
  FovSetting fov_setting;
  boost::endian::big_uint16_buf_t tcp_msop_port;
  boost::endian::big_uint16_buf_t phase_lock;
  FirmwareVersion top_firmware_version;
  FirmwareVersion bottom_firmware_version;
  FirmwareVersion bottom_software_version;
  FirmwareVersion motor_firmware_version;
  uint8_t reserved_first[230];
  boost::endian::big_uint16_buf_t reverse_zero_angle_offset;
  SerialNumber serial_number;
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
  SensorCalibration sensor_calibration;
  uint8_t reserved_fourth[586];
  boost::endian::big_uint16_buf_t tail;
};

#pragma pack(pop)
}  // namespace bpearl_v3

/// @brief Get the distance unit of the given @ref BpearlV3 packet in meters.
/// @return 0.005m (0.5cm)
template <>
double get_dis_unit<bpearl_v3::Packet>(const bpearl_v3::Packet & /* packet */)
{
  return 0.005;
}

}  // namespace robosense_packet

class BpearlV3 : public RobosenseSensor<
                   robosense_packet::bpearl_v3::Packet, robosense_packet::bpearl_v3::InfoPacket>
{
private:
  static constexpr int firing_time_offset_ns_single_[12][32]{
    {0,   256, 512, 768, 1024, 1280, 1536, 1792, 2568, 2824, 3080, 3336, 3592, 3848, 4104, 4360,
     128, 384, 640, 896, 1152, 1408, 1664, 1920, 2696, 2952, 3208, 3464, 3720, 3976, 4232, 4488},
    {5552, 5808, 6064, 6320, 6576, 6832, 7088, 7344, 8120, 8376, 8632,
     8888, 9144, 9400, 9656, 9912, 5680, 5936, 6192, 6448, 6704, 6960,
     7216, 7472, 8248, 8504, 8760, 9016, 9272, 9528, 9784, 10040},
    {11104, 11360, 11616, 11872, 12128, 12384, 12640, 12896, 13672, 13928, 14184,
     14440, 14696, 14952, 15208, 15464, 11232, 11488, 11744, 12000, 12256, 12512,
     12768, 13024, 13800, 14056, 14312, 14568, 14824, 15080, 15336, 15592},
    {16656, 16912, 17168, 17424, 17680, 17936, 18192, 18448, 19224, 19480, 19736,
     19992, 20248, 20504, 20760, 21016, 16784, 17040, 17296, 17552, 17808, 18064,
     18320, 18576, 19352, 19608, 19864, 20120, 20376, 20632, 20888, 21144},
    {22208, 22464, 22720, 22976, 23232, 23488, 23744, 24000, 24776, 25032, 25288,
     25544, 25800, 26056, 26312, 26568, 22336, 22592, 22848, 23104, 23360, 23616,
     23872, 24128, 24904, 25160, 25416, 25672, 25928, 26184, 26440, 26696},
    {27760, 28016, 28272, 28528, 28784, 29040, 29296, 29552, 30328, 30584, 30840,
     31096, 31352, 31608, 31864, 32120, 27888, 28144, 28400, 28656, 28912, 29168,
     29424, 29680, 30456, 30712, 30968, 31224, 31480, 31736, 31992, 32248},
    {33312, 33568, 33824, 34080, 34336, 34592, 34848, 35104, 35880, 36136, 36392,
     36648, 36904, 37160, 37416, 37672, 33440, 33696, 33952, 34208, 34464, 34720,
     34976, 35232, 36008, 36264, 36520, 36776, 37032, 37288, 37544, 37800},
    {38864, 39120, 39376, 39632, 39888, 40144, 40400, 40656, 41432, 41688, 41944,
     42200, 42456, 42712, 42968, 43224, 38992, 39248, 39504, 39760, 40016, 40272,
     40528, 40784, 41560, 41816, 42072, 42328, 42584, 42840, 43096, 43352},
    {44416, 44672, 44928, 45184, 45440, 45696, 45952, 46208, 46984, 47240, 47496,
     47752, 48008, 48264, 48520, 48776, 44544, 44800, 45056, 45312, 45568, 45824,
     46080, 46336, 47112, 47368, 47624, 47880, 48136, 48392, 48648, 48904},
    {49968, 50224, 50480, 50736, 50992, 51248, 51504, 51760, 52536, 52792, 53048,
     53304, 53560, 53816, 54072, 54328, 50096, 50352, 50608, 50864, 51120, 51376,
     51632, 51888, 52664, 52920, 53176, 53432, 53688, 53944, 54200, 54456},
    {55520, 55776, 56032, 56288, 56544, 56800, 57056, 57312, 58088, 58344, 58600,
     58856, 59112, 59368, 59624, 59880, 55648, 55904, 56160, 56416, 56672, 56928,
     57184, 57440, 58216, 58472, 58728, 58984, 59240, 59496, 59752, 60008},
    {61072, 61328, 61584, 61840, 62096, 62352, 62608, 62864, 63640, 63896, 64152,
     64408, 64664, 64920, 65176, 65432, 61200, 61456, 61712, 61968, 62224, 62480,
     62736, 62992, 63768, 64024, 64280, 64536, 64792, 65048, 65304, 65560}};

  static constexpr int firing_time_offset_ns_dual_[12][32]{
    {0,   256, 512, 768, 1024, 1280, 1536, 1792, 2568, 2824, 3080, 3336, 3592, 3848, 4104, 4360,
     128, 384, 640, 896, 1152, 1408, 1664, 1920, 2696, 2952, 3208, 3464, 3720, 3976, 4232, 4488},
    {0,   256, 512, 768, 1024, 1280, 1536, 1792, 2568, 2824, 3080, 3336, 3592, 3848, 4104, 4360,
     128, 384, 640, 896, 1152, 1408, 1664, 1920, 2696, 2952, 3208, 3464, 3720, 3976, 4232, 4488},
    {5552, 5808, 6064, 6320, 6576, 6832, 7088, 7344, 8120, 8376, 8632,
     8888, 9144, 9400, 9656, 9912, 5680, 5936, 6192, 6448, 6704, 6960,
     7216, 7472, 8248, 8504, 8760, 9016, 9272, 9528, 9784, 10040},
    {5552, 5808, 6064, 6320, 6576, 6832, 7088, 7344, 8120, 8376, 8632,
     8888, 9144, 9400, 9656, 9912, 5680, 5936, 6192, 6448, 6704, 6960,
     7216, 7472, 8248, 8504, 8760, 9016, 9272, 9528, 9784, 10040},
    {11104, 11360, 11616, 11872, 12128, 12384, 12640, 12896, 13672, 13928, 14184,
     14440, 14696, 14952, 15208, 15464, 11232, 11488, 11744, 12000, 12256, 12512,
     12768, 13024, 13800, 14056, 14312, 14568, 14824, 15080, 15336, 15592},
    {11104, 11360, 11616, 11872, 12128, 12384, 12640, 12896, 13672, 13928, 14184,
     14440, 14696, 14952, 15208, 15464, 11232, 11488, 11744, 12000, 12256, 12512,
     12768, 13024, 13800, 14056, 14312, 14568, 14824, 15080, 15336, 15592},
    {16656, 16912, 17168, 17424, 17680, 17936, 18192, 18448, 19224, 19480, 19736,
     19992, 20248, 20504, 20760, 21016, 16784, 17040, 17296, 17552, 17808, 18064,
     18320, 18576, 19352, 19608, 19864, 20120, 20376, 20632, 20888, 21144},
    {16656, 16912, 17168, 17424, 17680, 17936, 18192, 18448, 19224, 19480, 19736,
     19992, 20248, 20504, 20760, 21016, 16784, 17040, 17296, 17552, 17808, 18064,
     18320, 18576, 19352, 19608, 19864, 20120, 20376, 20632, 20888, 21144},
    {22208, 22464, 22720, 22976, 23232, 23488, 23744, 24000, 24776, 25032, 25288,
     25544, 25800, 26056, 26312, 26568, 22336, 22592, 22848, 23104, 23360, 23616,
     23872, 24128, 24904, 25160, 25416, 25672, 25928, 26184, 26440, 26696},
    {22208, 22464, 22720, 22976, 23232, 23488, 23744, 24000, 24776, 25032, 25288,
     25544, 25800, 26056, 26312, 26568, 22336, 22592, 22848, 23104, 23360, 23616,
     23872, 24128, 24904, 25160, 25416, 25672, 25928, 26184, 26440, 26696},
    {27760, 28016, 28272, 28528, 28784, 29040, 29296, 29552, 30328, 30584, 30840,
     31096, 31352, 31608, 31864, 32120, 27888, 28144, 28400, 28656, 28912, 29168,
     29424, 29680, 30456, 30712, 30968, 31224, 31480, 31736, 31992, 32248},
    {27760, 28016, 28272, 28528, 28784, 29040, 29296, 29552, 30328, 30584, 30840,
     31096, 31352, 31608, 31864, 32120, 27888, 28144, 28400, 28656, 28912, 29168,
     29424, 29680, 30456, 30712, 30968, 31224, 31480, 31736, 31992, 32248}};

public:
  static constexpr float MIN_RANGE = 0.1f;
  static constexpr float MAX_RANGE = 30.f;
  static constexpr size_t MAX_SCAN_BUFFER_POINTS = 1152000;

  int getPacketRelativePointTimeOffset(
    uint32_t block_id, uint32_t channel_id,
    const std::shared_ptr<RobosenseSensorConfiguration> & sensor_configuration) override
  {
    if (sensor_configuration->return_mode == ReturnMode::DUAL)
      return firing_time_offset_ns_dual_[block_id][channel_id];
    else
      return firing_time_offset_ns_single_[block_id][channel_id];
  }

  ReturnMode getReturnMode(const robosense_packet::bpearl_v3::InfoPacket & info_packet)
  {
    const uint8_t return_mode_data = info_packet.return_mode.value();
    if (return_mode_data == 0x00) {
      return ReturnMode::DUAL;
    } else if (return_mode_data == 0x01) {
      return ReturnMode::SINGLE_STRONGEST;
    } else if (return_mode_data == 0x02) {
      return ReturnMode::SINGLE_LAST;
    }
    return ReturnMode::UNKNOWN;
  }

  RobosenseCalibrationConfiguration getSensorCalibration(
    const robosense_packet::bpearl_v3::InfoPacket & info_packet)
  {
    return info_packet.sensor_calibration.getCalibration();
  }

  bool getSyncStatus(const robosense_packet::bpearl_v3::InfoPacket & info_packet)
  {
    if (info_packet.sync_status.value() != 0x00) {
      return true;
    }
    return false;
  }

  std::map<std::string, std::string> getSensorInfo(
    const robosense_packet::bpearl_v3::InfoPacket & info_packet)
  {
    std::map<std::string, std::string> sensor_info;
    sensor_info["motor_speed"] = std::to_string(info_packet.motor_speed.value());
    sensor_info["lidar_ip"] = info_packet.ethernet.lidar_ip.to_string();
    sensor_info["dest_pc_ip"] = info_packet.ethernet.dest_pc_ip.to_string();
    sensor_info["mac_addr"] = info_packet.ethernet.mac_addr.to_string();
    sensor_info["lidar_out_msop_port"] =
      std::to_string(info_packet.ethernet.lidar_out_msop_port.value());
    sensor_info["lidar_out_difop_port"] =
      std::to_string(info_packet.ethernet.lidar_out_difop_port.value());
    sensor_info["fov_start"] =
      robosense_packet::get_float_value(info_packet.fov_setting.fov_start.value());
    sensor_info["fov_end"] =
      robosense_packet::get_float_value(info_packet.fov_setting.fov_end.value());
    sensor_info["tcp_msop_port"] = std::to_string(info_packet.tcp_msop_port.value());
    sensor_info["phase_lock"] = std::to_string(info_packet.phase_lock.value());
    sensor_info["top_firmware_version"] = info_packet.top_firmware_version.to_string();
    sensor_info["bottom_firmware_version"] = info_packet.bottom_firmware_version.to_string();
    sensor_info["bottom_software_version"] = info_packet.bottom_software_version.to_string();
    sensor_info["motor_firmware_version"] = info_packet.motor_firmware_version.to_string();
    sensor_info["reverse_zero_angle_offset"] =
      std::to_string(info_packet.reverse_zero_angle_offset.value());
    sensor_info["serial_number"] = info_packet.serial_number.to_string();
    sensor_info["zero_angle_offset"] = std::to_string(info_packet.zero_angle_offset.value());

    if (info_packet.return_mode.value() == 0x00)
      sensor_info["return_mode"] = "dual";
    else if (info_packet.return_mode.value() == 0x01)
      sensor_info["return_mode"] = "strongest";
    else if (info_packet.return_mode.value() == 0x02)
      sensor_info["return_mode"] = "last";

    if (info_packet.time_sync_mode.value() == 0) sensor_info["time_sync_mode"] = "gps";
    if (info_packet.time_sync_mode.value() == 1) sensor_info["time_sync_mode"] = "e2e";
    if (info_packet.time_sync_mode.value() == 2) sensor_info["time_sync_mode"] = "p2p";
    if (info_packet.time_sync_mode.value() == 3) sensor_info["time_sync_mode"] = "gptp";

    if (info_packet.sync_status.value() == 0) sensor_info["sync_status"] = "time_sync_invalid";
    if (info_packet.sync_status.value() == 1)
      sensor_info["sync_status"] = "gps_time_sync_successful";
    if (info_packet.sync_status.value() == 2)
      sensor_info["sync_status"] = "ptp_time_sync_successful";

    sensor_info["time"] = std::to_string(info_packet.time.get_time_in_ns());
    sensor_info["v_dat_0v5"] =
      robosense_packet::get_float_value(info_packet.operating_status.v_dat_0v5.value());
    sensor_info["v_dat_12v"] =
      robosense_packet::get_float_value(info_packet.operating_status.v_dat_12v.value());
    sensor_info["v_dat_5v"] =
      robosense_packet::get_float_value(info_packet.operating_status.v_dat_5v.value());
    sensor_info["v_dat_1v25"] =
      robosense_packet::get_float_value(info_packet.operating_status.v_dat_1v25.value());
    sensor_info["v_dat_0v"] =
      robosense_packet::get_float_value(info_packet.operating_status.v_dat_0v.value());
    sensor_info["v_dat_1v"] =
      robosense_packet::get_float_value(info_packet.operating_status.v_dat_1v.value());
    sensor_info["rotation_direction"] = std::to_string(info_packet.rotation_direction.value());
    sensor_info["elapsed_time_flag"] = std::to_string(info_packet.elapsed_time_flag.value());

    sensor_info["manc_err1"] = std::to_string(info_packet.fault_diagnosis.manc_err1.value());
    sensor_info["manc_err2"] = std::to_string(info_packet.fault_diagnosis.manc_err2.value());

    std::bitset<8> gps_st_bits{info_packet.fault_diagnosis.gps_st.value()};
    if (gps_st_bits[0] == 1)
      sensor_info["pps_lock"] = "valid";
    else
      sensor_info["pps_lock"] = "invalid";
    if (gps_st_bits[1] == 1)
      sensor_info["gprmc_lock"] = "valid";
    else
      sensor_info["gprmc_lock"] = "invalid";
    if (gps_st_bits[2] == 1)
      sensor_info["utc_lock"] = "synchronized";
    else
      sensor_info["utc_lock"] = "not_synchronized";
    if (gps_st_bits[3] == 1)
      sensor_info["gprmc_input_status"] = "received_gprmc";
    else
      sensor_info["gprmc_input_status"] = "no_gprmc";
    if (gps_st_bits[4] == 1)
      sensor_info["pps_input_status"] = "received_pps";
    else
      sensor_info["pps_input_status"] = "no_pps";

    sensor_info["bottom_board_temp"] = std::to_string(
      (static_cast<float>(info_packet.fault_diagnosis.temperature1.value()) * 256 +
       static_cast<float>(info_packet.fault_diagnosis.temperature2.value())) *
        503.975 / 4096.0 -
      273.15);

    double apd_temp = ((info_packet.fault_diagnosis.temperature3.value() * 256 +
                        info_packet.fault_diagnosis.temperature4.value()) &
                       0x7FF8) /
                      128.0;
    if (
      ((info_packet.fault_diagnosis.temperature3.value() << 8)) +
        info_packet.fault_diagnosis.temperature4.value() >
      32768)
      apd_temp = -apd_temp;
    sensor_info["apd_temp"] = std::to_string(apd_temp);

    double top_board_temp = ((info_packet.fault_diagnosis.temperature5.value() * 256 +
                              info_packet.fault_diagnosis.temperature6.value()) &
                             0x7FF8) /
                            128.0;
    if (
      ((info_packet.fault_diagnosis.temperature5.value() << 8)) +
        info_packet.fault_diagnosis.temperature6.value() >
      32768)
      top_board_temp = -top_board_temp;
    sensor_info["top_board_temp"] = std::to_string(top_board_temp);

    sensor_info["real_time_rot_speed"] = std::to_string(
      (info_packet.fault_diagnosis.r_rpm1.value() * 256 +
       info_packet.fault_diagnosis.r_rpm2.value()) /
      6);

    std::string gprmc_string;
    for (auto i : info_packet.gprmc) {
      gprmc_string += static_cast<char>(i.value());
    }
    sensor_info["gprmc_string"] = gprmc_string;

    return sensor_info;
  }
};
}  // namespace drivers
}  // namespace nebula