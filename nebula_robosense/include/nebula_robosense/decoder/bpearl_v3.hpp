// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_sensor.hpp"

#include "boost/endian/buffers.hpp"

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

using namespace boost::endian;  // NOLINT(build/namespaces)

namespace nebula::drivers
{
namespace robosense_packet
{
namespace bpearl_v3
{
#pragma pack(push, 1)

struct Timestamp
{
  big_uint8_buf_t year;
  big_uint8_buf_t month;
  big_uint8_buf_t day;
  big_uint8_buf_t hour;
  big_uint8_buf_t minute;
  big_uint8_buf_t second;
  big_uint16_buf_t millisecond;
  big_uint16_buf_t microsecond;

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
  big_uint64_buf_t header_id;
  big_uint32_buf_t checksum;
  big_uint32_buf_t packet_count;
  big_uint32_buf_t reserved_first;
  Timestamp timestamp;
  big_uint8_buf_t lidar_model;
  uint8_t reserved_second[7];
  big_uint16_buf_t temperature;
  big_uint16_buf_t top_board_temperature;
};

struct Packet : public PacketBase<12, 32, 2, 100>
{
  typedef Body<Block<Unit, Packet::n_channels>, Packet::n_blocks> body_t;
  Header header;
  body_t body;
  big_uint48_buf_t tail;
};

struct OperatingStatus
{
  big_uint48_buf_t reserved;
  big_uint16_buf_t v_dat_0v5;
  big_uint16_buf_t v_dat_12v;
  big_uint16_buf_t v_dat_5v;
  big_uint16_buf_t v_dat_1v25;
  big_uint16_buf_t v_dat_0v;
  big_uint16_buf_t v_dat_1v;
};

struct FaultDiagnosis
{
  uint8_t reserved_first[11];
  big_uint16_buf_t manc_err1;
  big_uint16_buf_t manc_err2;
  big_uint8_buf_t gps_st;
  big_uint8_buf_t temperature1;
  big_uint8_buf_t temperature2;
  uint8_t reserved_second[2];
  big_uint8_buf_t temperature3;
  big_uint8_buf_t temperature4;
  big_uint8_buf_t temperature5;
  big_uint8_buf_t temperature6;
  uint8_t reserved_third[7];
  big_uint8_buf_t r_rpm1;
  big_uint8_buf_t r_rpm2;
  uint8_t reserved_fourth[7];
};

struct InfoPacket
{
  big_uint64_buf_t header;
  big_uint16_buf_t motor_speed;
  Ethernet ethernet;
  FovSetting fov_setting;
  big_uint16_buf_t tcp_msop_port;
  big_uint16_buf_t phase_lock;
  FirmwareVersion top_firmware_version;
  FirmwareVersion bottom_firmware_version;
  FirmwareVersion bottom_software_version;
  FirmwareVersion motor_firmware_version;
  uint8_t reserved_first[230];
  big_uint16_buf_t reverse_zero_angle_offset;
  SerialNumber serial_number;
  big_uint16_buf_t zero_angle_offset;
  big_uint8_buf_t return_mode;
  big_uint8_buf_t time_sync_mode;
  big_uint8_buf_t sync_status;
  Timestamp time;
  OperatingStatus operating_status;
  uint8_t reserved_second[6];
  big_uint8_buf_t rotation_direction;
  big_uint32_buf_t elapsed_time_flag;
  FaultDiagnosis fault_diagnosis;
  big_uint8_buf_t gprmc[86];
  SensorCalibration sensor_calibration;
  uint8_t reserved_fourth[586];
  big_uint16_buf_t tail;
};

#pragma pack(pop)
}  // namespace bpearl_v3

/// @brief Get the distance unit of the given @ref BpearlV3 packet in meters.
/// @return 0.005m (0.5cm)
template <>
inline double get_dis_unit<bpearl_v3::Packet>(const bpearl_v3::Packet & /* packet */)
{
  return 0.005;
}

}  // namespace robosense_packet

class BpearlV3 : public RobosenseSensor<
                   robosense_packet::bpearl_v3::Packet, robosense_packet::bpearl_v3::InfoPacket>
{
private:
  static constexpr int firing_time_offset_ns_single[12][32]{
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

  static constexpr int firing_time_offset_ns_dual[12][32]{
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

  static constexpr uint8_t dual_return_flag = 0x00;
  static constexpr uint8_t strongest_return_flag = 0x01;
  static constexpr uint8_t last_return_flag = 0x02;

  static constexpr uint8_t sync_mode_gps_flag = 0x00;
  static constexpr uint8_t sync_mode_e2_e_flag = 0x01;
  static constexpr uint8_t sync_mode_p2_p_flag = 0x02;
  static constexpr uint8_t sync_mode_gptp_flag = 0x03;

  static constexpr uint8_t sync_status_invalid_flag = 0x00;
  static constexpr uint8_t sync_status_gps_success_flag = 0x01;
  static constexpr uint8_t sync_status_ptp_success_flag = 0x02;

public:
  static constexpr float min_range = 0.1f;
  static constexpr float max_range = 30.f;
  static constexpr size_t max_scan_buffer_points = 1152000;

  int get_packet_relative_point_time_offset(
    const uint32_t block_id, const uint32_t channel_id,
    const std::shared_ptr<const RobosenseSensorConfiguration> & sensor_configuration) override
  {
    if (sensor_configuration->return_mode == ReturnMode::DUAL)
      return firing_time_offset_ns_dual[block_id][channel_id];
    else
      return firing_time_offset_ns_single[block_id][channel_id];
  }

  ReturnMode get_return_mode(const robosense_packet::bpearl_v3::InfoPacket & info_packet) override
  {
    switch (info_packet.return_mode.value()) {
      case dual_return_flag:
        return ReturnMode::DUAL;
      case strongest_return_flag:
        return ReturnMode::SINGLE_STRONGEST;
      case last_return_flag:
        return ReturnMode::SINGLE_LAST;
      default:
        return ReturnMode::UNKNOWN;
    }
  }

  RobosenseCalibrationConfiguration get_sensor_calibration(
    const robosense_packet::bpearl_v3::InfoPacket & info_packet) override
  {
    return info_packet.sensor_calibration.get_calibration();
  }

  bool get_sync_status(const robosense_packet::bpearl_v3::InfoPacket & info_packet) override
  {
    switch (info_packet.sync_status.value()) {
      case sync_status_invalid_flag:
        return false;
      case sync_status_gps_success_flag:
        return true;
      case sync_status_ptp_success_flag:
        return true;
      default:
        return false;
    }
  }

  std::map<std::string, std::string> get_sensor_info(
    const robosense_packet::bpearl_v3::InfoPacket & info_packet) override
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

    switch (info_packet.return_mode.value()) {
      case dual_return_flag:
        sensor_info["return_mode"] = "dual";
        break;
      case strongest_return_flag:
        sensor_info["return_mode"] = "strongest";
        break;
      case last_return_flag:
        sensor_info["return_mode"] = "last";
        break;
      default:
        sensor_info["return_mode"] = "n/a";
        break;
    }

    switch (info_packet.time_sync_mode.value()) {
      case sync_mode_gps_flag:
        sensor_info["time_sync_mode"] = "gps";
        break;
      case sync_mode_e2_e_flag:
        sensor_info["time_sync_mode"] = "e2e";
        break;
      case sync_mode_p2_p_flag:
        sensor_info["time_sync_mode"] = "p2p";
        break;
      case sync_mode_gptp_flag:
        sensor_info["time_sync_mode"] = "gptp";
        break;
      default:
        sensor_info["time_sync_mode"] = "n/a";
        break;
    }

    switch (info_packet.sync_status.value()) {
      case sync_status_invalid_flag:
        sensor_info["sync_status"] = "time_sync_invalid";
        break;
      case sync_status_gps_success_flag:
        sensor_info["sync_status"] = "gps_time_sync_successful";
        break;
      case sync_status_ptp_success_flag:
        sensor_info["sync_status"] = "ptp_time_sync_successful";
        break;
      default:
        sensor_info["sync_status"] = "n/a";
    }

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

    const std::bitset<8> gps_st_bits{info_packet.fault_diagnosis.gps_st.value()};
    if (gps_st_bits[0])
      sensor_info["pps_lock"] = "valid";
    else
      sensor_info["pps_lock"] = "invalid";
    if (gps_st_bits[1])
      sensor_info["gprmc_lock"] = "valid";
    else
      sensor_info["gprmc_lock"] = "invalid";
    if (gps_st_bits[2])
      sensor_info["utc_lock"] = "synchronized";
    else
      sensor_info["utc_lock"] = "not_synchronized";
    if (gps_st_bits[3])
      sensor_info["gprmc_input_status"] = "received_gprmc";
    else
      sensor_info["gprmc_input_status"] = "no_gprmc";
    if (gps_st_bits[4])
      sensor_info["pps_input_status"] = "received_pps";
    else
      sensor_info["pps_input_status"] = "no_pps";

    /*
     * From the manual, here is the formula for calculating bottom board temperature:
     * Temp =（Value（temp1）* 256 + Value（temp2）& 0Xffff）* 503.975 / 4096.0 - 273.15
     */
    sensor_info["bottom_board_temp"] = std::to_string(
      (static_cast<float>(info_packet.fault_diagnosis.temperature1.value()) * 256 +
       static_cast<float>(info_packet.fault_diagnosis.temperature2.value())) *
        503.975 / 4096.0 -
      273.15);

    /*
     * From the manual, here is the formula for calculating APD temperature:
     * When Value ≤ 32768, the temperature value is positive; otherwise negative.
     * Temp = ( (value(362) * 256 + value(363) ) & 0x7FF8 ) / 128.0
     */
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

    /*
     * From the manual, here is the formula for calculating top board temperature:
     * When Value ≤ 32768, the temperature value is positive; otherwise negative.
     * Temp= ( (value(362) * 256 + value(363) ) & 0x7FF8 ) / 128.0
     */
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

    /*
     * From the manual, here is the formula for calculating real time rotation speed:
     * Speed = (256 * r_rpm1 + r_rpm2) / 6
     */
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
}  // namespace nebula::drivers
