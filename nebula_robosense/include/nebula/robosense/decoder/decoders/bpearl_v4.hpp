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
namespace bpearl_v4
{
#pragma pack(push, 1)

struct Header
{
  big_uint64_buf_t header_id;
  uint8_t reserved_first[4];
  big_uint32_buf_t packet_count;
  uint8_t reserved_second[4];
  Timestamp timestamp;
  uint8_t reserved_third[1];
  big_uint8_buf_t lidar_type;
  big_uint8_buf_t lidar_model;
  uint8_t reserved_fourth[9];
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
  big_uint8_buf_t reserved_first;
  big_uint16_buf_t machine_current;
  big_uint24_buf_t reserved_second;
  big_uint16_buf_t machine_voltage;
  uint8_t reserved_third[16];
};

struct FaultDiagnosis
{
  big_uint16_buf_t startup_times;
  big_uint32_buf_t reserved;
  big_uint8_buf_t gps_status;
  big_uint16_buf_t machine_temp;
  uint8_t reserved_first[11];
  big_uint16_buf_t phase;
  big_uint16_buf_t rotation_speed;
};

struct InfoPacket
{
  big_uint64_buf_t header;
  big_uint16_buf_t motor_speed_setting;
  Ethernet ethernet;
  FovSetting fov_setting;
  big_uint16_buf_t tcp_msop_port;
  big_uint16_buf_t phase_lock;
  FirmwareVersion mainboard_firmware_version;
  FirmwareVersion bottom_firmware_version;
  FirmwareVersion app_software_version;
  FirmwareVersion motor_firmware_version;
  uint8_t reserved_first[228];
  big_uint8_buf_t baud_rate;
  uint8_t reserved_second[3];
  SerialNumber serial_number;
  uint8_t reserved_third[2];
  big_uint8_buf_t return_mode;
  big_uint8_buf_t time_sync_mode;
  big_uint8_buf_t time_sync_state;
  Timestamp time;
  OperatingStatus operating_status;
  big_uint8_buf_t rotation_direction;
  big_uint32_buf_t running_time;
  uint8_t reserved_fourth[9];
  FaultDiagnosis fault_diagnosis;
  uint8_t reserved_fifth[7];
  big_uint8_buf_t gprmc[86];
  SensorCalibration sensor_calibration;
  uint8_t reserved_sixth[586];
  big_uint16_buf_t tail;
};

#pragma pack(pop)
}  // namespace bpearl_v4

/// @brief Get the distance unit of the given @ref BpearlV3 packet in meters.
/// @return 0.0025m (0.25cm)
template <>
inline double get_dis_unit<bpearl_v4::Packet>(const bpearl_v4::Packet & /* packet */)
{
  return 0.0025;
}

}  // namespace robosense_packet

class BpearlV4 : public RobosenseSensor<
                   robosense_packet::bpearl_v4::Packet, robosense_packet::bpearl_v4::InfoPacket>
{
private:
  static constexpr int firing_time_offset_ns_single[12][32] = {
    {0,    167,  334,  500,  667,  834,  1001, 1168, 1334, 1501, 1668,
     1835, 2002, 2168, 2335, 2502, 2669, 2836, 3002, 3169, 3336, 3503,
     3670, 3836, 4003, 4170, 4337, 4504, 4670, 4837, 5004, 5171},
    {5555, 5722, 5889, 6055, 6222, 6389,  6556,  6723,  6889,  7056, 7223,
     7390, 7557, 7723, 7890, 8057, 8224,  8391,  8557,  8724,  8891, 9058,
     9225, 9391, 9558, 9725, 9892, 10059, 10225, 10392, 10559, 10726},
    {11110, 11277, 11444, 11610, 11777, 11944, 12111, 12278, 12444, 12611, 12778,
     12945, 13112, 13278, 13445, 13612, 13779, 13946, 14112, 14279, 14446, 14613,
     14780, 14946, 15113, 15280, 15447, 15614, 15780, 15947, 16114, 16281},
    {16665, 16832, 16999, 17165, 17332, 17499, 17666, 17833, 17999, 18166, 18333,
     18500, 18667, 18833, 19000, 19167, 19334, 19501, 19667, 19834, 20001, 20168,
     20335, 20501, 20668, 20835, 21002, 21169, 21335, 21502, 21669, 21836},
    {22220, 22387, 22554, 22720, 22887, 23054, 23221, 23388, 23554, 23721, 23888,
     24055, 24222, 24388, 24555, 24722, 24889, 25056, 25222, 25389, 25556, 25723,
     25890, 26056, 26223, 26390, 26557, 26724, 26890, 27057, 27224, 27391},
    {27775, 27942, 28109, 28275, 28442, 28609, 28776, 28943, 29109, 29276, 29443,
     29610, 29777, 29943, 30110, 30277, 30444, 30611, 30777, 30944, 31111, 31278,
     31445, 31611, 31778, 31945, 32112, 32279, 32445, 32612, 32779, 32946},
    {33330, 33497, 33664, 33830, 33997, 34164, 34331, 34498, 34664, 34831, 34998,
     35165, 35332, 35498, 35665, 35832, 35999, 36166, 36332, 36499, 36666, 36833,
     37000, 37166, 37333, 37500, 37667, 37834, 38000, 38167, 38334, 38501},
    {38885, 39052, 39219, 39385, 39552, 39719, 39886, 40053, 40219, 40386, 40553,
     40720, 40887, 41053, 41220, 41387, 41554, 41721, 41887, 42054, 42221, 42388,
     42555, 42721, 42888, 43055, 43222, 43389, 43555, 43722, 43889, 44056},
    {44440, 44607, 44774, 44940, 45107, 45274, 45441, 45608, 45774, 45941, 46108,
     46275, 46442, 46608, 46775, 46942, 47109, 47276, 47442, 47609, 47776, 47943,
     48110, 48276, 48443, 48610, 48777, 48944, 49110, 49277, 49444, 49611},
    {49995, 50162, 50329, 50495, 50662, 50829, 50996, 51163, 51329, 51496, 51663,
     51830, 51997, 52163, 52330, 52497, 52664, 52831, 52997, 53164, 53331, 53498,
     53665, 53831, 53998, 54165, 54332, 54499, 54665, 54832, 54999, 55166},
    {55550, 55717, 55884, 56050, 56217, 56384, 56551, 56718, 56884, 57051, 57218,
     57385, 57552, 57718, 57885, 58052, 58219, 58386, 58552, 58719, 58886, 59053,
     59220, 59386, 59553, 59720, 59887, 60054, 60220, 60387, 60554, 60721},
    {61105, 61272, 61439, 61605, 61772, 61939, 62106, 62273, 62439, 62606, 62773,
     62940, 63107, 63273, 63440, 63607, 63774, 63941, 64107, 64274, 64441, 64608,
     64775, 64941, 65108, 65275, 65442, 65609, 65775, 65942, 66109, 66276}};

  static constexpr int firing_time_offset_ns_dual[12][32]{
    {0,    167,  334,  500,  667,  834,  1001, 1168, 1334, 1501, 1668,
     1835, 2002, 2168, 2335, 2502, 2669, 2836, 3002, 3169, 3336, 3503,
     3670, 3836, 4003, 4170, 4337, 4504, 4670, 4837, 5004, 5171},
    {0,    167,  334,  500,  667,  834,  1001, 1168, 1334, 1501, 1668,
     1835, 2002, 2168, 2335, 2502, 2669, 2836, 3002, 3169, 3336, 3503,
     3670, 3836, 4003, 4170, 4337, 4504, 4670, 4837, 5004, 5171},
    {5555, 5722, 5889, 6055, 6222, 6389,  6556,  6723,  6889,  7056, 7223,
     7390, 7557, 7723, 7890, 8057, 8224,  8391,  8557,  8724,  8891, 9058,
     9225, 9391, 9558, 9725, 9892, 10059, 10225, 10392, 10559, 10726},
    {5555, 5722, 5889, 6055, 6222, 6389,  6556,  6723,  6889,  7056, 7223,
     7390, 7557, 7723, 7890, 8057, 8224,  8391,  8557,  8724,  8891, 9058,
     9225, 9391, 9558, 9725, 9892, 10059, 10225, 10392, 10559, 10726},
    {11110, 11277, 11444, 11610, 11777, 11944, 12111, 12278, 12444, 12611, 12778,
     12945, 13112, 13278, 13445, 13612, 13779, 13946, 14112, 14279, 14446, 14613,
     14780, 14946, 15113, 15280, 15447, 15614, 15780, 15947, 16114, 16281},
    {11110, 11277, 11444, 11610, 11777, 11944, 12111, 12278, 12444, 12611, 12778,
     12945, 13112, 13278, 13445, 13612, 13779, 13946, 14112, 14279, 14446, 14613,
     14780, 14946, 15113, 15280, 15447, 15614, 15780, 15947, 16114, 16281},
    {16665, 16832, 16999, 17165, 17332, 17499, 17666, 17833, 17999, 18166, 18333,
     18500, 18667, 18833, 19000, 19167, 19334, 19501, 19667, 19834, 20001, 20168,
     20335, 20501, 20668, 20835, 21002, 21169, 21335, 21502, 21669, 21836},
    {16665, 16832, 16999, 17165, 17332, 17499, 17666, 17833, 17999, 18166, 18333,
     18500, 18667, 18833, 19000, 19167, 19334, 19501, 19667, 19834, 20001, 20168,
     20335, 20501, 20668, 20835, 21002, 21169, 21335, 21502, 21669, 21836},
    {22220, 22387, 22554, 22720, 22887, 23054, 23221, 23388, 23554, 23721, 23888,
     24055, 24222, 24388, 24555, 24722, 24889, 25056, 25222, 25389, 25556, 25723,
     25890, 26056, 26223, 26390, 26557, 26724, 26890, 27057, 27224, 27391},
    {22220, 22387, 22554, 22720, 22887, 23054, 23221, 23388, 23554, 23721, 23888,
     24055, 24222, 24388, 24555, 24722, 24889, 25056, 25222, 25389, 25556, 25723,
     25890, 26056, 26223, 26390, 26557, 26724, 26890, 27057, 27224, 27391},
    {27775, 27942, 28109, 28275, 28442, 28609, 28776, 28943, 29109, 29276, 29443,
     29610, 29777, 29943, 30110, 30277, 30444, 30611, 30777, 30944, 31111, 31278,
     31445, 31611, 31778, 31945, 32112, 32279, 32445, 32612, 32779, 32946},
    {27775, 27942, 28109, 28275, 28442, 28609, 28776, 28943, 29109, 29276, 29443,
     29610, 29777, 29943, 30110, 30277, 30444, 30611, 30777, 30944, 31111, 31278,
     31445, 31611, 31778, 31945, 32112, 32279, 32445, 32612, 32779, 32946}};

  static constexpr uint8_t dual_return_flag = 0x00;
  static constexpr uint8_t strongest_return_flag = 0x04;
  static constexpr uint8_t last_return_flag = 0x05;
  static constexpr uint8_t first_return_flag = 0x06;

  static constexpr uint8_t sync_mode_gps_flag = 0x00;
  static constexpr uint8_t sync_mode_e2e_flag = 0x01;
  static constexpr uint8_t sync_mode_p2p_flag = 0x02;
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

  ReturnMode get_return_mode(const robosense_packet::bpearl_v4::InfoPacket & info_packet) override
  {
    switch (info_packet.return_mode.value()) {
      case dual_return_flag:
        return ReturnMode::DUAL;
      case strongest_return_flag:
        return ReturnMode::SINGLE_STRONGEST;
      case last_return_flag:
        return ReturnMode::SINGLE_LAST;
      case first_return_flag:
        return ReturnMode::SINGLE_FIRST;
      default:
        return ReturnMode::UNKNOWN;
    }
  }

  RobosenseCalibrationConfiguration get_sensor_calibration(
    const robosense_packet::bpearl_v4::InfoPacket & info_packet) override
  {
    return info_packet.sensor_calibration.get_calibration();
  }

  bool get_sync_status(const robosense_packet::bpearl_v4::InfoPacket & info_packet) override
  {
    switch (info_packet.time_sync_state.value()) {
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
    const robosense_packet::bpearl_v4::InfoPacket & info_packet) override
  {
    std::map<std::string, std::string> sensor_info;
    sensor_info["motor_speed"] = std::to_string(info_packet.motor_speed_setting.value());
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
    sensor_info["mainboard_firmware_version"] = info_packet.mainboard_firmware_version.to_string();
    sensor_info["bottom_firmware_version"] = info_packet.bottom_firmware_version.to_string();
    sensor_info["app_software_version"] = info_packet.app_software_version.to_string();
    sensor_info["motor_firmware_version"] = info_packet.motor_firmware_version.to_string();
    sensor_info["baud_rate"] = std::to_string(info_packet.baud_rate.value());
    sensor_info["serial_number"] = info_packet.serial_number.to_string();

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
      case first_return_flag:
        sensor_info["return_mode"] = "first";
        break;
      default:
        sensor_info["return_mode"] = "n/a";
        break;
    }

    switch (info_packet.time_sync_mode.value()) {
      case sync_mode_gps_flag:
        sensor_info["time_sync_mode"] = "gps";
        break;
      case sync_mode_e2e_flag:
        sensor_info["time_sync_mode"] = "e2e";
        break;
      case sync_mode_p2p_flag:
        sensor_info["time_sync_mode"] = "p2p";
        break;
      case sync_mode_gptp_flag:
        sensor_info["time_sync_mode"] = "gptp";
        break;
      default:
        sensor_info["time_sync_mode"] = "n/a";
        break;
    }

    switch (info_packet.time_sync_state.value()) {
      case sync_status_invalid_flag:
        sensor_info["time_sync_state"] = "time_sync_invalid";
        break;
      case sync_status_gps_success_flag:
        sensor_info["time_sync_state"] = "gps_time_sync_successful";
        break;
      case sync_status_ptp_success_flag:
        sensor_info["time_sync_state"] = "ptp_time_sync_successful";
        break;
      default:
        sensor_info["time_sync_state"] = "n/a";
    }

    sensor_info["time"] = std::to_string(info_packet.time.get_time_in_ns());
    sensor_info["machine_current"] =
      robosense_packet::get_float_value(info_packet.operating_status.machine_current.value());
    sensor_info["machine_voltage"] =
      robosense_packet::get_float_value(info_packet.operating_status.machine_voltage.value());
    sensor_info["rotation_direction"] = std::to_string(info_packet.rotation_direction.value());
    sensor_info["running_time"] = std::to_string(info_packet.running_time.value());
    sensor_info["startup_times"] =
      std::to_string(info_packet.fault_diagnosis.startup_times.value());

    const std::bitset<8> gps_st_bits{info_packet.fault_diagnosis.gps_status.value()};
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

    sensor_info["machine_temp"] =
      robosense_packet::get_float_value(info_packet.fault_diagnosis.machine_temp.value());
    sensor_info["phase"] = std::to_string(info_packet.fault_diagnosis.phase.value());
    sensor_info["rotation_speed"] =
      std::to_string(info_packet.fault_diagnosis.rotation_speed.value());

    std::string gprmc_string;
    for (auto i : info_packet.gprmc) {
      gprmc_string += static_cast<char>(i.value());
    }
    sensor_info["gprmc_string"] = gprmc_string;

    return sensor_info;
  }
};
}  // namespace nebula::drivers
