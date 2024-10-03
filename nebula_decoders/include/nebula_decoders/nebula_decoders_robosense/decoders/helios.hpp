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
namespace robosense_packet::helios
{
#pragma pack(push, 1)

struct Header
{
  big_uint32_buf_t header_id;
  big_uint16_buf_t protocol_version;
  big_uint16_buf_t reserved_first;
  big_uint32_buf_t top_packet_count;
  big_uint32_buf_t bottom_packet_count;
  big_uint8_buf_t reserved_second;
  big_uint8_buf_t range_resolution;
  big_uint16_buf_t angle_interval_count;
  Timestamp timestamp;
  big_uint8_buf_t reserved_third;
  big_uint8_buf_t lidar_type;
  big_uint8_buf_t lidar_model;
  big_uint8_buf_t reserved_fourth[9];
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
  big_uint16_buf_t i_dat;
  big_uint16_buf_t v_dat;
  big_uint16_buf_t v_dat_12v;
  big_uint16_buf_t v_dat_5v;
  big_uint16_buf_t v_dat_2v5;
  big_uint16_buf_t v_dat_apd;
};

struct FaultDiagnosis
{
  big_uint16_buf_t temperature1;
  big_uint16_buf_t temperature2;
  big_uint16_buf_t temperature3;
  big_uint16_buf_t temperature4;
  big_uint16_buf_t temperature5;
  big_uint16_buf_t r_rpm;
  big_uint8_buf_t lane_up;
  big_uint16_buf_t lane_up_cnt;
  big_uint16_buf_t top_status;
  big_uint8_buf_t gps_status;
};

struct SensorHwVersion
{
  big_uint8_buf_t first_octet;
  big_uint8_buf_t second_octet;
  big_uint8_buf_t third_octet;

  [[nodiscard]] std::string to_string() const
  {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(first_octet.value())
       << std::setw(2) << static_cast<int>(second_octet.value()) << std::setw(2)
       << static_cast<int>(third_octet.value());
    return ss.str();
  }
};

struct WebPageVersion
{
  big_uint8_buf_t first_octet;
  big_uint8_buf_t second_octet;
  big_uint8_buf_t third_octet;
  big_uint8_buf_t fourth_octet;

  [[nodiscard]] std::string to_string() const
  {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(first_octet.value())
       << std::setw(2) << static_cast<int>(second_octet.value()) << std::setw(2)
       << static_cast<int>(third_octet.value()) << std::setw(2)
       << static_cast<int>(fourth_octet.value());
    return ss.str();
  }
};

struct InfoPacket
{
  big_uint64_buf_t header;
  big_uint16_buf_t motor_speed;
  Ethernet ethernet;
  FovSetting fov_setting;
  big_uint16_buf_t reserved_first;
  big_uint16_buf_t phase_lock;
  FirmwareVersion top_firmware_version;
  FirmwareVersion bottom_firmware_version;
  FirmwareVersion bottom_software_version;
  FirmwareVersion motor_firmware_version;
  SensorHwVersion sensor_hw_version;
  WebPageVersion web_page_version;
  big_uint32_buf_t top_backup_crc;
  big_uint32_buf_t bottom_backup_crc;
  big_uint32_buf_t software_backup_crc;
  big_uint32_buf_t webpage_backup_crc;
  IpAddress ethernet_gateway;
  IpAddress subnet_mask;
  uint8_t reserved_second[201];
  SerialNumber serial_number;
  big_uint16_buf_t zero_angle_offset;
  big_uint8_buf_t return_mode;
  big_uint8_buf_t time_sync_mode;
  big_uint8_buf_t sync_status;
  Timestamp time;
  OperatingStatus operating_status;
  uint8_t reserved_third[17];
  FaultDiagnosis fault_diagnosis;
  big_uint8_buf_t code_wheel_status;
  big_uint8_buf_t pps_trigger_mode;
  uint8_t reserved_fourth[20];
  big_uint8_buf_t gprmc[86];
  SensorCalibration sensor_calibration;
  uint8_t reserved_fifth[586];
  big_uint16_buf_t tail;
};

#pragma pack(pop)
}  // namespace robosense_packet::helios

class Helios
: public RobosenseSensor<robosense_packet::helios::Packet, robosense_packet::helios::InfoPacket>
{
private:
  static constexpr int firing_time_offset_ns_single[12][32] = {
    {0,     1570,  3150,  4720,  6300,  7870,  9450,  11360, 13260, 15170, 17080,
     18990, 20560, 22140, 23710, 25290, 26530, 29010, 27770, 30250, 31490, 32730,
     33980, 35220, 36460, 37700, 38940, 40180, 41420, 42670, 43910, 45150},
    {55560, 57130, 58700, 60280, 61850, 63430, 65000, 66910, 68820, 70730, 72640,
     74540, 76120, 77690, 79270, 80840, 82080, 84570, 83320, 85810, 87050, 89530,
     88290, 90770, 92010, 93260, 94500, 95740, 96980, 98220, 99460, 100700},
    {111110, 112690, 114260, 115840, 117410, 118980, 120560, 122470, 124380, 126280, 128190,
     130100, 131670, 133250, 134820, 136400, 137640, 140120, 138880, 141360, 142600, 145090,
     143850, 146330, 147570, 148810, 150050, 151290, 152540, 153780, 155020, 156260},
    {166670, 168240, 169820, 171390, 172970, 174540, 176110, 178020, 179930, 181840, 183750,
     185650, 187230, 188800, 190380, 191950, 193190, 195680, 194440, 196920, 198160, 200640,
     199400, 201880, 203130, 204370, 205610, 206850, 208090, 209330, 210570, 211810},
    {222220, 223800, 225370, 226950, 228520, 230100, 231670, 233580, 235490, 237390, 239300,
     241210, 242780, 244360, 245930, 247510, 248750, 251230, 249990, 252470, 253720, 256200,
     254960, 257440, 258680, 259920, 261160, 262400, 263650, 264890, 266130, 267370},
    {277780, 279350, 280930, 282500, 284080, 285650, 287230, 289130, 291040, 292950, 294860,
     296770, 298340, 299920, 301490, 303060, 304310, 306790, 305550, 308030, 309270, 311750,
     310510, 313000, 314240, 315480, 316720, 317960, 319200, 320440, 321680, 322930},
    {333330, 334910, 336480, 338060, 339630, 341210, 342780, 344690, 346600, 348510, 350410,
     352320, 353900, 355470, 357050, 358620, 359860, 362340, 361100, 363590, 364830, 367310,
     366070, 368550, 369790, 371030, 372270, 373520, 374760, 376000, 377240, 378480},
    {388890, 390460, 392040, 393610, 395190, 396760, 398340, 400240, 402150, 404060, 405970,
     407880, 409450, 411030, 412600, 414180, 415420, 417900, 416660, 419140, 420380, 422860,
     421620, 424110, 425350, 426590, 427830, 429070, 430310, 431550, 432800, 434040},
    {444440, 446020, 447590, 449170, 450740, 452320, 453890, 455800, 457710, 459620, 461520,
     463430, 465010, 466580, 468160, 469730, 470970, 473460, 472210, 474700, 475940, 478420,
     477180, 479660, 480900, 482140, 483390, 484630, 485870, 487110, 488350, 489590},
    {500000, 501570, 503150, 504720, 506300, 507870, 509450, 511360, 513260, 515170, 517080,
     518990, 520560, 522140, 523710, 525290, 526530, 529010, 527770, 530250, 531490, 533980,
     532730, 535220, 536460, 537700, 538940, 540180, 541420, 542670, 543910, 545150},
    {555560, 557130, 558700, 560280, 561850, 563430, 565000, 566910, 568820, 570730, 572640,
     574540, 576120, 577690, 579270, 580840, 582080, 584570, 583320, 585810, 587050, 589530,
     588290, 590770, 592010, 593260, 594500, 595740, 596980, 598220, 599460, 600700},
    {611110, 612690, 614260, 615840, 617410, 618980, 620560, 622470, 624380, 626280, 628190,
     630100, 631670, 633250, 634820, 636400, 637640, 640120, 638880, 641360, 642600, 645090,
     643850, 646330, 647570, 648810, 650050, 651290, 652540, 653780, 655020, 656260}};

  static constexpr int firing_time_offset_ns_dual[12][32]{
    {0,     1570,  3150,  4720,  6300,  7870,  9450,  11360, 13260, 15170, 17080,
     18990, 20560, 22140, 23710, 25290, 26530, 27770, 29010, 30250, 31490, 32730,
     33980, 35220, 36460, 37700, 38940, 40180, 41420, 42670, 43910, 45150},
    {0,     1570,  3150,  4720,  6300,  7870,  9450,  11360, 13260, 15170, 17080,
     18990, 20560, 22140, 23710, 25290, 26530, 27770, 29010, 30250, 31490, 32730,
     33980, 35220, 36460, 37700, 38940, 40180, 41420, 42670, 43910, 45150},
    {55560, 57130, 58700, 60280, 61850, 63430, 65000, 66910, 68820, 70730, 72640,
     74540, 76120, 77690, 79270, 80840, 82080, 83320, 84570, 85810, 87050, 88290,
     89530, 90770, 92010, 93260, 94500, 95740, 96980, 98220, 99460, 100700},
    {55560, 57130, 58700, 60280, 61850, 63430, 65000, 66910, 68820, 70730, 72640,
     74540, 76120, 77690, 79270, 80840, 82080, 83320, 84570, 85810, 87050, 88290,
     89530, 90770, 92010, 93260, 94500, 95740, 96980, 98220, 99460, 100700},
    {111110, 112690, 114260, 115840, 117410, 118980, 120560, 122470, 124380, 126280, 128190,
     130100, 131670, 133250, 134820, 136400, 137640, 138880, 140120, 141360, 142600, 143850,
     145090, 146330, 147570, 148810, 150050, 151290, 152540, 153780, 155020, 156260},
    {111110, 112690, 114260, 115840, 117410, 118980, 120560, 122470, 124380, 126280, 128190,
     130100, 131670, 133250, 134820, 136400, 137640, 138880, 140120, 141360, 142600, 143850,
     145090, 146330, 147570, 148810, 150050, 151290, 152540, 153780, 155020, 156260},
    {166670, 168240, 169820, 171390, 172970, 174540, 176110, 178020, 179930, 181840, 183750,
     185650, 187230, 188800, 190380, 191950, 193190, 194440, 195680, 196920, 198160, 199400,
     200640, 201880, 203130, 204370, 205610, 206850, 208090, 209330, 210570, 211810},
    {166670, 168240, 169820, 171390, 172970, 174540, 176110, 178020, 179930, 181840, 183750,
     185650, 187230, 188800, 190380, 191950, 193190, 194440, 195680, 196920, 198160, 199400,
     200640, 201880, 203130, 204370, 205610, 206850, 208090, 209330, 210570, 211810},
    {222220, 223800, 225370, 226950, 228520, 230100, 231670, 233580, 235490, 237390, 239300,
     241210, 242780, 244360, 245930, 247510, 248750, 249990, 251230, 252470, 253720, 254960,
     256200, 257440, 258680, 259920, 261160, 262400, 263650, 264890, 266130, 267370},
    {222220, 223800, 225370, 226950, 228520, 230100, 231670, 233580, 235490, 237390, 239300,
     241210, 242780, 244360, 245930, 247510, 248750, 249990, 251230, 252470, 253720, 254960,
     256200, 257440, 258680, 259920, 261160, 262400, 263650, 264890, 266130, 267370},
    {277780, 279350, 280930, 282500, 284080, 285650, 287230, 289130, 291040, 292950, 294860,
     296770, 298340, 299920, 301490, 303060, 304310, 305550, 306790, 308030, 309270, 310510,
     311750, 313000, 314240, 315480, 316720, 317960, 319200, 320440, 321680, 322930},
    {277780, 279350, 280930, 282500, 284080, 285650, 287230, 289130, 291040, 292950, 294860,
     296770, 298340, 299920, 301490, 303060, 304310, 305550, 306790, 308030, 309270, 310510,
     311750, 313000, 314240, 315480, 316720, 317960, 319200, 320440, 321680, 322930}};

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
  static constexpr float min_range = 0.2f;
  static constexpr float max_range = 150.f;
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

  ReturnMode get_return_mode(const robosense_packet::helios::InfoPacket & info_packet) override
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
    const robosense_packet::helios::InfoPacket & info_packet) override
  {
    return info_packet.sensor_calibration.get_calibration();
  }

  bool get_sync_status(const robosense_packet::helios::InfoPacket & info_packet) override
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
    const robosense_packet::helios::InfoPacket & info_packet) override
  {
    std::map<std::string, std::string> sensor_info;
    sensor_info["motor_speed"] = std::to_string(info_packet.motor_speed.value());
    sensor_info["lidar_ip"] = info_packet.ethernet.lidar_ip.to_string();
    sensor_info["dest_pc_ip"] = info_packet.ethernet.dest_pc_ip.to_string();
    sensor_info["mac_addr"] = info_packet.ethernet.mac_addr.to_string();
    sensor_info["lidar_out_msop_port"] =
      std::to_string(info_packet.ethernet.lidar_out_msop_port.value());
    sensor_info["pc_dest_msop_port"] =
      std::to_string(info_packet.ethernet.pc_dest_msop_port.value());
    sensor_info["lidar_out_difop_port"] =
      std::to_string(info_packet.ethernet.lidar_out_difop_port.value());
    sensor_info["pc_dest_difop_port"] =
      std::to_string(info_packet.ethernet.pc_dest_difop_port.value());
    sensor_info["fov_start"] =
      robosense_packet::get_float_value(info_packet.fov_setting.fov_start.value());
    sensor_info["fov_end"] =
      robosense_packet::get_float_value(info_packet.fov_setting.fov_end.value());
    sensor_info["phase_lock"] = std::to_string(info_packet.phase_lock.value());
    sensor_info["top_firmware_version"] = info_packet.top_firmware_version.to_string();
    sensor_info["bottom_firmware_version"] = info_packet.bottom_firmware_version.to_string();
    sensor_info["bottom_software_version"] = info_packet.bottom_software_version.to_string();
    sensor_info["motor_firmware_version"] = info_packet.motor_firmware_version.to_string();
    sensor_info["sensor_hw_version"] = info_packet.sensor_hw_version.to_string();
    sensor_info["web_page_version"] = info_packet.web_page_version.to_string();
    sensor_info["top_backup_crc"] = std::to_string(info_packet.top_backup_crc.value());
    sensor_info["bottom_backup_crc"] = std::to_string(info_packet.bottom_backup_crc.value());
    sensor_info["software_backup_crc"] = std::to_string(info_packet.software_backup_crc.value());
    sensor_info["webpage_backup_crc"] = std::to_string(info_packet.webpage_backup_crc.value());
    sensor_info["ethernet_gateway"] = info_packet.ethernet_gateway.to_string();
    sensor_info["subnet_mask"] = info_packet.subnet_mask.to_string();
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

    /*
     * From the manual, here are the formulas for calculating following values:
     *
     * Idat = Value_temp / 4096 * 5 [A]
     * Vdat = value / 4096 [V]
     * Vdat_12V_reg = value / 4096 * 24.5 [V]
     * Vdat_5V_reg = value / 4096 x 11 [V]
     * Vdat_2V5_reg = value / 4096 x 10 [V]
     * Vdat_APD = 516.65 * (value) / 4096 - 465.8 [V](negative value)
     * Temperature 2&3&4 = 200 *（value）/ 4096 - 50
     * Temperature 1&5 = 503.975 * (value) / 4096 - 273.15
     */
    sensor_info["time"] = std::to_string(info_packet.time.get_time_in_ns());
    sensor_info["i_dat"] = std::to_string(info_packet.operating_status.i_dat.value() / 4096.0 * 5);
    sensor_info["v_dat"] = std::to_string(info_packet.operating_status.v_dat.value() / 4096.0);
    sensor_info["v_dat_12v"] =
      std::to_string(info_packet.operating_status.v_dat_12v.value() / 4096.0 * 24.5);
    sensor_info["v_dat_5v"] =
      std::to_string(info_packet.operating_status.v_dat_5v.value() / 4096.0 * 11);
    sensor_info["v_dat_2v5"] =
      std::to_string(info_packet.operating_status.v_dat_2v5.value() / 4096.0 * 10);
    sensor_info["v_dat_apd"] =
      std::to_string(info_packet.operating_status.v_dat_apd.value() * 516.65 / 4096 - 465.8);
    sensor_info["top_board_temp"] =
      std::to_string(info_packet.fault_diagnosis.temperature1.value() * 503.975 / 4096.0 - 273.15);
    sensor_info["temperature2"] =
      std::to_string(info_packet.fault_diagnosis.temperature2.value() * 200 / 4096.0 - 50);
    sensor_info["temperature3"] =
      std::to_string(info_packet.fault_diagnosis.temperature3.value() * 200 / 4096.0 - 50);
    sensor_info["temperature4"] =
      std::to_string(info_packet.fault_diagnosis.temperature4.value() * 200 / 4096.0 - 50);
    sensor_info["bottom_board_temp"] =
      std::to_string(info_packet.fault_diagnosis.temperature5.value() * 503.975 / 4096.0 - 273.15);
    sensor_info["real_time_rot_speed"] = std::to_string(info_packet.fault_diagnosis.r_rpm.value());
    sensor_info["lane_up"] = std::to_string(info_packet.fault_diagnosis.lane_up.value());
    sensor_info["lane_up_cnt"] = std::to_string(info_packet.fault_diagnosis.lane_up_cnt.value());
    sensor_info["top_status"] = std::to_string(info_packet.fault_diagnosis.top_status.value());

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

    sensor_info["code_wheel_status"] = std::to_string(info_packet.code_wheel_status.value());
    sensor_info["pps_trigger_mode"] = std::to_string(info_packet.pps_trigger_mode.value());

    std::string gprmc_string;
    for (auto i : info_packet.gprmc) {
      gprmc_string += static_cast<char>(i.value());
    }
    sensor_info["gprmc_string"] = gprmc_string;

    return sensor_info;
  }
};

}  // namespace nebula::drivers
