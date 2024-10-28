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

#ifndef HESAI_CMD_RESPONSE_HPP
#define HESAI_CMD_RESPONSE_HPP

#include <boost/algorithm/string/join.hpp>
#include <boost/endian/buffers.hpp>
#include <boost/format.hpp>

#include <array>
#include <cstdint>
#include <ostream>
#include <queue>
#include <sstream>
#include <string>
#include <nlohmann/json.hpp>

using namespace boost::endian;  // NOLINT(build/namespaces)
namespace nebula
{
using nlohmann::json;

#pragma pack(push, 1)

/// @brief PTP STATUS struct of PTC_COMMAND_PTP_DIAGNOSTICS
struct HesaiPtpDiagStatus
{
  big_int64_buf_t master_offset;
  big_int32_buf_t ptp_state;
  big_int32_buf_t elapsed_millisec;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpDiagStatus const & arg)
  {
    os << "master_offset: " << arg.master_offset;
    os << ", ";
    os << "ptp_state: " << arg.ptp_state;
    os << ", ";
    os << "elapsed_millisec: " << arg.elapsed_millisec;

    return os;
  }
};

/// @brief PTP TLV PORT_DATA_SET struct of PTC_COMMAND_PTP_DIAGNOSTICS
struct HesaiPtpDiagPort
{
  uint8_t portIdentity[10];
  big_int32_buf_t portState;
  big_int32_buf_t logMinDelayReqInterval;
  big_int64_buf_t peerMeanPathDelay;
  big_int32_buf_t logAnnounceInterval;
  big_int32_buf_t announceReceiptTimeout;
  big_int32_buf_t logSyncInterval;
  big_int32_buf_t delayMechanism;
  big_int32_buf_t logMinPdelayReqInterval;
  big_int32_buf_t versionNumber;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpDiagPort const & arg)
  {
    os << "portIdentity: " << std::string(std::begin(arg.portIdentity), std::end(arg.portIdentity));
    os << ", ";
    os << "portState: " << arg.portState;
    os << ", ";
    os << "logMinDelayReqInterval: " << arg.logMinDelayReqInterval;
    os << ", ";
    os << "peerMeanPathDelay: " << arg.peerMeanPathDelay;
    os << ", ";
    os << "logAnnounceInterval: " << arg.logAnnounceInterval;
    os << ", ";
    os << "announceReceiptTimeout: " << arg.announceReceiptTimeout;
    os << ", ";
    os << "logSyncInterval: " << arg.logSyncInterval;
    os << ", ";
    os << "delayMechanism: " << arg.delayMechanism;
    os << ", ";
    os << "logMinPdelayReqInterval: " << arg.logMinPdelayReqInterval;
    os << ", ";
    os << "versionNumber: " << arg.versionNumber;

    return os;
  }
};

/// @brief LinuxPTP TLV TIME_STATUS_NP struct of PTC_COMMAND_PTP_DIAGNOSTICS
struct HesaiPtpDiagTime
{
  big_int64_buf_t master_offset;
  big_int64_buf_t ingress_time;
  big_int32_buf_t cumulativeScaledRateOffset;
  big_int32_buf_t scaledLastGmPhaseChange;
  big_int16_buf_t gmTimeBaseIndicator;
  uint8_t lastGmPhaseChange[12];
  big_int32_buf_t gmPresent;
  big_int64_buf_t gmIdentity;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpDiagTime const & arg)
  {
    os << "master_offset: " << arg.master_offset;
    os << ", ";
    os << "ingress_time: " << arg.ingress_time;
    os << ", ";
    os << "cumulativeScaledRateOffset: " << arg.cumulativeScaledRateOffset;
    os << ", ";
    os << "scaledLastGmPhaseChange: " << arg.scaledLastGmPhaseChange;
    os << ", ";
    os << "gmTimeBaseIndicator: " << arg.gmTimeBaseIndicator;
    os << ", ";
    // FIXME: lastGmPhaseChange is a binary number, displaying it as string is incorrect
    os << "lastGmPhaseChange: "
       << std::string(std::begin(arg.lastGmPhaseChange), std::end(arg.lastGmPhaseChange));
    os << ", ";
    os << "gmPresent: " << arg.gmPresent;
    os << ", ";
    os << "gmIdentity: " << arg.gmIdentity;

    return os;
  }
};

/// @brief LinuxPTP TLV GRANDMASTER_SETTINGS_NP struct of PTC_COMMAND_PTP_DIAGNOSTICS
struct HesaiPtpDiagGrandmaster
{
  big_int32_buf_t clockQuality;
  big_int16_buf_t utc_offset;
  int8_t time_flags;
  int8_t time_source;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpDiagGrandmaster const & arg)
  {
    os << "clockQuality: " << arg.clockQuality;
    os << ", ";
    os << "utc_offset: " << arg.utc_offset;
    os << ", ";
    os << "time_flags: " << +arg.time_flags;
    os << ", ";
    os << "time_source: " << +arg.time_source;

    return os;
  }
};

/// @brief struct of PTC_COMMAND_GET_INVENTORY_INFO
struct HesaiInventory
{
  char sn[18];
  char date_of_manufacture[16];
  uint8_t mac[6];
  char sw_ver[16];
  char hw_ver[16];
  char control_fw_ver[16];
  char sensor_fw_ver[16];
  big_uint16_buf_t angle_offset;
  uint8_t model;
  uint8_t motor_type;
  uint8_t num_of_lines;
  uint8_t reserved[11];

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiInventory const & arg)
  {
    std::ios initial_format(nullptr);
    initial_format.copyfmt(os);

    os << "sn: " << std::string(arg.sn, strnlen(arg.sn, sizeof(arg.sn)));
    os << ", ";
    os << "date_of_manufacture: "
       << std::string(
            arg.date_of_manufacture,
            strnlen(arg.date_of_manufacture, sizeof(arg.date_of_manufacture)));
    os << ", ";
    os << "mac: ";

    for (size_t i = 0; i < sizeof(arg.mac); i++) {
      if (i != 0) {
        os << ':';
      }
      os << std::hex << std::setfill('0') << std::setw(2) << (+arg.mac[i]);
    }
    os.copyfmt(initial_format);

    os << ", ";
    os << "sw_ver: " << std::string(arg.sw_ver, strnlen(arg.sw_ver, sizeof(arg.sw_ver)));
    os << ", ";
    os << "hw_ver: " << std::string(arg.hw_ver, strnlen(arg.hw_ver, sizeof(arg.hw_ver)));
    os << ", ";
    os << "control_fw_ver: "
       << std::string(arg.control_fw_ver, strnlen(arg.control_fw_ver, sizeof(arg.control_fw_ver)));
    os << ", ";
    os << "sensor_fw_ver: "
       << std::string(arg.sensor_fw_ver, strnlen(arg.sensor_fw_ver, sizeof(arg.sensor_fw_ver)));
    os << ", ";
    os << "angle_offset: " << arg.angle_offset;
    os << ", ";
    os << "model: " << +arg.model;
    os << ", ";
    os << "motor_type: " << +arg.motor_type;
    os << ", ";
    os << "num_of_lines: " << +arg.num_of_lines;
    os << ", ";

    for (size_t i = 0; i < sizeof(arg.reserved); i++) {
      if (i != 0) {
        os << ' ';
      }
      os << std::hex << std::setfill('0') << std::setw(2) << (+arg.reserved[i]);
    }
    os.copyfmt(initial_format);

    return os;
  }

  std::string get_str_model()
  {
    switch (model) {
      case 0:
        return "Pandar40P";
      case 2:
        return "Pandar64";
      case 3:
        return "Pandar128";
      case 15:
        return "PandarQT";
      case 17:
        return "Pandar40M";
      case 20:
        return "PandarMind(PM64)";
      case 25:
        return "PandarXT32";
      case 26:
        return "PandarXT16";
      case 32:
        return "QT128C2X";
      case 38:
        return "PandarXT32M";
      case 42:
        return "OT128";
      case 48:
        return "PandarAT128";
      default:
        return "Unknown(" + std::to_string(static_cast<int>(model)) + ")";
    }
  }
};

/// @brief struct of PTC_COMMAND_GET_CONFIG_INFO
struct HesaiConfigBase
{
  struct Internal{
    uint8_t ipaddr[4];
    uint8_t mask[4];
    uint8_t gateway[4];
    uint8_t dest_ipaddr[4];
    big_uint16_buf_t dest_LiDAR_udp_port;
    big_uint16_buf_t dest_gps_udp_port;
    big_uint16_buf_t spin_rate;
    uint8_t sync;
    big_uint16_buf_t sync_angle;
    big_uint16_buf_t start_angle;
    big_uint16_buf_t stop_angle;
    uint8_t clock_source;
    uint8_t udp_seq;
    uint8_t trigger_method;
    uint8_t return_mode;
    uint8_t standby_mode;
    uint8_t motor_status;
    uint8_t vlan_flag;
    big_uint16_buf_t vlan_id;
  };

  json to_json()
  {
    json j;
    {
      std::stringstream ss;
      ss << static_cast<int>(get().ipaddr[0]) << "."
        << static_cast<int>(get().ipaddr[1]) << "."
        << static_cast<int>(get().ipaddr[2]) << "."
        << static_cast<int>(get().ipaddr[3]);
      j["ipaddr"] = ss.str();
    }
    {
      std::stringstream ss;
      ss << static_cast<int>(get().mask[0]) << "."
        << static_cast<int>(get().mask[1]) << "."
        << static_cast<int>(get().mask[2]) << "."
        << static_cast<int>(get().mask[3]);
      j["mask"] = ss.str();
    }
    {
      std::stringstream ss;
      ss << static_cast<int>(get().gateway[0]) << "."
        << static_cast<int>(get().gateway[1]) << "."
        << static_cast<int>(get().gateway[2]) << "."
        << static_cast<int>(get().gateway[3]);
      j["gateway"] = ss.str();
    }
    {
      std::stringstream ss;
      ss << static_cast<int>(get().dest_ipaddr[0]) << "."
        << static_cast<int>(get().dest_ipaddr[1]) << "."
        << static_cast<int>(get().dest_ipaddr[2]) << "."
        << static_cast<int>(get().dest_ipaddr[3]);
      j["dest_ipaddr"] = ss.str();
    }
    j["dest_LiDAR_udp_port"] = get().dest_LiDAR_udp_port.value();
    j["dest_gps_udp_port"] = get().dest_gps_udp_port.value();
    j["spin_rate"] = std::to_string(get().spin_rate.value()) + " RPM";
    j["sync"] = get().sync;
    j["sync_angle"] = std::to_string(get().sync * 0.01) + " deg";
    j["start_angle"] = get().start_angle.value();
    j["stop_angle"] = get().stop_angle.value();
    j["clock_source"] = get().clock_source;
    j["udp_seq"] = get().udp_seq;
    j["trigger_method"] = get().trigger_method;
    j["return_mode"] = get().return_mode;
    j["standby_mode"] = get().standby_mode;
    j["motor_status"] = get().motor_status;
    j["vlan_flag"] = get().vlan_flag;
    j["vlan_id"] = get().vlan_id.value();
    j.update(sensor_specifics_to_json());

    return j;
  }

  [[nodiscard]] virtual const Internal & get() const = 0;

  protected:
    virtual json sensor_specifics_to_json() = 0;
};

inline std::ostream & operator<<(std::ostream & os, HesaiConfigBase & arg)
{
  for (auto & [key, value] : arg.to_json().items()) {
    os << key << " : " << value << std::endl;
  }
  return os;
}

struct HesaiConfig_OT128_AT128: public HesaiConfigBase
{
  struct Internal : HesaiConfigBase::Internal{
    uint8_t gps_nmea_sentence;
    uint8_t noise_filtering;
    uint8_t reflectivity_mapping;
    unsigned char reserved[6];
  };

  HesaiConfig_OT128_AT128(Internal value) : value(value) {}
  [[nodiscard]] const HesaiConfigBase::Internal & get() const override
  {
    return value;
  }
  json sensor_specifics_to_json() override
  {
    json j;
    j["gps_nmea_sentence"] = value.gps_nmea_sentence;
    j["noise_filtering"] = value.noise_filtering;
    j["reflectivity_mapping"] = value.reflectivity_mapping;

    return j;
  }
  private:
    Internal value;
};

inline std::ostream & operator<<(std::ostream & os, HesaiConfig_OT128_AT128 & arg)
{
  for (auto & [key, value] : arg.to_json().items()) {
    os << key << " : " << value << std::endl;
  }
  return os;
}

struct HesaiConfig_XT_40p: public HesaiConfigBase
{
  struct Internal : HesaiConfigBase::Internal{
    uint8_t clock_data_fmt;
    uint8_t noise_filtering;
    uint8_t reflectivity_mapping;
    unsigned char reserved[6];
  };

  HesaiConfig_XT_40p(Internal value) : value(value) {}
  [[nodiscard]] const HesaiConfigBase::Internal & get() const override
  {
    return value;
  }
  json sensor_specifics_to_json() override
  {
    json j;
    j["clock_data_fmt"] = value.clock_data_fmt;
    j["noise_filtering"] = value.noise_filtering;
    j["reflectivity_mapping"] = value.reflectivity_mapping;

    return j;
  }
  private:
    Internal value;
};

inline std::ostream & operator<<(std::ostream & os, HesaiConfig_XT_40p & arg)
{
  for (auto & [key, value] : arg.to_json().items()) {
    os << key << " : " << value << std::endl;
  }
  return os;
}

/// @brief struct of PTC_COMMAND_GET_LIDAR_STATUS
struct HesaiLidarStatusBase
{
  struct Internal{
    big_uint32_buf_t system_uptime;
    big_uint16_buf_t motor_speed;
  };

  json to_json()
  {
    json j;
    j["system_uptime"] = std::to_string(get().system_uptime.value()) + " s";
    j["motor_speed"] = std::to_string(get().motor_speed.value()) + " RPM";
    j.update(sensor_specifics_to_json());

    return j;
  }
  
 [[nodiscard]] virtual const Internal & get() const = 0;

  protected:
    virtual json sensor_specifics_to_json() = 0;

  std::string get_str_gps_pps_lock(uint8_t value)
  {
    switch (value) {
      case 1:
        return "Lock";
      case 0:
        return "Unlock";
      default:
        return "Unknown";
    }
  }
  std::string get_str_gps_gprmc_status(uint8_t value)
  {
    switch (value) {
      case 1:
        return "Lock";
      case 0:
        return "Unlock";
      default:
        return "Unknown";
    }
  }
  std::string get_str_ptp_clock_status(uint8_t value)
  {
    switch (value) {
      case 0:
        return "free run";
      case 1:
        return "tracking";
      case 2:
        return "locked";
      case 3:
        return "frozen";
      default:
        return "Unknown";
    }
  }
};

inline std::ostream & operator<<(std::ostream & os, HesaiLidarStatusBase const & arg)
{
  os << "system_uptime: " << arg.get().system_uptime << "\n";
  os << "motor_speed: " << arg.get().motor_speed;
  
  return os;
}

struct HesaiLidarStatusAT128: public HesaiLidarStatusBase
{
  struct Internal: public HesaiLidarStatusBase::Internal{
    big_int32_buf_t temperature[9];
    uint8_t gps_pps_lock;
    uint8_t gps_gprmc_status;
    big_int32_buf_t startup_times;
    big_int32_buf_t total_operation_time;
    uint8_t ptp_status;
    unsigned char reserved[1];
  };
  HesaiLidarStatusAT128(Internal value) : value(value) {}
  json sensor_specifics_to_json() override
  {
    json j;
    json temperature;
    temperature["tx1 temperature"] = std::to_string(value.temperature[0].value() * 0.01) + " deg";
    temperature["tx2 temperature"] = std::to_string(value.temperature[1].value() * 0.01) + " deg";
    temperature["fpga temperature"] = std::to_string(value.temperature[2].value() * 0.01) + " deg";
    temperature["rx1 temperature"] = std::to_string(value.temperature[3].value() * 0.01) + " deg";
    temperature["rx2 temperature"] = std::to_string(value.temperature[4].value() * 0.01) + " deg";
    temperature["mb1 temperature"] = std::to_string(value.temperature[5].value() * 0.01) + " deg";
    temperature["mb2 temperature"] = std::to_string(value.temperature[6].value() * 0.01) + " deg";
    temperature["pb temperature"] = std::to_string(value.temperature[7].value() * 0.01) + " deg";
    temperature["hot temperature"] = std::to_string(value.temperature[8].value() * 0.01) + " deg";
    j["temperature"] = temperature;
    j["gps_pps_lock"] = get_str_gps_pps_lock(value.gps_pps_lock);
    j["gps_gprmc_status"] = get_str_gps_gprmc_status(value.gps_gprmc_status);
    j["startup_times"] = value.startup_times.value();
    j["total_operation_time"] = std::to_string(value.total_operation_time.value()) + " min";
    j["ptp_status"] = get_str_ptp_clock_status(value.ptp_status);
    j["reserved"] = value.reserved;

    return j;
  }
  [[nodiscard]] const HesaiLidarStatusBase::Internal & get() const override
  {
    return value;
  }
  private:
    Internal value;
};

inline std::ostream & operator<<(std::ostream & os, HesaiLidarStatusAT128 & arg)
{
  for (auto & [key, value] : arg.to_json().items()) {
    os << key << " : " << value << std::endl;
  }  
  return os;
}

struct HesaiLidarStatusOT128: public HesaiLidarStatusBase
{
  struct Internal: public HesaiLidarStatusBase::Internal{
    big_int32_buf_t temperature[8];
    uint8_t gps_pps_lock;
    uint8_t gps_gprmc_status;
    big_int32_buf_t startup_times;
    big_int32_buf_t total_operation_time;
    uint8_t ptp_status;
    big_uint32_buf_t humidity;
    unsigned char reserved[1];
  };
  HesaiLidarStatusOT128(Internal value) : value(value) {}
  [[nodiscard]] const HesaiLidarStatusBase::Internal & get() const override
  {
    return value;
  }
  json sensor_specifics_to_json() override
  {
    json j;
    json temperature;
    temperature["Bottom circuit board T1 temperature"] = std::to_string(value.temperature[0].value() * 0.01) + " deg";
    temperature["Bottom circuit board T2 temperature"] = std::to_string(value.temperature[1].value() * 0.01) + " deg";
    temperature["Laser emitting board RT_L1 temperature"] = std::to_string(value.temperature[2].value() * 0.01) + " deg";
    temperature["Laser emitting board RT_L2 temperature"] = std::to_string(value.temperature[3].value() * 0.01) + " deg";
    temperature["Laser Receiving board RT_R temperature"] = std::to_string(value.temperature[4].value() * 0.01) + " deg";
    temperature["Laser Receiving board RT2 temperature"] = std::to_string(value.temperature[5].value() * 0.01) + " deg";
    temperature["Top circuit RT3 temperature"] = std::to_string(value.temperature[6].value() * 0.01) + " deg";
    temperature["Top circuit RT4 temperature"] = std::to_string(value.temperature[7].value() * 0.01) + " deg";
    j["temperature"] = temperature;
    j["gps_pps_lock"] = get_str_gps_pps_lock(value.gps_pps_lock);
    j["gps_gprmc_status"] = get_str_gps_gprmc_status(value.gps_gprmc_status);
    j["startup_times"] = value.startup_times.value();
    j["total_operation_time"] = std::to_string(value.total_operation_time.value()) + " min";
    j["ptp_status"] = get_str_ptp_clock_status(value.ptp_status);
    j["humidity"] = std::to_string(value.humidity.value() * 0.1) + " %";
    j["reserved"] = value.reserved;

    return j;
  }
  private:
    Internal value;
};

inline std::ostream & operator<<(std::ostream & os, HesaiLidarStatusOT128 & arg)
{
  for (auto & [key, value] : arg.to_json().items()) {
    os << key << " : " << value << std::endl;
  }
  return os;
}

struct HesaiLidarStatus_XT_40p: public HesaiLidarStatusBase
{
  struct Internal: public HesaiLidarStatusBase::Internal{
    big_int32_buf_t temperature[8];
    uint8_t gps_pps_lock;
    uint8_t gps_gprmc_status;
    big_int32_buf_t startup_times;
    big_int32_buf_t total_operation_time;
    uint8_t ptp_status;
    unsigned char reserved[5];
  };
  HesaiLidarStatus_XT_40p(Internal value) : value(value) {}
  [[nodiscard]] const HesaiLidarStatusBase::Internal & get() const override
  {
    return value;
  }
  json sensor_specifics_to_json() override
  {
    json j;
    json temperature;
    temperature["Bottom circuit board T1 temperature"] = std::to_string(value.temperature[0].value() * 0.01) + " deg";
    temperature["Bottom circuit board T2 temperature"] = std::to_string(value.temperature[1].value() * 0.01) + " deg";
    temperature["Laser emitting board RT_L temperature"] = std::to_string(value.temperature[2].value() * 0.01) + " deg";
    temperature["Laser emitting board RT_R temperature"] = std::to_string(value.temperature[3].value() * 0.01) + " deg";
    temperature["Laser Receiving board RT2 temperature"] = std::to_string(value.temperature[4].value() * 0.01) + " deg";
    temperature["Top circult RT3 temperature"] = std::to_string(value.temperature[5].value() * 0.01) + " deg";
    temperature["Top circuit RT4 temperature"] = std::to_string(value.temperature[6].value() * 0.01) + " deg";
    temperature["Top circuit RT5 temperature"] = std::to_string(value.temperature[7].value() * 0.01) + " deg";
    j["temperature"] = temperature;
    j["gps_pps_lock"] = get_str_gps_pps_lock(value.gps_pps_lock);
    j["gps_gprmc_status"] = get_str_gps_gprmc_status(value.gps_gprmc_status);
    j["startup_times"] = value.startup_times.value();
    j["total_operation_time"] = std::to_string(value.total_operation_time.value()) + " min";
    j["ptp_status"] = get_str_ptp_clock_status(value.ptp_status);
    j["reserved"] = value.reserved;

    return j;
  }
  private:
    Internal value;
};

inline std::ostream & operator<<(std::ostream & os, HesaiLidarStatus_XT_40p & arg)
{
  for (auto & [key, value] : arg.to_json().items()) {
    os << key << " : " << value << std::endl;
  }
  return os;
}

/// @brief struct of PTC_COMMAND_GET_LIDAR_RANGE
struct HesaiLidarRangeAll
{
  uint8_t method;
  big_uint16_buf_t start;
  big_uint16_buf_t end;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiLidarRangeAll const & arg)
  {
    os << "method: " << static_cast<int>(arg.method);
    os << ", ";
    os << "start: " << static_cast<int>(arg.start.value());
    os << ", ";
    os << "end: " << static_cast<int>(arg.end.value());

    return os;
  }
};

/// @brief struct of PTC_COMMAND_GET_PTP_CONFIG
struct HesaiPtpConfig
{
  int8_t status;
  int8_t profile;
  int8_t domain;
  int8_t network;
  int8_t logAnnounceInterval;
  int8_t logSyncInterval;
  int8_t logMinDelayReqInterval;
  // FIXME: this format is not correct for OT128, or for AT128 on 802.1AS

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpConfig const & arg)
  {
    os << "status: " << static_cast<int>(arg.status);
    os << ", ";
    os << "profile: " << static_cast<int>(arg.profile);
    os << ", ";
    os << "domain: " << static_cast<int>(arg.domain);
    os << ", ";
    os << "network: " << static_cast<int>(arg.network);
    if (arg.status == 0) {
      os << ", ";
      os << "logAnnounceInterval: " << static_cast<int>(arg.logAnnounceInterval);
      os << ", ";
      os << "logSyncInterval: " << static_cast<int>(arg.logSyncInterval);
      os << ", ";
      os << "logMinDelayReqInterval: " << static_cast<int>(arg.logMinDelayReqInterval);
    }
    return os;
  }
};

/// @brief struct of PTC_COMMAND_LIDAR_MONITOR
struct HesaiLidarMonitor_OT128
{
  // FIXME: this format is not correct for OT128
  big_int32_buf_t input_current;
  big_int32_buf_t input_voltage;
  big_int32_buf_t input_power;
  big_int32_buf_t phase_offset;
  uint8_t reserved[48];

  json to_json()
  {
    json j;
    j["input_current"] = std::to_string(input_current.value() * 0.01) + " mA";
    j["input_voltage"] = std::to_string(input_voltage.value() * 0.01) + " V";
    j["input_power"] = std::to_string(input_power.value() * 0.01) + " W";
    j["phase_offset"] = std::to_string(phase_offset.value() * 0.01) + " deg";
    j["reserved"] = reserved;

    return j;
  }
};


inline std::ostream & operator<<(std::ostream & os, HesaiLidarMonitor_OT128 const & arg)
{
  os << "input_current: " << arg.input_current;
  os << ", ";
  os << "input_voltage: " << arg.input_voltage;
  os << ", ";
  os << "input_power: " << arg.input_power;
  os << ", ";
  os << "phase_offset: " << arg.phase_offset;
  os << ", ";
  os << "reserved: ";
  for (size_t i = 0; i < sizeof(arg.reserved); i++) {
    if (i != 0) {
      os << ' ';
    }
    os << std::hex << std::setfill('0') << std::setw(2) << (static_cast<int>(arg.reserved[i]));
  }
  return os;
}

#pragma pack(pop)

}  // namespace nebula
#endif  // HESAI_CMD_RESPONSE_HPP
