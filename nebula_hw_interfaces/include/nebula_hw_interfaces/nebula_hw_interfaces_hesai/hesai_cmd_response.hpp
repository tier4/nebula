#ifndef HESAI_CMD_RESPONSE_HPP
#define HESAI_CMD_RESPONSE_HPP

#include <boost/algorithm/string/join.hpp>
#include <boost/format.hpp>

#include <ostream>
#include <iomanip>

namespace nebula
{
/// @brief PTP STATUS struct of PTC_COMMAND_PTP_DIAGNOSTICS
struct HesaiPtpDiagStatus
{
  long long master_offset;
  int ptp_state;
  int elapsed_millisec;

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
  std::vector<char> portIdentity = std::vector<char>(10);
  int portState;
  int logMinDelayReqInterval;
  long long peerMeanPathDelay;
  int logAnnounceInterval;
  int announceReceiptTimeout;
  int logSyncInterval;
  int delayMechanism;
  int logMinPdelayReqInterval;
  int versionNumber;

  HesaiPtpDiagPort() {}
  HesaiPtpDiagPort(const HesaiPtpDiagPort & arg)
  {
    std::copy(arg.portIdentity.begin(), arg.portIdentity.end(), portIdentity.begin());
    portState = arg.portState;
    logMinDelayReqInterval = arg.logMinDelayReqInterval;
    peerMeanPathDelay = arg.peerMeanPathDelay;
    logAnnounceInterval = arg.logAnnounceInterval;
    announceReceiptTimeout = arg.announceReceiptTimeout;
    logSyncInterval = arg.logSyncInterval;
    delayMechanism = arg.delayMechanism;
    logMinPdelayReqInterval = arg.logMinPdelayReqInterval;
    versionNumber = arg.versionNumber;
  }

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpDiagPort const & arg)
  {
    os << "portIdentity: " << std::string(arg.portIdentity.begin(), arg.portIdentity.end());
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
  long long master_offset;
  long long ingress_time;
  int cumulativeScaledRateOffset;
  int scaledLastGmPhaseChange;
  int gmTimeBaseIndicator;
  std::vector<char> lastGmPhaseChange = std::vector<char>(12);
  int gmPresent;
  long long gmIdentity;

  HesaiPtpDiagTime() {}
  HesaiPtpDiagTime(const HesaiPtpDiagTime & arg)
  {
    master_offset = arg.master_offset;
    ingress_time = arg.ingress_time;
    cumulativeScaledRateOffset = arg.cumulativeScaledRateOffset;
    scaledLastGmPhaseChange = arg.scaledLastGmPhaseChange;
    gmTimeBaseIndicator = arg.gmTimeBaseIndicator;
    std::copy(
      arg.lastGmPhaseChange.begin(), arg.lastGmPhaseChange.end(), lastGmPhaseChange.begin());
    gmPresent = arg.gmPresent;
    gmIdentity = arg.gmIdentity;
  }

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
    os << "lastGmPhaseChange: "
       << std::string(arg.lastGmPhaseChange.begin(), arg.lastGmPhaseChange.end());
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
  int clockQuality;
  int utc_offset;
  int time_flags;
  int time_source;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpDiagGrandmaster const & arg)
  {
    os << "clockQuality: " << arg.clockQuality;
    os << ", ";
    os << "utc_offset: " << arg.utc_offset;
    os << ", ";
    os << "time_flags: " << arg.time_flags;
    os << ", ";
    os << "time_source: " << arg.time_source;

    return os;
  }
};

/// @brief struct of PTC_COMMAND_GET_INVENTORY_INFO
struct HesaiInventory
{
  std::vector<char> sn = std::vector<char>(18);
  std::vector<char> date_of_manufacture = std::vector<char>(16);
  std::vector<char> mac = std::vector<char>(6);
  std::vector<char> sw_ver = std::vector<char>(16);
  std::vector<char> hw_ver = std::vector<char>(16);
  std::vector<char> control_fw_ver = std::vector<char>(16);
  std::vector<char> sensor_fw_ver = std::vector<char>(16);
  int angle_offset;
  int model;
  int motor_type;
  int num_of_lines;
  std::vector<unsigned char> reserved = std::vector<unsigned char>(11);

  HesaiInventory() {}
  HesaiInventory(const HesaiInventory & arg)
  {
    std::copy(arg.sn.begin(), arg.sn.end(), sn.begin());
    std::copy(
      arg.date_of_manufacture.begin(), arg.date_of_manufacture.end(), date_of_manufacture.begin());
    std::copy(arg.mac.begin(), arg.mac.end(), mac.begin());
    std::copy(arg.sw_ver.begin(), arg.sw_ver.end(), sw_ver.begin());
    std::copy(arg.hw_ver.begin(), arg.hw_ver.end(), hw_ver.begin());
    std::copy(arg.control_fw_ver.begin(), arg.control_fw_ver.end(), control_fw_ver.begin());
    std::copy(arg.sensor_fw_ver.begin(), arg.sensor_fw_ver.end(), sensor_fw_ver.begin());
    angle_offset = arg.angle_offset;
    model = arg.model;
    motor_type = arg.motor_type;
    num_of_lines = arg.num_of_lines;
    std::copy(arg.reserved.begin(), arg.reserved.end(), reserved.begin());
  }

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiInventory const & arg)
  {
    os << "sn: " << std::string(arg.sn.begin(), arg.sn.end());
    os << ", ";
    os << "date_of_manufacture: "
       << std::string(arg.date_of_manufacture.begin(), arg.date_of_manufacture.end());
    os << ", ";
    os << "mac: ";
    std::stringstream ss;
    for (long unsigned int i = 0; i < arg.mac.size() - 1; i++) {
      ss << std::hex << std::setfill('0') << std::setw(2) << (static_cast<int>(arg.mac[i]) & 0xff)
         << ":";
    }
    ss << std::hex << std::setfill('0') << std::setw(2)
       << (static_cast<int>(arg.mac[arg.mac.size() - 1]) & 0xff);
    os << ss.str();
    os << ", ";
    os << "sw_ver: " << std::string(arg.sw_ver.begin(), arg.sw_ver.end());
    os << ", ";
    os << "hw_ver: " << std::string(arg.hw_ver.begin(), arg.hw_ver.end());
    os << ", ";
    os << "control_fw_ver: " << std::string(arg.control_fw_ver.begin(), arg.control_fw_ver.end());
    os << ", ";
    os << "sensor_fw_ver: " << std::string(arg.sensor_fw_ver.begin(), arg.sensor_fw_ver.end());
    os << ", ";
    os << "angle_offset: " << arg.angle_offset;
    os << ", ";
    os << "model: " << arg.model;
    os << ", ";
    os << "motor_type: " << arg.motor_type;
    os << ", ";
    os << "num_of_lines: " << arg.num_of_lines;
    os << ", ";
    //      os << "reserved: " << boost::algorithm::join(arg.reserved, ",");
    os << "reserved: ";
    for (long unsigned int i = 0; i < arg.reserved.size() - 1; i++) {
      os << arg.reserved[i] << ",";
    }
    os << arg.reserved[arg.reserved.size() - 1];

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
        return "Unknown(" + std::to_string(model) + ")";
    }
  }
};

/// @brief struct of PTC_COMMAND_GET_CONFIG_INFO
struct HesaiConfig
{
  int ipaddr[4];
  int mask[4];
  int gateway[4];
  int dest_ipaddr[4];
  int dest_LiDAR_udp_port;
  int dest_gps_udp_port;
  int spin_rate;
  int sync;
  int sync_angle;
  int start_angle;
  int stop_angle;
  int clock_source;
  int udp_seq;
  int trigger_method;
  int return_mode;
  int standby_mode;
  int motor_status;
  int vlan_flag;
  int vlan_id;
  int clock_data_fmt;
  int noise_filtering;
  int reflectivity_mapping;
  unsigned char reserved[6];

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiConfig const & arg)
  {
    os << "ipaddr: " << arg.ipaddr[0] << "." << arg.ipaddr[1] << "." << arg.ipaddr[2] << "."
       << arg.ipaddr[3];
    os << ", ";
    os << "mask: " << arg.mask[0] << "." << arg.mask[1] << "." << arg.mask[2] << "." << arg.mask[3];
    os << ", ";
    os << "gateway: " << arg.gateway[0] << "." << arg.gateway[1] << "." << arg.gateway[2] << "."
       << arg.gateway[3];
    os << ", ";
    os << "dest_ipaddr: " << arg.dest_ipaddr[0] << "." << arg.dest_ipaddr[1] << "."
       << arg.dest_ipaddr[2] << "." << arg.dest_ipaddr[3];
    os << ", ";
    os << "dest_LiDAR_udp_port: " << arg.dest_LiDAR_udp_port;
    os << ", ";
    os << "dest_gps_udp_port: " << arg.dest_gps_udp_port;
    os << ", ";
    os << "spin_rate: " << arg.spin_rate;
    os << ", ";
    os << "sync: " << arg.sync;
    os << ", ";
    os << "sync_angle: " << arg.sync_angle;
    os << ", ";
    os << "start_angle: " << arg.start_angle;
    os << ", ";
    os << "stop_angle: " << arg.stop_angle;
    os << ", ";
    os << "clock_source: " << arg.clock_source;
    os << ", ";
    os << "udp_seq: " << arg.udp_seq;
    os << ", ";
    os << "trigger_method: " << arg.trigger_method;
    os << ", ";
    os << "return_mode: " << arg.return_mode;
    os << ", ";
    os << "standby_mode: " << arg.standby_mode;
    os << ", ";
    os << "motor_status: " << arg.motor_status;
    os << ", ";
    os << "vlan_flag: " << arg.vlan_flag;
    os << ", ";
    os << "vlan_id: " << arg.vlan_id;
    os << ", ";
    os << "clock_data_fmt: " << arg.clock_data_fmt;
    os << ", ";
    os << "noise_filtering: " << arg.noise_filtering;
    os << ", ";
    os << "reflectivity_mapping: " << arg.reflectivity_mapping;
    os << ", ";
    os << "reserved: " << arg.reserved[0] << "," << arg.reserved[1] << "," << arg.reserved[2] << ","
       << arg.reserved[3] << "," << arg.reserved[4] << "," << arg.reserved[5];

    return os;
  }
};

/// @brief struct of PTC_COMMAND_GET_LIDAR_STATUS
struct HesaiLidarStatus
{
  int system_uptime;
  int motor_speed;
  //    int temperature[8];
  std::vector<int> temperature = std::vector<int>(8);
  int gps_pps_lock;
  int gps_gprmc_status;
  int startup_times;
  int total_operation_time;
  int ptp_clock_status;
  std::vector<unsigned char> reserved = std::vector<unsigned char>(5);

  HesaiLidarStatus() {}
  HesaiLidarStatus(const HesaiLidarStatus & arg)
  {
    system_uptime = arg.system_uptime;
    motor_speed = arg.motor_speed;
    std::copy(arg.temperature.begin(), arg.temperature.end(), temperature.begin());
    gps_pps_lock = arg.gps_pps_lock;
    gps_gprmc_status = arg.gps_gprmc_status;
    startup_times = arg.startup_times;
    total_operation_time = arg.total_operation_time;
    ptp_clock_status = arg.ptp_clock_status;
    std::copy(arg.reserved.begin(), arg.reserved.end(), reserved.begin());
  }

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiLidarStatus const & arg)
  {
    os << "system_uptime: " << arg.system_uptime;
    os << ", ";
    os << "motor_speed: " << arg.motor_speed;
    os << ", ";
    os << "temperature: ";
    for (long unsigned int i = 0; i < arg.temperature.size() - 1; i++) {
      os << arg.temperature[i] << ",";
    }
    os << arg.temperature[arg.temperature.size() - 1];
    os << ", ";
    os << "gps_pps_lock: " << arg.gps_pps_lock;
    os << ", ";
    os << "gps_gprmc_status: " << arg.gps_gprmc_status;
    os << ", ";
    os << "startup_times: " << arg.startup_times;
    os << ", ";
    os << "total_operation_time: " << arg.total_operation_time;
    os << ", ";
    os << "ptp_clock_status: " << arg.ptp_clock_status;
    os << ", ";
    os << "reserved: ";
    for (long unsigned int i = 0; i < arg.reserved.size() - 1; i++) {
      os << arg.reserved[i] << ",";
    }
    os << arg.reserved[arg.reserved.size() - 1];

    return os;
  }

  std::string get_str_gps_pps_lock()
  {
    switch (gps_pps_lock) {
      case 1:
        return "Lock";
        break;
      case 0:
        return "Unlock";
        break;
      default:
        return "Unknown";
        break;
    }
  }
  std::string get_str_gps_gprmc_status()
  {
    switch (gps_gprmc_status) {
      case 1:
        return "Lock";
        break;
      case 0:
        return "Unlock";
        break;
      default:
        return "Unknown";
        break;
    }
  }
  std::string get_str_ptp_clock_status()
  {
    switch (ptp_clock_status) {
      case 0:
        return "free run";
        break;
      case 1:
        return "tracking";
        break;
      case 2:
        return "locked";
        break;
      case 3:
        return "frozen";
        break;
      default:
        return "Unknown";
        break;
    }
  }
};

/// @brief struct of PTC_COMMAND_GET_LIDAR_RANGE
struct HesaiLidarRangeAll
{
  int method;
  int start;
  int end;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiLidarRangeAll const & arg)
  {
    os << "method: " << arg.method;
    os << ", ";
    os << "start: " << arg.start;
    os << ", ";
    os << "end: " << arg.end;

    return os;
  }
};

/// @brief struct of PTC_COMMAND_GET_PTP_CONFIG
struct HesaiPtpConfig
{
  int status;
  int profile;
  int domain;
  int network;
  int logAnnounceInterval;
  int logSyncInterval;
  int logMinDelayReqInterval;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpConfig const & arg)
  {
    os << "status: " << arg.status;
    os << ", ";
    os << "profile: " << arg.profile;
    os << ", ";
    os << "domain: " << arg.domain;
    os << ", ";
    os << "network: " << arg.network;
    if (arg.status == 0) {
      os << ", ";
      os << "logAnnounceInterval: " << arg.logAnnounceInterval;
      os << ", ";
      os << "logSyncInterval: " << arg.logSyncInterval;
      os << ", ";
      os << "logMinDelayReqInterval: " << arg.logMinDelayReqInterval;
    }
    return os;
  }
};

/// @brief struct of PTC_COMMAND_LIDAR_MONITOR
struct HesaiLidarMonitor
{
  int input_voltage;
  int input_current;
  int input_power;
  std::vector<unsigned char> reserved = std::vector<unsigned char>(52);

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiLidarMonitor const & arg)
  {
    os << "input_voltage: " << arg.input_voltage;
    os << ", ";
    os << "input_current: " << arg.input_current;
    os << ", ";
    os << "input_power: " << arg.input_power;
    os << ", ";
    os << "reserved: ";
    for (long unsigned int i = 0; i < arg.reserved.size() - 1; i++) {
      os << arg.reserved[i] << ",";
    }
    os << arg.reserved[arg.reserved.size() - 1];

    return os;
  }
};

}  // namespace nebula
#endif  // HESAI_CMD_RESPONSE_HPP
