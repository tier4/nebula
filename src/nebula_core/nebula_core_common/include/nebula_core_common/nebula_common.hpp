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

#ifndef NEBULA_COMMON_H
#define NEBULA_COMMON_H

#include <nebula_core_common/point_types.hpp>

#include <boost/tokenizer.hpp>

#include <algorithm>
#include <ostream>
#include <string>

namespace nebula::drivers
{
/// @brief Return type of each scan
enum class ReturnType : uint8_t {
  UNKNOWN = 0,
  LAST,
  FIRST,
  STRONGEST,
  FIRST_WEAK,
  LAST_WEAK,
  IDENTICAL,
  SECOND,
  SECONDSTRONGEST,
  FIRST_STRONGEST,
  LAST_STRONGEST
};

/// @brief Return mode of each LiDAR
enum class ReturnMode : uint8_t {
  UNKNOWN = 0,
  SINGLE_STRONGEST,
  SINGLE_LAST,
  DUAL_FIRST,
  DUAL_LAST,
  DUAL_ONLY,
  SINGLE_FIRST,
  DUAL_STRONGEST_FIRST,
  DUAL_STRONGEST_LAST,
  DUAL_WEAK_FIRST,
  DUAL_WEAK_LAST,
  TRIPLE,
  LAST,
  STRONGEST,
  DUAL_LAST_STRONGEST,
  FIRST,
  DUAL_LAST_FIRST,
  DUAL_FIRST_STRONGEST,
  DUAL
};

/// @brief Convert ReturnType enum to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, nebula::drivers::ReturnType const & arg)
{
  switch (arg) {
    case ReturnType::UNKNOWN:
      os << "Unknown";
      break;
    case ReturnType::LAST:
      os << "Last";
      break;
    case ReturnType::FIRST:
      os << "First";
      break;
    case ReturnType::STRONGEST:
      os << "Strongest";
      break;
    case ReturnType::FIRST_WEAK:
      os << "FirstWeak";
      break;
    case ReturnType::LAST_WEAK:
      os << "LastWeak";
      break;
    case ReturnType::IDENTICAL:
      os << "Identical";
      break;
    case ReturnType::SECOND:
      os << "Second";
      break;
    case ReturnType::SECONDSTRONGEST:
      os << "SecondStrongest";
      break;
    case ReturnType::FIRST_STRONGEST:
      os << "FirstStrongest";
      break;
    case ReturnType::LAST_STRONGEST:
      os << "LastStrongest";
      break;
  }
  return os;
}

/// @brief Convert ReturnMode enum to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, nebula::drivers::ReturnMode const & arg)
{
  switch (arg) {
    case ReturnMode::SINGLE_FIRST:
      os << "SingleFirst";
      break;
    case ReturnMode::SINGLE_STRONGEST:
      os << "SingleStrongest";
      break;
    case ReturnMode::SINGLE_LAST:
      os << "SingleLast";
      break;
    case ReturnMode::DUAL_ONLY:
      os << "Dual";
      break;
    case ReturnMode::DUAL_FIRST:
      os << "DualFirst";
      break;
    case ReturnMode::DUAL_LAST:
      os << "DualLast";
      break;
    case ReturnMode::DUAL_WEAK_FIRST:
      os << "WeakFirst";
      break;
    case ReturnMode::DUAL_WEAK_LAST:
      os << "WeakLast";
      break;
    case ReturnMode::DUAL_STRONGEST_LAST:
      os << "StrongLast";
      break;
    case ReturnMode::DUAL_STRONGEST_FIRST:
      os << "StrongFirst";
      break;
    case ReturnMode::TRIPLE:
      os << "Triple";
      break;
    // for Hesai
    case ReturnMode::LAST:
      os << "Last";
      break;
    case ReturnMode::STRONGEST:
      os << "Strongest";
      break;
    case ReturnMode::DUAL_LAST_STRONGEST:
      os << "LastStrongest";
      break;
    case ReturnMode::FIRST:
      os << "First";
      break;
    case ReturnMode::DUAL_LAST_FIRST:
      os << "LastFirst";
      break;
    case ReturnMode::DUAL_FIRST_STRONGEST:
      os << "FirstStrongest";
      break;
    case ReturnMode::DUAL:
      os << "Dual";
      break;
    case ReturnMode::UNKNOWN:
      os << "Unknown";
      break;
  }
  return os;
}

// SENSOR_CONFIGURATION

/// @brief Type of sensor
enum class SensorModel : uint8_t {
  UNKNOWN = 0,
  HESAI_PANDAR64,
  HESAI_PANDAR40P,
  HESAI_PANDAR40M,
  HESAI_PANDARQT64,
  HESAI_PANDARQT128,
  HESAI_PANDARXT16,
  HESAI_PANDARXT32,
  HESAI_PANDARXT32M,
  HESAI_PANDARAT128,
  HESAI_PANDAR128_E3X,
  HESAI_PANDAR128_E4X,
  VELODYNE_VLS128,
  VELODYNE_HDL64,
  VELODYNE_VLP32,
  VELODYNE_VLP32MR,
  VELODYNE_HDL32,
  VELODYNE_VLP16,
  ROBOSENSE_HELIOS,
  ROBOSENSE_BPEARL_V3,
  ROBOSENSE_BPEARL_V4,
  CONTINENTAL_ARS548,
  CONTINENTAL_SRR520
};

enum class PtpProfile : uint8_t {
  IEEE_1588v2 = 0,
  IEEE_802_1AS,
  IEEE_802_1AS_AUTO,
  UNKNOWN_PROFILE
};

enum class PtpTransportType : uint8_t { UDP_IP = 0, L2, UNKNOWN_TRANSPORT };

enum class PtpSwitchType : uint8_t { NON_TSN = 0, TSN, UNKNOWN_SWITCH };

/// @brief Convert SensorModel enum to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, nebula::drivers::SensorModel const & arg)
{
  switch (arg) {
    case SensorModel::HESAI_PANDAR64:
      os << "Pandar64";
      break;
    case SensorModel::HESAI_PANDAR40P:
      os << "Pandar40P";
      break;
    case SensorModel::HESAI_PANDAR40M:
      os << "Pandar40M";
      break;
    case SensorModel::HESAI_PANDARQT64:
      os << "PandarQT64";
      break;
    case SensorModel::HESAI_PANDARQT128:
      os << "PandarQT128";
      break;
    case SensorModel::HESAI_PANDARXT16:
      os << "PandarXT16";
      break;
    case SensorModel::HESAI_PANDARXT32:
      os << "PandarXT32";
      break;
    case SensorModel::HESAI_PANDARXT32M:
      os << "PandarXT32M";
      break;
    case SensorModel::HESAI_PANDARAT128:
      os << "PandarAT128";
      break;
    case SensorModel::HESAI_PANDAR128_E3X:
      os << "Pandar128_E3X";
      break;
    case SensorModel::HESAI_PANDAR128_E4X:
      os << "Pandar128_E4X_OT";
      break;
    case SensorModel::VELODYNE_VLS128:
      os << "VLS128";
      break;
    case SensorModel::VELODYNE_HDL64:
      os << "HDL64";
      break;
    case SensorModel::VELODYNE_VLP32:
      os << "VLP32";
      break;
    case SensorModel::VELODYNE_VLP32MR:
      os << "VLP32MR";
      break;
    case SensorModel::VELODYNE_HDL32:
      os << "HDL32";
      break;
    case SensorModel::VELODYNE_VLP16:
      os << "VLP16";
      break;
    case SensorModel::ROBOSENSE_HELIOS:
      os << "HELIOS";
      break;
    case SensorModel::ROBOSENSE_BPEARL_V3:
      os << "BPEARL V3.0";
      break;
    case SensorModel::ROBOSENSE_BPEARL_V4:
      os << "BPEARL V4.0";
      break;
    case SensorModel::CONTINENTAL_ARS548:
      os << "ARS548";
      break;
    case SensorModel::CONTINENTAL_SRR520:
      os << "SRR520";
      break;
    case SensorModel::UNKNOWN:
      os << "Sensor Unknown";
      break;
  }
  return os;
}

/// @brief Base struct for Sensor configuration
struct SensorConfigurationBase
{
  SensorModel sensor_model;
  std::string frame_id;
};

/// @brief Base struct for Ethernet-based Sensor configuration
struct EthernetSensorConfigurationBase : public SensorConfigurationBase
{
  std::string host_ip{};
  std::string sensor_ip{};
  uint16_t data_port{};
};

/// @brief Base struct for CAN-based Sensor configuration
struct CANSensorConfigurationBase : public SensorConfigurationBase
{
  std::string interface;
  float receiver_timeout_sec{};
  float sender_timeout_sec{};
  /// @brief Socketcan filters, see the documentation of SocketCanReceiver::CanFilterList for
  /// details
  std::string filters;
  bool use_bus_time{};
};

/// @brief Base struct for Lidar configuration
struct LidarConfigurationBase : public EthernetSensorConfigurationBase
{
  ReturnMode return_mode;
  uint16_t packet_mtu_size;
  double min_range;
  double max_range;
  bool use_sensor_time{false};
};

/// @brief Convert SensorConfigurationBase to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, SensorConfigurationBase const & arg)
{
  os << "Sensor Model: " << arg.sensor_model << '\n';
  os << "Frame ID: " << arg.frame_id;
  return os;
}

/// @brief Convert EthernetSensorConfigurationBase to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, EthernetSensorConfigurationBase const & arg)
{
  os << static_cast<const SensorConfigurationBase &>(arg) << '\n';
  os << "Host IP: " << arg.host_ip << '\n';
  os << "Sensor IP: " << arg.sensor_ip << '\n';
  os << "Data Port: " << arg.data_port;
  return os;
}

/// @brief Convert CANSensorConfigurationBase to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, CANSensorConfigurationBase const & arg)
{
  os << static_cast<const SensorConfigurationBase &>(arg) << '\n';
  os << "Interface: " << arg.interface << '\n';
  os << "Receiver Timeout (s): " << arg.receiver_timeout_sec << '\n';
  os << "Sender Timeout (s): " << arg.sender_timeout_sec << '\n';
  os << "Filters: " << arg.filters << '\n';
  os << "Use Bus Time: " << arg.use_bus_time;
  return os;
}

/// @brief Convert LidarConfigurationBase to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(
  std::ostream & os, nebula::drivers::LidarConfigurationBase const & arg)
{
  os << static_cast<const EthernetSensorConfigurationBase &>(arg) << '\n';
  os << "Return Mode: " << arg.return_mode << '\n';
  os << "MTU: " << arg.packet_mtu_size << '\n';
  os << "Use Sensor Time: " << arg.use_sensor_time;
  return os;
}

/// @brief Base struct for Calibration configuration (Requires extensions in child struct)
struct CalibrationConfigurationBase
{
  std::string calibration_file;
};

/// @brief Convert sensor name to SensorModel enum (Upper and lower case letters must match)
/// @param sensor_model Sensor name (Upper and lower case letters must match)
/// @return Corresponding SensorModel
inline SensorModel sensor_model_from_string(const std::string & sensor_model)
{
  // Hesai
  if (sensor_model == "Pandar64") return SensorModel::HESAI_PANDAR64;
  if (sensor_model == "Pandar40P") return SensorModel::HESAI_PANDAR40P;
  if (sensor_model == "Pandar40M") return SensorModel::HESAI_PANDAR40M;
  if (sensor_model == "PandarXT16") return SensorModel::HESAI_PANDARXT16;
  if (sensor_model == "PandarXT32") return SensorModel::HESAI_PANDARXT32;
  if (sensor_model == "PandarXT32M") return SensorModel::HESAI_PANDARXT32M;
  if (sensor_model == "PandarAT128") return SensorModel::HESAI_PANDARAT128;
  if (sensor_model == "PandarQT64") return SensorModel::HESAI_PANDARQT64;
  if (sensor_model == "PandarQT128") return SensorModel::HESAI_PANDARQT128;
  if (sensor_model == "Pandar128E4X") return SensorModel::HESAI_PANDAR128_E4X;
  // Velodyne
  if (sensor_model == "VLS128") return SensorModel::VELODYNE_VLS128;
  if (sensor_model == "HDL64") return SensorModel::VELODYNE_HDL64;
  if (sensor_model == "VLP32") return SensorModel::VELODYNE_VLP32;
  if (sensor_model == "VLP32MR") return SensorModel::VELODYNE_VLP32MR;
  if (sensor_model == "HDL32") return SensorModel::VELODYNE_HDL32;
  if (sensor_model == "VLP16") return SensorModel::VELODYNE_VLP16;
  // Robosense
  if (sensor_model == "Helios") return SensorModel::ROBOSENSE_HELIOS;
  if (sensor_model == "Bpearl" || sensor_model == "Bpearl_V4")
    return SensorModel::ROBOSENSE_BPEARL_V4;
  if (sensor_model == "Bpearl_V3") return SensorModel::ROBOSENSE_BPEARL_V3;
  // Continental
  if (sensor_model == "ARS548") return SensorModel::CONTINENTAL_ARS548;
  if (sensor_model == "SRR520") return SensorModel::CONTINENTAL_SRR520;
  return SensorModel::UNKNOWN;
}

inline std::string sensor_model_to_string(const SensorModel & sensor_model)
{
  switch (sensor_model) {
    // Hesai
    case SensorModel::HESAI_PANDAR64:
      return "Pandar64";
    case SensorModel::HESAI_PANDAR40P:
      return "Pandar40P";
    case SensorModel::HESAI_PANDAR40M:
      return "Pandar40M";
    case SensorModel::HESAI_PANDARXT16:
      return "PandarXT16";
    case SensorModel::HESAI_PANDARXT32:
      return "PandarXT32";
    case SensorModel::HESAI_PANDARXT32M:
      return "PandarXT32M";
    case SensorModel::HESAI_PANDARAT128:
      return "PandarAT128";
    case SensorModel::HESAI_PANDARQT64:
      return "PandarQT64";
    case SensorModel::HESAI_PANDARQT128:
      return "PandarQT128";
    case SensorModel::HESAI_PANDAR128_E4X:
      return "Pandar128E4X";
    // Velodyne
    case SensorModel::VELODYNE_VLS128:
      return "VLS128";
    case SensorModel::VELODYNE_HDL64:
      return "HDL64";
    case SensorModel::VELODYNE_VLP32:
      return "VLP32";
    case SensorModel::VELODYNE_VLP32MR:
      return "VLP32MR";
    case SensorModel::VELODYNE_HDL32:
      return "HDL32";
    case SensorModel::VELODYNE_VLP16:
      return "VLP16";
    // Robosense
    case SensorModel::ROBOSENSE_HELIOS:
      return "Helios";
    case SensorModel::ROBOSENSE_BPEARL_V3:
      return "Bpearl_V3";
    case SensorModel::ROBOSENSE_BPEARL_V4:
      return "Bpearl_V4";
    // Continental
    case SensorModel::CONTINENTAL_ARS548:
      return "ARS548";
    case SensorModel::CONTINENTAL_SRR520:
      return "SRR520";
    default:
      return "UNKNOWN";
  }
}

/// @brief Convert return mode name to ReturnMode enum
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @return Corresponding ReturnMode
inline ReturnMode return_mode_from_string(const std::string & return_mode)
{
  if (return_mode == "SingleFirst") return ReturnMode::SINGLE_FIRST;
  if (return_mode == "SingleStrongest") return ReturnMode::SINGLE_STRONGEST;
  if (return_mode == "SingleLast") return ReturnMode::SINGLE_LAST;
  if (return_mode == "Dual") return ReturnMode::DUAL_ONLY;

  return ReturnMode::UNKNOWN;
}

[[maybe_unused]] pcl::PointCloud<PointXYZIR>::Ptr convert_point_xyziradt_to_point_xyzir(
  const pcl::PointCloud<PointXYZIRADT>::ConstPtr & input_pointcloud);

[[maybe_unused]] pcl::PointCloud<PointXYZIR>::Ptr convert_point_xyzircaedt_to_point_xyzir(
  const pcl::PointCloud<PointXYZIRCAEDT>::ConstPtr & input_pointcloud);

pcl::PointCloud<PointXYZIRADT>::Ptr convert_point_xyzircaedt_to_point_xyziradt(
  const pcl::PointCloud<PointXYZIRCAEDT>::ConstPtr & input_pointcloud, double stamp);

/// @brief Converts degrees to radians
/// @param radians
/// @return degrees
template <typename T>
static inline std::enable_if_t<std::is_floating_point_v<T>, T> deg2rad(T degrees)
{
  return degrees * static_cast<T>(M_PI / 180.0);
}

/// @brief Converts radians to degrees
/// @param radians
/// @return degrees
template <typename T>
static inline std::enable_if_t<std::is_floating_point_v<T>, T> rad2deg(T radians)
{
  return radians * static_cast<T>(180.0 / M_PI);
}

/// @brief Converts RPM to Hertz
/// @param rpm
/// @return Hertz
static inline double rpm2hz(double rpm)
{
  return rpm / 60.0;
}

}  // namespace nebula::drivers

#endif  // NEBULA_CONFIGURATION_BASE_H
