#ifndef NEBULA_COMMON_H
#define NEBULA_COMMON_H

#include <map>
#include <ostream>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{
// COMMON
enum class CoordinateMode { UNKNOWN = 0, CARTESIAN, SPHERICAL, CYLINDRICAL };

enum class ReturnMode {
  UNKNOWN = 0,
  SINGLE_FIRST,
  SINGLE_STRONGEST,
  SINGLE_LAST,
  DUAL_ONLY,
  DUAL_FIRST,
  DUAL_LAST,
  DUAL_WEAK_FIRST,
  DUAL_WEAK_LAST,
  DUAL_STRONGEST_LAST,
  DUAL_STRONGEST_FIRST,
  TRIPLE,
  LAST,
  STRONGEST,
  LAST_STRONGEST,
  FIRST,
  LAST_FIRST,
  FIRST_STRONGEST,
  DUAL
};

inline uint8_t ReturnModeToInt(const ReturnMode & mode)
{
  switch (mode) {
    case ReturnMode::SINGLE_STRONGEST:
      return 1;
      break;
    case ReturnMode::SINGLE_LAST:
      return 2;
      break;
    case ReturnMode::DUAL_FIRST:
      return 3;
      break;
    case ReturnMode::DUAL_LAST:
      return 4;
      break;
    case ReturnMode::DUAL_ONLY:
      return 5;
      break;
    case ReturnMode::SINGLE_FIRST:
      return 6;
      break;
    case ReturnMode::DUAL_STRONGEST_FIRST:
      return 7;
      break;
    case ReturnMode::DUAL_STRONGEST_LAST:
      return 8;
      break;
    case ReturnMode::DUAL_WEAK_FIRST:
      return 9;
      break;
    case ReturnMode::DUAL_WEAK_LAST:
      return 10;
      break;
    case ReturnMode::TRIPLE:
      return 11;
      break;
    // for Hesai
    case ReturnMode::LAST:
      return 12;
      break;
    case ReturnMode::STRONGEST:
      return 13;
      break;
    case ReturnMode::LAST_STRONGEST:
      return 14;
      break;
    case ReturnMode::FIRST:
      return 15;
      break;
    case ReturnMode::LAST_FIRST:
      return 16;
      break;
    case ReturnMode::FIRST_STRONGEST:
      return 17;
      break;
    case ReturnMode::DUAL:
      return 18;
      break;
    default:
    case ReturnMode::UNKNOWN:
      return 0;
      break;
  }
}

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
    case ReturnMode::LAST_STRONGEST:
      os << "LastStrongest";
      break;
    case ReturnMode::FIRST:
      os << "First";
      break;
    case ReturnMode::LAST_FIRST:
      os << "LastFirst";
      break;
    case ReturnMode::FIRST_STRONGEST:
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
enum class SensorModel {
  UNKNOWN = 0,
  HESAI_PANDAR64,
  HESAI_PANDAR40P,
  HESAI_PANDAR40M,
  HESAI_PANDARQT64,
  HESAI_PANDARQT128,
  HESAI_PANDARXT32,
  HESAI_PANDARXT32M,
  HESAI_PANDARAT128,
  HESAI_PANDAR128_V13,
  HESAI_PANDAR128_V14,
  VELODYNE_VLS128,
  VELODYNE_HDL64,
  VELODYNE_VLP32,
  VELODYNE_VLP32MR,
  VELODYNE_HDL32,
  VELODYNE_VLP16,
};

enum class datatype {
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8
};

struct PointField
{
  std::string name;
  uint32_t offset;
  uint8_t datatype;
  uint32_t count;
};

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
    case SensorModel::HESAI_PANDARXT32:
      os << "PandarXT32";
      break;
    case SensorModel::HESAI_PANDARXT32M:
      os << "PandarXT32M";
      break;
    case SensorModel::HESAI_PANDARAT128:
      os << "PandarAT128";
      break;
    case SensorModel::HESAI_PANDAR128_V13:
      os << "Pandar128_1.3";
      break;
    case SensorModel::HESAI_PANDAR128_V14:
      os << "Pandar128_1.4";
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
    case SensorModel::UNKNOWN:
      os << "Sensor Unknown";
      break;
  }
  return os;
}

struct SensorConfigurationBase
{
  SensorModel sensor_model;
  ReturnMode return_mode;
  std::string host_ip;
  std::string sensor_ip;
  std::string frame_id;
  uint16_t data_port;
  uint16_t frequency_ms;
  uint16_t packet_mtu_size;
  CoordinateMode coordinate_mode;
  double min_range;
  double max_range;
  bool remove_nans;  /// todo: consider changing to only_finite
  std::vector<PointField> fields;
};
inline std::ostream & operator<<(
  std::ostream & os, nebula::drivers::SensorConfigurationBase const & arg)
{
  os << "SensorModel: " << arg.sensor_model << ", ReturnMode: " << arg.return_mode
     << ", HostIP: " << arg.host_ip << ", SensorIP: " << arg.sensor_ip
     << ", FrameID: " << arg.frame_id << ", DataPort: " << arg.data_port
     << ", Frequency: " << arg.frequency_ms << ", MTU: " << arg.packet_mtu_size;
  return os;
}

struct CalibrationConfigurationBase
{
  std::string calibration_file;
};

inline SensorModel SensorModelFromString(const std::string & sensor_model)
{
  // Hesai
  if (sensor_model == "Pandar64") return SensorModel::HESAI_PANDAR64;
  if (sensor_model == "Pandar40P") return SensorModel::HESAI_PANDAR40P;
  if (sensor_model == "Pandar40M") return SensorModel::HESAI_PANDAR40M;
  if (sensor_model == "PandarXT32") return SensorModel::HESAI_PANDARXT32;
  if (sensor_model == "PandarXT32M") return SensorModel::HESAI_PANDARXT32M;
  if (sensor_model == "PandarAT128") return SensorModel::HESAI_PANDARAT128;
  if (sensor_model == "PandarQT64") return SensorModel::HESAI_PANDARQT64;
  if (sensor_model == "PandarQT128") return SensorModel::HESAI_PANDARQT128;
  if (sensor_model == "Pandar128") return SensorModel::HESAI_PANDAR128_V14;
  // Velodyne
  if (sensor_model == "VLS128") return SensorModel::VELODYNE_VLS128;
  if (sensor_model == "HDL64") return SensorModel::VELODYNE_HDL64;
  if (sensor_model == "VLP32") return SensorModel::VELODYNE_VLP32;
  if (sensor_model == "VLP32MR") return SensorModel::VELODYNE_VLP32MR;
  if (sensor_model == "HDL32") return SensorModel::VELODYNE_HDL32;
  if (sensor_model == "VLP16") return SensorModel::VELODYNE_VLP16;
  return SensorModel::UNKNOWN;
}

inline ReturnMode ReturnModeFromString(const std::string & return_mode)
{
  if (return_mode == "SingleFirst") return ReturnMode::SINGLE_FIRST;
  if (return_mode == "SingleStrongest") return ReturnMode::SINGLE_STRONGEST;
  if (return_mode == "SingleLast") return ReturnMode::SINGLE_LAST;
  if (return_mode == "Dual") return ReturnMode::DUAL_ONLY;

  return ReturnMode::UNKNOWN;
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_CONFIGURATION_BASE_H
