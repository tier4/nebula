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
  DUAL_LAST
};

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
  HESAI_PANDAR128_V13,
  HESAI_PANDAR128_V14
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
    case SensorModel::HESAI_PANDAR128_V13:
    case SensorModel::HESAI_PANDAR128_V14:
      os << "Pandar128";
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
  bool sensor_online;
};
inline std::ostream & operator<<(std::ostream & os, nebula::drivers::SensorConfigurationBase const & arg)
{
  os << "sensor_model: " << arg.sensor_model << ", return_mode: " << arg.return_mode << ", host_ip"
     << arg.host_ip << ", sensor_ip: " << arg.sensor_ip << ", frame_id: " << arg.frame_id
     << ", data_port: " << arg.data_port << ", frequency: " << arg.frequency_ms;
  return os;
}

// CALIBRATION_CONFIGURATION
struct CalibrationConfigurationBase
{
  std::string calibration_file;
};

// CLOUD_CONFIGURATION
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

struct CloudConfigurationBase
{
  CoordinateMode coordinate_mode;
  ReturnMode return_mode;
  double cloud_min_range;
  double cloud_max_range;
  bool remove_nans;  /// todo: consider changing to only_finite
  std::vector<PointField> fields;
};

inline SensorModel SensorModelFromString(const std::string & sensor_model)
{
  if (sensor_model == "Pandar64") return SensorModel::HESAI_PANDAR64;
  if (sensor_model == "Pandar40P") return SensorModel::HESAI_PANDAR40P;
  if (sensor_model == "Pandar40M") return SensorModel::HESAI_PANDAR40M;
  if (sensor_model == "PandarXT32") return SensorModel::HESAI_PANDARXT32;
  if (sensor_model == "PandarQT64") return SensorModel::HESAI_PANDARQT64;
  if (sensor_model == "PandarQT128") return SensorModel::HESAI_PANDARQT128;
  if (sensor_model == "Pandar128") return SensorModel::HESAI_PANDAR128_V14;

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
