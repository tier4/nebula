#ifndef NEBULA_CONFIGURATION_BASE_H
#define NEBULA_CONFIGURATION_BASE_H

namespace nebula
{
namespace drivers
{
// COMMON
enum class CoordinateMode { UNKNOWN = 0, CARTESIAN, SPHERICAL, CYLINDRICAL };

enum class EchoMode {
  UNKNOWN = 0,
  SINGLE_FIRST,
  SINGLE_STRONGEST,
  SINGLE_LAST,
  DUAL_ONLY,
  DUAL_FIRST,
  DUAL_LAST
};

// SENSOR_CONFIGURATION
class SensorModelBase
{
  const int UNKNOWN = 0;
};

class SensorConfigurationBase
{
  SensorModelBase sensor_model;
  EchoMode echo_mode;
  std::string host_ip;
  std::string sensor_ip;
  uint16_t data_port;
  uint16_t frequency_ms;
};

// CALIBRATION_CONFIGURATION
class CalibrationConfigurationBase
{
  std::string calibration_file;
};

// CLOUD_CONFIGURATION
enum class datatype {
  uint8 INT8 = 1,
  uint8 UINT8 = 2,
  uint8 INT16 = 3,
  uint8 UINT16 = 4,
  uint8 INT32 = 5,
  uint8 UINT32 = 6,
  uint8 FLOAT32 = 7,
  uint8 FLOAT64 = 8
};

class PointField
{
  std::string name;
  uint32 offset;
  uint8 datatype;
  uint32 count;
};

class CloudConfigurationBase
{
  PointField[] fields;
  CoordinateMode coordinate_mode;
  EchoMode echo_mode;
  double cloud_min_range;
  double cloud_max_range;
  bool remove_nans;  /// This may mean removing "all zeros" in some cases
};

}  // namespace drivers
}  // namespace nebula

#endif  //NEBULA_CONFIGURATION_BASE_H
