#ifndef NEBULA_CONFIGURATION_BASE_H
#define NEBULA_CONFIGURATION_BASE_H

#include <vector>
#include <string>
namespace nebula
{
namespace drivers
{
// COMMON
enum class CoordinateMode {
  UNKNOWN = 0,
  CARTESIAN,
  SPHERICAL,
  CYLINDRICAL
};

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
enum class SensorModelBase
{
  UNKNOWN = 0
};

struct SensorConfigurationBase
{
  SensorModelBase sensor_model;
  EchoMode echo_mode;
  std::string host_ip;
  std::string sensor_ip;
  uint16_t data_port;
  uint16_t frequency_ms;
};

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
  EchoMode echo_mode;
  double cloud_min_range;
  double cloud_max_range;
  bool remove_nans;  /// This may mean removing "all zeros" in some cases
  std::vector<PointField> fields;
};

}  // namespace drivers
}  // namespace nebula

#endif  //NEBULA_CONFIGURATION_BASE_H
