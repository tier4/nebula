#ifndef NEBULA_DRIVER_SENSOR_CONFIGURATION_H
#define NEBULA_DRIVER_SENSOR_CONFIGURATION_H

namespace nebula
{
namespace drivers
{
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

class SensorModelBase
{
  const int UNKNOWN = 0;
};

class SensorConfigurationBase
{
  SensorModelBase sensor_model;
  CoordinateMode coordinate_model;
  EchoMode echo_mode;
  std::string sensor_ip;
  uint16_t data_port;
  uint16_t cmd_port;
  uint16_t frequency_ms;
};

}  // namespace drivers
}  // namespace nebula

#endif  //NEBULA_DRIVER_SENSOR_CONFIGURATION_H
