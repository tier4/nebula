#ifndef NEBULA_HESAI_COMMON_H
#define NEBULA_HESAI_COMMON_H

#include "common/nebula_common.hpp"

namespace nebula
{
namespace drivers
{

struct HesaiSensorConfiguration : SensorConfigurationBase
{
  uint16_t gnss_port{};
  double scan_phase{};
};

struct HesaiCalibrationConfiguration : CalibrationConfigurationBase
{

};

}//drivers
}//nebula

#endif  //NEBULA_HESAI_COMMON_H
