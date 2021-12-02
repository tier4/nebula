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
inline std::ostream& operator<<(std::ostream& os, HesaiSensorConfiguration const& arg)
{
  os << (SensorConfigurationBase)(arg) << ", gnss_port: " << arg.gnss_port << ", scan_phase:" << arg.scan_phase;
  return os;
}

struct HesaiCalibrationConfiguration : CalibrationConfigurationBase
{
  std::map<int, float> elev_angle_map;
  std::map<int, float> azimuth_offset_map;
};

struct HesaiCloudConfiguration : CloudConfigurationBase
{
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_COMMON_H
