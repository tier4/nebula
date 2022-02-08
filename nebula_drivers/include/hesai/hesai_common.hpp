#ifndef NEBULA_HESAI_COMMON_H
#define NEBULA_HESAI_COMMON_H

#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"

#include <fstream>
#include <sstream>
namespace nebula
{
namespace drivers
{
struct HesaiSensorConfiguration : SensorConfigurationBase
{
  uint16_t gnss_port{};
  double scan_phase{};
  double dual_return_distance_threshold{};
};
inline std::ostream & operator<<(std::ostream & os, HesaiSensorConfiguration const & arg)
{
  os << (SensorConfigurationBase)(arg) << ", GnssPort: " << arg.gnss_port
     << ", ScanPhase:" << arg.scan_phase;
  return os;
}

struct HesaiCalibrationConfiguration : CalibrationConfigurationBase
{
  std::map<size_t, float> elev_angle_map;
  std::map<size_t, float> azimuth_offset_map;

  inline nebula::Status LoadFromFile(const std::string & calibration_file)
  {
    std::ifstream ifs(calibration_file);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }

    std::string header;
    std::getline(ifs, header);

    char sep;
    int laser_id;
    float elevation;
    float azimuth;
    while (!ifs.eof()) {
      ifs >> laser_id >> sep >> elevation >> sep >> azimuth;
      elev_angle_map[laser_id - 1] = elevation;
      azimuth_offset_map[laser_id - 1] = azimuth;
    }
    ifs.close();
    return Status::OK;
  }

  inline nebula::Status LoadFromString(const std::string & calibration_content)
  {
    std::stringstream ss;
    ss << calibration_content;

    std::string header;
    std::getline(ss, header);

    char sep;
    int laser_id;
    float elevation;
    float azimuth;
    while (!ss.eof()) {
      ss >> laser_id >> sep >> elevation >> sep >> azimuth;
      elev_angle_map[laser_id - 1] = elevation;
      azimuth_offset_map[laser_id - 1] = azimuth;
    }
    return Status::OK;
  }

  inline nebula::Status SaveFile(const std::string & calibration_file)
  {
    std::ofstream ofs(calibration_file);
    if (!ofs) {
      return Status::CANNOT_SAVE_FILE;
    }
    ofs << "Laser id,Elevation,Azimuth" << std::endl;
    for (const auto & pair : elev_angle_map) {
      auto laser_id = pair.first + 1;
      float elevation = pair.second;
      float azimuth = azimuth_offset_map[pair.first];
      ofs << laser_id << "," << elevation << "," << azimuth << std::endl;
    }
    ofs.close();

    return Status::OK;
  }
};


}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_COMMON_H
