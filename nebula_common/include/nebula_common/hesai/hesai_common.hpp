#ifndef NEBULA_HESAI_COMMON_H
#define NEBULA_HESAI_COMMON_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"

#include <bitset>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
namespace nebula
{
namespace drivers
{
/// @brief struct for Hesai sensor configuration
struct HesaiSensorConfiguration : SensorConfigurationBase
{
  uint16_t gnss_port{};
  double scan_phase{};
  double dual_return_distance_threshold{};
  uint16_t rotation_speed;
  uint16_t cloud_min_angle;
  uint16_t cloud_max_angle;
};
/// @brief Convert HesaiSensorConfiguration to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, HesaiSensorConfiguration const & arg)
{
  os << (SensorConfigurationBase)(arg) << ", GnssPort: " << arg.gnss_port
     << ", ScanPhase:" << arg.scan_phase << ", RotationSpeed:" << arg.rotation_speed
     << ", FOV(Start):" << arg.cloud_min_angle << ", FOV(End):" << arg.cloud_max_angle
     << ", DualReturnDistanceThreshold:" << arg.dual_return_distance_threshold;
  return os;
}

/// @brief struct for Hesai calibration configuration
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

  /// @brief Loading calibration data (not used)
  /// @param calibration_content
  /// @return Resulting status
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

  /// @brief Saving calibration data (not used)
  /// @param calibration_file
  /// @return Resulting status
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

/// @brief struct for Hesai correction configuration (for AT)
struct HesaiCorrection
{
  uint16_t delimiter;
  uint8_t versionMajor;
  uint8_t versionMinor;
  uint8_t channelNumber;
  uint8_t mirrorNumber;
  uint8_t frameNumber;
  uint8_t frameConfig[8];
  uint8_t resolution;

  uint32_t startFrame[8];
  uint32_t endFrame[8];
  int32_t azimuth[128];
  int32_t elevation[128];
  int8_t azimuthOffset[36000];
  int8_t elevationOffset[36000];
  uint8_t SHA256[32];

  /// @brief Load correction data from file
  /// @param buf Binary buffer
  /// @return Resulting status
  inline nebula::Status LoadFromBinary(const std::vector<uint8_t> & buf)
  {
    size_t index = 0;
    delimiter = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
    versionMajor = buf[index + 2] & 0xff;
    versionMinor = buf[index + 3] & 0xff;
    std::cout << "versionMajor=" << static_cast<int>(versionMajor) << std::endl;
    std::cout << "versionMinor=" << static_cast<int>(versionMinor) << std::endl;
    channelNumber = buf[index + 4] & 0xff;
    std::cout << "channelNumber=" << static_cast<int>(channelNumber) << std::endl;
    mirrorNumber = buf[index + 5] & 0xff;
    std::cout << "mirrorNumber=" << static_cast<int>(mirrorNumber) << std::endl;
    frameNumber = buf[index + 6] & 0xff;
    std::cout << "frameNumber=" << static_cast<int>(frameNumber) << std::endl;
    index += 7;
    for (uint8_t i = 0; i < 8; i++) {
      frameConfig[i] = buf[index] & 0xff;
      index++;
    }
    resolution = buf[index] & 0xff;
    std::cout << "resolution=" << static_cast<int>(resolution) << std::endl;
    index++;
    switch (versionMinor) {
      case 5:
        for (uint8_t i = 0; i < mirrorNumber; i++) {
          startFrame[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                          ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
          index += 4;
        }
        for (uint8_t i = 0; i < mirrorNumber; i++) {
          endFrame[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                        ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
          index += 4;
        }
        for (uint8_t i = 0; i < channelNumber; i++) {
          azimuth[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                       ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
          index += 4;
        }
        for (uint8_t i = 0; i < channelNumber; i++) {
          elevation[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                         ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
          index += 4;
        }
        for (int i = 0; i < channelNumber * 180; i++) {
          azimuthOffset[i] = buf[index] & 0xff;
          index++;
        }
        for (int i = 0; i < channelNumber * 180; i++) {
          elevationOffset[i] = buf[index] & 0xff;
          index++;
        }

        // 230328 add
        for (uint8_t i = 0; i < mirrorNumber; i++) {
          startFrame[i] *= resolution;
          endFrame[i] *= resolution;
          std::cout << "startFrame[" << static_cast<int>(i)
                    << "]=" << static_cast<int>(startFrame[i]) << std::endl;
          std::cout << "endFrame[" << static_cast<int>(i) << "]=" << static_cast<int>(endFrame[i])
                    << std::endl;
        }
        for (uint8_t i = 0; i < channelNumber; i++) {
          azimuth[i] *= resolution;
          elevation[i] *= resolution;
        }
        for (int i = 0; i < channelNumber * 180; i++) {
          azimuthOffset[i] *= resolution;
          elevationOffset[i] *= resolution;
        }
        break;

      case 3:  // not worked...
        for (uint8_t i = 0; i < mirrorNumber; i++) {
          startFrame[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8;
          index += 2;
        }
        for (uint8_t i = 0; i < mirrorNumber; i++) {
          endFrame[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8;
          index += 2;
        }
        for (uint8_t i = 0; i < channelNumber; i++) {
          azimuth[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8;
          index += 2;
        }
        for (uint8_t i = 0; i < channelNumber; i++) {
          elevation[i] = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8;
          index += 2;
        }
        for (int i = 0; i < 36000; i++) {
          azimuthOffset[i] = buf[index] & 0xff;
          index++;
        }
        for (int i = 0; i < 36000; i++) {
          elevationOffset[i] = buf[index] & 0xff;
          index++;
        }

        for (uint8_t i = 0; i < mirrorNumber; i++) {
          std::cout << "startFrame[" << static_cast<int>(i)
                    << "]=" << static_cast<int>(startFrame[i]) << std::endl;
          std::cout << "endFrame[" << static_cast<int>(i) << "]=" << static_cast<int>(endFrame[i])
                    << std::endl;
          /*
        startFrame[i] *= 2.56;
        endFrame[i] *= 2.56;
        std::cout << "startFrame[" << static_cast<int>(i) << "]=" << static_cast<int>(startFrame[i])
        << std::endl; std::cout << "endFrame[" << static_cast<int>(i) << "]=" <<
        static_cast<int>(endFrame[i]) << std::endl;
        */
        }

        break;

      default:
        break;
    }
    return Status::OK;
  }

  /// @brief Load correction data from file
  /// @param correction_file path
  /// @return Resulting status
  inline nebula::Status LoadFromFile(const std::string & correction_file)
  {
    std::ifstream ifs(correction_file, std::ios::in | std::ios::binary);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }
    std::vector<unsigned char> buf;
    //    int cnt = 0;
    while (!ifs.eof()) {
      unsigned char c;
      ifs.read((char *)&c, sizeof(unsigned char));
      buf.emplace_back(c);
    }
    LoadFromBinary(buf);

    ifs.close();
    return Status::OK;
  }

  static const int STEP3 = 200 * 256;

  /// @brief Get azimuth adjustment for channel and precision azimuth
  /// @param ch The channel id
  /// @param azi The precision azimuth in (0.01 / 256) degree unit
  /// @return The azimuth adjustment in 0.01 degree unit
  int8_t getAzimuthAdjustV3(uint8_t ch, uint32_t azi) const
  {
    unsigned int i = std::floor(1.f * azi / STEP3);
    unsigned int l = azi - i * STEP3;
    float k = 1.f * l / STEP3;
    return round((1 - k) * azimuthOffset[ch * 180 + i] + k * azimuthOffset[ch * 180 + i + 1]);
  }

  /// @brief Get elevation adjustment for channel and precision azimuth
  /// @param ch The channel id
  /// @param azi The precision azimuth in (0.01 / 256) degree unit
  /// @return The elevation adjustment in 0.01 degree unit
  int8_t getElevationAdjustV3(uint8_t ch, uint32_t azi) const
  {
    unsigned int i = std::floor(1.f * azi / STEP3);
    unsigned int l = azi - i * STEP3;
    float k = 1.f * l / STEP3;
    return round((1 - k) * elevationOffset[ch * 180 + i] + k * elevationOffset[ch * 180 + i + 1]);
  }
};

/*
<option value="0">Last Return</option>
<option value="1">Strongest Return</option>
<option value="3">First Return</option>
<option value="2">Last Return + Strongest Return</option>
<option value="4">First Return + Last Return</option>
<option value="5">First Return + Strongest Return</option>
*/
/*
<option value="0">Last Return</option>
<option value="1">Strongest Return</option>
<option value="3">First Return</option>
<option value="2">Last Return + Strongest Return</option>
<option value="4">First Return + Strongest Return</option>
<option value="5">First Return + Last Return</option>
<option value="6">First Return + Last Return + Strongest Return</option>
*/

/// @brief Convert return mode name to ReturnMode enum (Hesai-specific ReturnModeFromString)
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @param sensor_model Model for correct conversion
/// @return Corresponding ReturnMode
inline ReturnMode ReturnModeFromStringHesai(
  const std::string & return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDARXT32M:
    case SensorModel::HESAI_PANDARAT128:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "LastStrongest") return ReturnMode::DUAL_LAST_STRONGEST;
      if (return_mode == "First") return ReturnMode::FIRST;
      if (return_mode == "LastFirst") return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == "FirstStrongest") return ReturnMode::DUAL_FIRST_STRONGEST;
      if (return_mode == "Dual") return ReturnMode::DUAL;
      break;
    case SensorModel::HESAI_PANDARQT64:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Dual") return ReturnMode::DUAL;
      if (return_mode == "First") return ReturnMode::FIRST;
      break;
    default:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "Dual") return ReturnMode::DUAL;
      break;
  }

  return ReturnMode::UNKNOWN;
}

/// @brief Convert return mode number to ReturnMode enum
/// @param return_mode Return mode number from the hardware response
/// @param sensor_model Model for correct conversion
/// @return Corresponding ReturnMode
inline ReturnMode ReturnModeFromIntHesai(const int return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDARXT32M:
    case SensorModel::HESAI_PANDARAT128:
      if (return_mode == 0) return ReturnMode::LAST;
      if (return_mode == 1) return ReturnMode::STRONGEST;
      if (return_mode == 2) return ReturnMode::DUAL_LAST_STRONGEST;
      if (return_mode == 3) return ReturnMode::FIRST;
      if (return_mode == 4) return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == 5) return ReturnMode::DUAL_FIRST_STRONGEST;
      break;
    case SensorModel::HESAI_PANDARQT64:
      if (return_mode == 0) return ReturnMode::LAST;
      if (return_mode == 2) return ReturnMode::DUAL;
      if (return_mode == 3) return ReturnMode::FIRST;
      break;
    default:
      if (return_mode == 0) return ReturnMode::LAST;
      if (return_mode == 1) return ReturnMode::STRONGEST;
      if (return_mode == 2) return ReturnMode::DUAL;
      break;
  }

  return ReturnMode::UNKNOWN;
}

/// @brief Convert ReturnMode enum to return mode number
/// @param return_mode target ReturnMode
/// @param sensor_model Model for correct conversion
/// @return Corresponding return mode number for the hardware
inline int IntFromReturnModeHesai(const ReturnMode return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDARXT32M:
    case SensorModel::HESAI_PANDARAT128:
      if (return_mode == ReturnMode::LAST) return 0;
      if (return_mode == ReturnMode::STRONGEST) return 1;
      if (return_mode == ReturnMode::DUAL_LAST_STRONGEST) return 2;
      if (return_mode == ReturnMode::FIRST) return 3;
      if (return_mode == ReturnMode::DUAL_LAST_FIRST) return 4;
      if (return_mode == ReturnMode::DUAL_FIRST_STRONGEST) return 5;
      break;
    case SensorModel::HESAI_PANDARQT64:
      if (return_mode == ReturnMode::LAST) return 0;
      if (return_mode == ReturnMode::DUAL) return 2;
      if (return_mode == ReturnMode::FIRST) return 3;
      break;
    default:
      if (return_mode == ReturnMode::LAST) return 0;
      if (return_mode == ReturnMode::STRONGEST) return 1;
      if (return_mode == ReturnMode::DUAL) return 2;
      break;
  }

  return -1;
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_COMMON_H
