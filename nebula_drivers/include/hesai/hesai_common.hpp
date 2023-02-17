#ifndef NEBULA_HESAI_COMMON_H
#define NEBULA_HESAI_COMMON_H

#include <bitset>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

#include "common/nebula_common.hpp"
#include "common/nebula_status.hpp"
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
     << ", FOV(Start):" << arg.cloud_min_angle << ", FOV(End):" << arg.cloud_max_angle;
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
  /// @param correction_file path
  /// @return Resulting status
  inline nebula::Status LoadFromFile(const std::string & correction_file)
  {
    std::ifstream ifs(correction_file, std::ios::in | std::ios::binary);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }
    /*
    std::vector<char> buf;
    while(!ifs.eof()){
      char c;
      ifs.read( ( char * ) &c, sizeof( char ) );
//      std::cout << c << std::endl;
      buf.emplace_back(c);
    }
    */
    std::vector<unsigned char> buf;
    //    int cnt = 0;
    while (!ifs.eof()) {
      unsigned char c;
      ifs.read((char *)&c, sizeof(unsigned char));
      //      std::cout << c << std::endl;
      /*
      if(cnt < 30){
        std::cout << static_cast<int>(c) << std::endl;
        std::cout << static_cast<int>(c & 0xff) << std::endl;
      }
      */
      buf.emplace_back(c);
      /*
      if(cnt < 30){
        std::cout << static_cast<int>(buf[buf.size()-1] & 0xff) << std::endl;
      }
      cnt++;
      */
    }
    /*
    std::cout << "-----" << std::endl;
    cnt = 0;
    for (int8_t i = 0; i < buf.size(); i++){
      if(cnt < 30)
        std::cout << static_cast<int>(buf[i]) << std::endl;
      cnt++;
    }
    std::cout << "-----" << std::endl;
    */

    size_t index = 0;
    /*
    std::cout << "buf[2]: " << buf[2] << std::endl;
    std::cout << "(buf[index + 2] & 0xff): " << (buf[index + 2] & 0xff) << std::endl;
    std::cout << "static_cast<uint8_t>(buf[2]): " << static_cast<uint8_t>(buf[2]) << std::endl;
    std::cout << "static_cast<uint8_t>((buf[index + 2] & 0xff)): " << static_cast<uint8_t>((buf[index + 2] & 0xff)) << std::endl;
    std::cout << "int static_cast<uint8_t>(buf[2]): " << static_cast<int>(static_cast<uint8_t>(buf[2])) << std::endl;
    std::cout << "int static_cast<uint8_t>((buf[index + 2] & 0xff)): " << static_cast<int>(static_cast<uint8_t>((buf[index + 2] & 0xff))) << std::endl;
    uint8_t tmp = static_cast<uint8_t>((buf[index + 2] & 0xff));
    std::cout << "tmp: " << tmp << std::endl;
    std::cout << "static_cast<int>(tmp): " << static_cast<int>(tmp) << std::endl;
    std::cout << "std::bitset<8>(tmp): " << std::bitset<8>(tmp) << std::endl;

    std::cout << "std::bitset<8>(static_cast<int>(tmp)): " << std::bitset<8>(static_cast<int>(tmp)) << std::endl;
    std::cout << "buf[4]: " << buf[4] << std::endl;
    std::cout << "(buf[index + 4] & 0xff): " << (buf[index + 4] & 0xff) << std::endl;
    std::cout << "static_cast<uint8_t>(buf[4]): " << static_cast<uint8_t>(buf[4]) << std::endl;
    std::cout << "static_cast<uint8_t>((buf[index + 4] & 0xff)): " << static_cast<uint8_t>((buf[index + 4] & 0xff)) << std::endl;
    std::cout << "int static_cast<uint8_t>(buf[4]): " << static_cast<int>(static_cast<uint8_t>(buf[4])) << std::endl;
    std::cout << "int static_cast<uint8_t>((buf[index + 4] & 0xff)): " << static_cast<int>(static_cast<uint8_t>((buf[index + 2] & 0xff))) << std::endl;
    tmp = static_cast<uint8_t>((buf[index + 4] & 0xff));
    std::cout << "tmp: " << tmp << std::endl;
    std::cout << "static_cast<int>(tmp): " << static_cast<int>(tmp) << std::endl;
    std::cout << "std::bitset<8>(tmp): " << std::bitset<8>(tmp) << std::endl;
    std::cout << "std::bitset<8>(static_cast<int>(tmp)): " << std::bitset<8>(static_cast<int>(tmp)) << std::endl;
    */
    //*
    delimiter = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
    versionMajor = buf[index + 2] & 0xff;
    versionMinor = buf[index + 3] & 0xff;
    channelNumber = buf[index + 4] & 0xff;
    mirrorNumber = buf[index + 5] & 0xff;
    frameNumber = buf[index + 6] & 0xff;
    //*/
    /*
    delimiter = static_cast<uint16_t>((buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff)));
    versionMajor = static_cast<uint8_t>(buf[index + 2] & 0xff);
    versionMinor = static_cast<uint8_t>(buf[index + 3] & 0xff);
    channelNumber = static_cast<uint8_t>(buf[index + 4] & 0xff);
    */
    /*
    std::cout << "channelNumber: " << channelNumber << std::endl;
    std::cout << "static_cast<int>(channelNumber): " << static_cast<int>(channelNumber) << std::endl;
    std::cout << "std::bitset<8>(channelNumber): " << std::bitset<8>(channelNumber) << std::endl;
    std::cout << "std::bitset<8>(static_cast<int>(channelNumber)): " << std::bitset<8>(static_cast<int>(channelNumber)) << std::endl;
    */
    /*
    mirrorNumber = static_cast<uint8_t>(buf[index + 5] & 0xff);
    frameNumber = static_cast<uint8_t>(buf[index + 6] & 0xff);
    */
    index += 7;
    for (uint8_t i = 0; i < 8; i++) {
      frameConfig[i] = buf[index] & 0xff;
      index++;
    }
    resolution = buf[index] & 0xff;
    index++;
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
    /*
    std::cout << "channelNumber: " << channelNumber << std::endl;
    std::cout << "mirrorNumber: " << mirrorNumber << std::endl;
    std::cout << "frameNumber: " << frameNumber << std::endl;
    std::cout << "mirrorNumber: " << mirrorNumber << std::endl;
    for (int8_t i = 0; i < 8; i++) {
      std::cout << "frameConfig[" << i << "]: " << frameConfig[i] << std::endl;
    }

    std::cout << "channelNumber: " << std::bitset<8>(channelNumber) << std::endl;
    std::cout << "mirrorNumber: " << std::bitset<8>(mirrorNumber) << std::endl;
    std::cout << "frameNumber: " << std::bitset<8>(frameNumber) << std::endl;
    std::cout << "mirrorNumber: " << std::bitset<8>(mirrorNumber) << std::endl;
    for (int8_t i = 0; i < 8; i++) {
      std::cout << "frameConfig[" << i << "]: " << std::bitset<8>(frameConfig[i]) << std::endl;
    }

    std::cout << "channelNumber: " << static_cast<int>(channelNumber) << std::endl;
    std::cout << "mirrorNumber: " << static_cast<int>(mirrorNumber) << std::endl;
    std::cout << "frameNumber: " << static_cast<int>(frameNumber) << std::endl;
    std::cout << "mirrorNumber: " << static_cast<int>(mirrorNumber) << std::endl;
    for (int8_t i = 0; i < 8; i++) {
      std::cout << "frameConfig[" << static_cast<int>(i) << "]: " << static_cast<int>(frameConfig[i]) << std::endl;
    }
    */

    ifs.close();
    return Status::OK;
  }

  static const int STEP3 = 200 * 256;
  int8_t getAzimuthAdjustV3(uint8_t ch, uint32_t azi) const
  {
    unsigned int i = std::floor(1.f * azi / STEP3);
    unsigned int l = azi - i * STEP3;
    float k = 1.f * l / STEP3;
    return round((1 - k) * azimuthOffset[ch * 180 + i] + k * azimuthOffset[ch * 180 + i + 1]);
  }
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
