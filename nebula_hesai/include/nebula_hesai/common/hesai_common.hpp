// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NEBULA_HESAI_COMMON_H
#define NEBULA_HESAI_COMMON_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/util/string_conversions.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
namespace nebula
{
namespace drivers
{
/// @brief struct for Hesai sensor configuration
struct HesaiSensorConfiguration : public LidarConfigurationBase
{
  std::string multicast_ip;
  uint16_t gnss_port{};
  uint16_t sync_angle{};
  double cut_angle{};
  double dual_return_distance_threshold{};
  std::string calibration_path;
  uint16_t rotation_speed;
  uint16_t cloud_min_angle;
  uint16_t cloud_max_angle;
  PtpProfile ptp_profile;
  uint8_t ptp_domain;
  PtpTransportType ptp_transport_type;
  PtpSwitchType ptp_switch_type;
  uint8_t ptp_lock_threshold;
  std::optional<std::string> downsample_mask_path;
  bool hires_mode;
};
/// @brief Convert HesaiSensorConfiguration to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, HesaiSensorConfiguration const & arg)
{
  os << "Hesai Sensor Configuration:" << '\n';
  os << (LidarConfigurationBase)(arg) << '\n';
  os << "Multicast: "
     << (arg.multicast_ip.empty() ? "disabled" : "enabled, group: " + arg.multicast_ip) << '\n';
  os << "GNSS Port: " << arg.gnss_port << '\n';
  os << "Rotation Speed: " << arg.rotation_speed << '\n';
  os << "Sync Angle: " << arg.sync_angle << '\n';
  os << "Cut Angle: " << arg.cut_angle << '\n';
  os << "FoV Start: " << arg.cloud_min_angle << '\n';
  os << "FoV End: " << arg.cloud_max_angle << '\n';
  os << "Dual Return Distance Threshold: " << arg.dual_return_distance_threshold << '\n';
  os << "Calibration Path: " << arg.calibration_path << '\n';
  os << "PTP Profile: " << arg.ptp_profile << '\n';
  os << "PTP Domain: " << std::to_string(arg.ptp_domain) << '\n';
  os << "PTP Transport Type: " << arg.ptp_transport_type << '\n';
  os << "PTP Switch Type: " << arg.ptp_switch_type << '\n';
  os << "High Resolution Mode: " << arg.hires_mode << '\n';
  os << "PTP Lock Threshold: " << std::to_string(arg.ptp_lock_threshold) << '\n';
  os << "High Resolution Mode: " << (arg.hires_mode ? "enabled" : "disabled") << '\n';
  os << "Downsample Filter: "
     << (arg.downsample_mask_path ? "enabled, path: " + arg.downsample_mask_path.value()
                                  : "disabled");
  return os;
}

struct HesaiCalibrationConfigurationBase : public CalibrationConfigurationBase
{
  virtual nebula::Status load_from_bytes(const std::vector<uint8_t> & buf) = 0;
  virtual nebula::Status load_from_file(const std::string & calibration_file) = 0;
  virtual nebula::Status save_to_file_from_bytes(
    const std::string & calibration_file, const std::vector<uint8_t> & buf) = 0;

  [[nodiscard]] virtual std::tuple<float, float> get_fov_padding() const = 0;
};

/// @brief struct for Hesai calibration configuration
struct HesaiCalibrationConfiguration : public HesaiCalibrationConfigurationBase
{
  std::map<size_t, float> elev_angle_map;
  std::map<size_t, float> azimuth_offset_map;

  inline nebula::Status load_from_file(const std::string & calibration_file) override
  {
    std::ifstream ifs(calibration_file);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }
    std::ostringstream ss;
    ss << ifs.rdbuf();  // reading data
    ifs.close();
    return load_from_string(ss.str());
  }

  nebula::Status load_from_bytes(const std::vector<uint8_t> & buf) override
  {
    std::string calibration_string = std::string(buf.begin(), buf.end());
    return load_from_string(calibration_string);
  }

  /// @brief Loading calibration data
  /// @param calibration_content
  /// @return Resulting status
  inline nebula::Status load_from_string(const std::string & calibration_content)
  {
    std::stringstream ss;
    ss << calibration_content;
    std::string line;
    constexpr size_t expected_cols = 3;
    while (std::getline(ss, line)) {
      boost::char_separator<char> sep(",");
      boost::tokenizer<boost::char_separator<char>> tok(line, sep);

      std::vector<std::string> actual_tokens(tok.begin(), tok.end());
      if (actual_tokens.size() < expected_cols || actual_tokens.size() > expected_cols) {
        std::cerr << "Ignoring line with unexpected data: " << line << std::endl;
        continue;
      }

      try {
        int laser_id = std::stoi(actual_tokens[0]);
        float elevation = std::stof(actual_tokens[1]);
        float azimuth = std::stof(actual_tokens[2]);
        elev_angle_map[laser_id - 1] = elevation;
        azimuth_offset_map[laser_id - 1] = azimuth;
      } catch (const std::invalid_argument & ia) {
        continue;
      }
    }
    return Status::OK;
  }

  /// @brief Saving calibration data (not used)
  /// @param calibration_file
  /// @return Resulting status
  inline nebula::Status save_to_file(const std::string & calibration_file)
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

  nebula::Status save_to_file_from_bytes(
    const std::string & calibration_file, const std::vector<uint8_t> & buf) override
  {
    std::string calibration_string = std::string(buf.begin(), buf.end());
    return save_file_from_string(calibration_file, calibration_string);
  }

  /// @brief Saving calibration data from string
  /// @param calibration_file path
  /// @param calibration_string calibration string
  /// @return Resulting status
  inline nebula::Status save_file_from_string(
    const std::string & calibration_file, const std::string & calibration_string)
  {
    std::ofstream ofs(calibration_file);
    if (!ofs) {
      return Status::CANNOT_SAVE_FILE;
    }
    ofs << calibration_string;
    ofs.close();
    return Status::OK;
  }

  [[nodiscard]] std::tuple<float, float> get_fov_padding() const override
  {
    float min = INFINITY;
    float max = -INFINITY;

    for (const auto & item : azimuth_offset_map) {
      min = std::min(min, item.second);
      max = std::max(max, item.second);
    }

    return {-max, -min};
  }
};

/// @brief struct for Hesai correction configuration (for AT)
struct HesaiCorrection : public HesaiCalibrationConfigurationBase
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
  inline nebula::Status load_from_bytes(const std::vector<uint8_t> & buf) override
  {
    size_t index;
    for (index = 0; index < buf.size() - 1; index++) {
      if (buf[index] == 0xee && buf[index + 1] == 0xff) break;
    }
    delimiter = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
    versionMajor = buf[index + 2] & 0xff;
    versionMinor = buf[index + 3] & 0xff;
    channelNumber = buf[index + 4] & 0xff;
    mirrorNumber = buf[index + 5] & 0xff;
    frameNumber = buf[index + 6] & 0xff;
    index += 7;
    for (uint8_t i = 0; i < 8; i++) {
      frameConfig[i] = buf[index] & 0xff;
      index++;
    }
    resolution = buf[index] & 0xff;
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

        for (uint8_t i = 0; i < mirrorNumber; i++) {
          startFrame[i] *= resolution;
          endFrame[i] *= resolution;
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

        break;

      default:
        break;
    }
    return Status::OK;
  }

  /// @brief Load correction data from file
  /// @param correction_file path
  /// @return Resulting status
  inline nebula::Status load_from_file(const std::string & correction_file) override
  {
    std::ifstream ifs(correction_file, std::ios::in | std::ios::binary);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }
    std::vector<unsigned char> buf;
    //    int cnt = 0;
    while (!ifs.eof()) {
      unsigned char c;
      ifs.read(reinterpret_cast<char *>(&c), sizeof(unsigned char));
      buf.emplace_back(c);
    }
    load_from_bytes(buf);

    ifs.close();
    return Status::OK;
  }

  /// @brief Save correction data from binary buffer
  /// @param correction_file path
  /// @param buf correction binary
  /// @return Resulting status
  inline nebula::Status save_to_file_from_bytes(
    const std::string & correction_file, const std::vector<uint8_t> & buf) override
  {
    std::ofstream ofs(correction_file, std::ios::trunc | std::ios::binary);
    if (!ofs) {
      std::cerr << "Could not create file: " << correction_file << "\n";
      return Status::CANNOT_SAVE_FILE;
    }
    bool sop_received = false;
    for (const auto & byte : buf) {
      if (!sop_received) {
        if (byte == 0xEE) {
          sop_received = true;
        }
      }
      if (sop_received) {
        ofs << byte;
      }
    }
    ofs.close();
    if (sop_received) return Status::OK;
    return Status::INVALID_CALIBRATION_FILE;
  }

  static const int g_step3 = 200 * 256;

  /// @brief Get azimuth adjustment for channel and precision azimuth
  /// @param ch The channel id
  /// @param azi The precision azimuth in (0.01 / 256) degree unit
  /// @return The azimuth adjustment in 0.01 degree unit
  [[nodiscard]] int8_t get_azimuth_adjust_v3(uint8_t ch, uint32_t azi) const
  {
    unsigned int i = std::floor(1.f * azi / g_step3);
    unsigned int l = azi - i * g_step3;
    float k = 1.f * l / g_step3;
    return round((1 - k) * azimuthOffset[ch * 180 + i] + k * azimuthOffset[ch * 180 + i + 1]);
  }

  /// @brief Get elevation adjustment for channel and precision azimuth
  /// @param ch The channel id
  /// @param azi The precision azimuth in (0.01 / 256) degree unit
  /// @return The elevation adjustment in 0.01 degree unit
  [[nodiscard]] int8_t get_elevation_adjust_v3(uint8_t ch, uint32_t azi) const
  {
    unsigned int i = std::floor(1.f * azi / g_step3);
    unsigned int l = azi - i * g_step3;
    float k = 1.f * l / g_step3;
    return round((1 - k) * elevationOffset[ch * 180 + i] + k * elevationOffset[ch * 180 + i + 1]);
  }

  [[nodiscard]] std::tuple<float, float> get_fov_padding() const override
  {
    // TODO(mojomex): calculate instead of hard-coding
    // The reason this is tricky is that an upper bound over all azimuth/elevation combinations has
    // to be found. For other sensors, this is only a function of elevation, so the search space is
    // tiny compared to AT128. We should be able to find an upper bound of `getAzimuthAdjustV3` but
    // I have not invested the time for now.
    return {-5, 5};
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

/// @brief Convert return mode name to ReturnMode enum (Hesai-specific return_mode_from_string)
/// @param return_mode Return mode name (Upper and lower case letters must match)
/// @param sensor_model Model for correct conversion
/// @return Corresponding ReturnMode
inline ReturnMode return_mode_from_string_hesai(
  const std::string & return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDARXT16:
    case SensorModel::HESAI_PANDARXT32:
    case SensorModel::HESAI_PANDARXT32M:
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
    case SensorModel::HESAI_PANDARQT128:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "Dual" || return_mode == "LastStrongest")
        return ReturnMode::DUAL_LAST_STRONGEST;
      if (return_mode == "First") return ReturnMode::FIRST;
      if (return_mode == "LastFirst") return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == "FirstStrongest") return ReturnMode::DUAL_FIRST_STRONGEST;
      break;
    case SensorModel::HESAI_PANDARQT64:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Dual" || return_mode == "LastFirst") return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == "First") return ReturnMode::FIRST;
      break;
    case SensorModel::HESAI_PANDARAT128:
    case SensorModel::HESAI_PANDAR64:
    case SensorModel::HESAI_PANDAR40P:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "Dual" || return_mode == "LastStrongest")
        return ReturnMode::DUAL_LAST_STRONGEST;
      break;
    default:
      throw std::runtime_error("Unsupported sensor model: " + util::to_string(sensor_model));
  }

  return ReturnMode::UNKNOWN;
}

/// @brief Convert return mode number to ReturnMode enum
/// @param return_mode Return mode number from the hardware response
/// @param sensor_model Model for correct conversion
/// @return Corresponding ReturnMode
inline ReturnMode return_mode_from_int_hesai(
  const int return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDARXT16:
    case SensorModel::HESAI_PANDARXT32:
    case SensorModel::HESAI_PANDARXT32M:
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
    case SensorModel::HESAI_PANDARQT128:
      if (return_mode == 0) return ReturnMode::LAST;
      if (return_mode == 1) return ReturnMode::STRONGEST;
      if (return_mode == 2) return ReturnMode::DUAL_LAST_STRONGEST;
      if (return_mode == 3) return ReturnMode::FIRST;
      if (return_mode == 4) return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == 5) return ReturnMode::DUAL_FIRST_STRONGEST;
      break;
    case SensorModel::HESAI_PANDARQT64:
      if (return_mode == 0) return ReturnMode::LAST;
      if (return_mode == 2) return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == 3) return ReturnMode::FIRST;
      break;
    case SensorModel::HESAI_PANDARAT128:
    case SensorModel::HESAI_PANDAR64:
    case SensorModel::HESAI_PANDAR40P:
      if (return_mode == 0) return ReturnMode::LAST;
      if (return_mode == 1) return ReturnMode::STRONGEST;
      if (return_mode == 2) return ReturnMode::DUAL_LAST_STRONGEST;
      break;
    default:
      throw std::runtime_error("Unsupported sensor model: " + util::to_string(sensor_model));
  }

  return ReturnMode::UNKNOWN;
}

/// @brief Convert ReturnMode enum to return mode number
/// @param return_mode target ReturnMode
/// @param sensor_model Model for correct conversion
/// @return Corresponding return mode number for the hardware
inline int int_from_return_mode_hesai(
  const ReturnMode return_mode, const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDARXT16:
    case SensorModel::HESAI_PANDARXT32:
    case SensorModel::HESAI_PANDARXT32M:
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
    case SensorModel::HESAI_PANDARQT128:
      if (return_mode == ReturnMode::LAST) return 0;
      if (return_mode == ReturnMode::STRONGEST) return 1;
      if (return_mode == ReturnMode::DUAL || return_mode == ReturnMode::DUAL_LAST_STRONGEST)
        return 2;
      if (return_mode == ReturnMode::FIRST) return 3;
      if (return_mode == ReturnMode::DUAL_LAST_FIRST) return 4;
      if (return_mode == ReturnMode::DUAL_FIRST_STRONGEST) return 5;
      break;
    case SensorModel::HESAI_PANDARQT64:
      if (return_mode == ReturnMode::LAST) return 0;
      if (return_mode == ReturnMode::DUAL || return_mode == ReturnMode::DUAL_LAST_FIRST) return 2;
      if (return_mode == ReturnMode::FIRST) return 3;
      break;
    case SensorModel::HESAI_PANDARAT128:
    case SensorModel::HESAI_PANDAR64:
    case SensorModel::HESAI_PANDAR40P:
      if (return_mode == ReturnMode::LAST) return 0;
      if (return_mode == ReturnMode::STRONGEST) return 1;
      if (return_mode == ReturnMode::DUAL || return_mode == ReturnMode::DUAL_LAST_STRONGEST)
        return 2;
      break;
    default:
      throw std::runtime_error("Unsupported sensor model: " + util::to_string(sensor_model));
  }

  return -1;
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_COMMON_H
