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

// cspell:ignore libhesai

#ifndef NEBULA_HESAI_COMMON__HESAI_COMMON_HPP_
#define NEBULA_HESAI_COMMON__HESAI_COMMON_HPP_

#include "nebula_core_common/nebula_common.hpp"
#include "nebula_core_common/nebula_status.hpp"
#include "nebula_core_common/util/string_conversions.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
namespace nebula
{
namespace drivers
{

bool supports_functional_safety(const SensorModel & sensor_model);

struct AdvancedFunctionalSafetyConfiguration
{
  std::string error_definitions_path;
  std::vector<uint16_t> ignored_error_codes;

  friend std::ostream & operator<<(
    std::ostream & os, const AdvancedFunctionalSafetyConfiguration & arg)
  {
    os << "advanced\n  error definitions: " << arg.error_definitions_path;
    os << "\n  ignored codes: ";
    if (!arg.ignored_error_codes.empty()) {
      for (size_t i = 0; i < arg.ignored_error_codes.size(); ++i) {
        if (i > 0) os << ", ";
        os << "0x" << std::hex << arg.ignored_error_codes[i];
      }
    } else {
      os << "none";
    }
    return os;
  }
};

/// @brief Converts String to PTP Profile
/// @param ptp_profile Profile as String
/// @return Corresponding PtpProfile
inline PtpProfile ptp_profile_from_string(const std::string & ptp_profile)
{
  // Hesai
  auto tmp_str = ptp_profile;
  std::transform(tmp_str.begin(), tmp_str.end(), tmp_str.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  if (tmp_str == "1588v2") return PtpProfile::IEEE_1588v2;
  if (tmp_str == "802.1as") return PtpProfile::IEEE_802_1AS;
  if (tmp_str == "automotive") return PtpProfile::IEEE_802_1AS_AUTO;

  return PtpProfile::UNKNOWN_PROFILE;
}

/// @brief Convert PtpProfile enum to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, nebula::drivers::PtpProfile const & arg)
{
  switch (arg) {
    case PtpProfile::IEEE_1588v2:
      os << "IEEE_1588v2";
      break;
    case PtpProfile::IEEE_802_1AS:
      os << "IEEE_802.1AS";
      break;
    case PtpProfile::IEEE_802_1AS_AUTO:
      os << "IEEE_802.1AS Automotive";
      break;
    case PtpProfile::UNKNOWN_PROFILE:
      os << "UNKNOWN";
      break;
  }
  return os;
}

/// @brief Converts String to PTP TransportType
/// @param transport_type Transport as String
/// @return Corresponding PtpTransportType
inline PtpTransportType ptp_transport_type_from_string(const std::string & transport_type)
{
  // Hesai
  auto tmp_str = transport_type;
  std::transform(tmp_str.begin(), tmp_str.end(), tmp_str.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  if (tmp_str == "udp") return PtpTransportType::UDP_IP;
  if (tmp_str == "l2") return PtpTransportType::L2;

  return PtpTransportType::UNKNOWN_TRANSPORT;
}

/// @brief Convert PtpTransportType enum to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, nebula::drivers::PtpTransportType const & arg)
{
  switch (arg) {
    case PtpTransportType::UDP_IP:
      os << "UDP/IP";
      break;
    case PtpTransportType::L2:
      os << "L2";
      break;
    case PtpTransportType::UNKNOWN_TRANSPORT:
      os << "UNKNOWN";
      break;
  }
  return os;
}

/// @brief Converts String to PTP SwitchType
/// @param switch_type Switch as String
/// @return Corresponding PtpSwitchType
inline PtpSwitchType ptp_switch_type_from_string(const std::string & switch_type)
{
  // Hesai
  auto tmp_str = switch_type;
  std::transform(tmp_str.begin(), tmp_str.end(), tmp_str.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  if (tmp_str == "tsn") return PtpSwitchType::TSN;
  if (tmp_str == "non_tsn") return PtpSwitchType::NON_TSN;

  return PtpSwitchType::UNKNOWN_SWITCH;
}

/// @brief Convert PtpSwitchType enum to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, nebula::drivers::PtpSwitchType const & arg)
{
  switch (arg) {
    case PtpSwitchType::TSN:
      os << "TSN";
      break;
    case PtpSwitchType::NON_TSN:
      os << "NON_TSN";
      break;
    case PtpSwitchType::UNKNOWN_SWITCH:
      os << "UNKNOWN";
      break;
  }
  return os;
}

/// @brief struct for Hesai sensor configuration
struct HesaiSensorConfiguration : public LidarConfigurationBase
{
  std::string multicast_ip;
  uint16_t gnss_port{};
  size_t udp_socket_receive_buffer_size_bytes{};
  uint16_t sync_angle{};
  double cut_angle{};
  double dual_return_distance_threshold{};
  std::string calibration_path;
  bool calibration_download_enabled;
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
  std::optional<uint32_t> blockage_mask_horizontal_bin_size_mdeg;
  std::optional<std::string> sync_diagnostics_topic;
  std::optional<AdvancedFunctionalSafetyConfiguration> functional_safety;
};
/// @brief Convert HesaiSensorConfiguration to string (Overloading the << operator)
/// @param os
/// @param arg
/// @return stream
inline std::ostream & operator<<(std::ostream & os, HesaiSensorConfiguration const & arg)
{
  os << "Hesai Sensor Configuration:" << '\n';
  os << static_cast<const LidarConfigurationBase &>(arg) << '\n';
  os << "Multicast: "
     << (arg.multicast_ip.empty() ? "disabled" : "enabled, group: " + arg.multicast_ip) << '\n';
  os << "GNSS Port: " << arg.gnss_port << '\n';
  os << "UDP Socket Receive Buffer Size: " << arg.udp_socket_receive_buffer_size_bytes << " B"
     << '\n';
  os << "Rotation Speed: " << arg.rotation_speed << '\n';
  os << "Sync Angle: " << arg.sync_angle << '\n';
  os << "Cut Angle: " << arg.cut_angle << '\n';
  os << "FoV Start: " << arg.cloud_min_angle << '\n';
  os << "FoV End: " << arg.cloud_max_angle << '\n';
  os << "Dual Return Distance Threshold: " << arg.dual_return_distance_threshold << '\n';
  os << "Calibration Path: " << arg.calibration_path << '\n';
  os << "Calibration Download: " << (arg.calibration_download_enabled ? "enabled" : "disabled")
     << '\n';
  os << "PTP Profile: " << arg.ptp_profile << '\n';
  os << "PTP Domain: " << std::to_string(arg.ptp_domain) << '\n';
  os << "PTP Transport Type: " << arg.ptp_transport_type << '\n';
  os << "PTP Switch Type: " << arg.ptp_switch_type << '\n';
  os << "High Resolution Mode: " << arg.hires_mode << '\n';
  os << "PTP Lock Threshold: " << std::to_string(arg.ptp_lock_threshold) << '\n';
  os << "High Resolution Mode: " << (arg.hires_mode ? "enabled" : "disabled") << '\n';
  os << "Downsample Filter: "
     << (arg.downsample_mask_path ? "enabled, path: " + arg.downsample_mask_path.value()
                                  : "disabled")
     << '\n';
  os << "Blockage Mask Output: "
     << (arg.blockage_mask_horizontal_bin_size_mdeg
           ? "enabled, horizontal bin size: " +
               std::to_string(arg.blockage_mask_horizontal_bin_size_mdeg.value()) + " mdeg"
           : "disabled")
     << '\n';
  os << "Synchronization Diagnostics: "
     << (arg.sync_diagnostics_topic ? ("enabled, topic: " + arg.sync_diagnostics_topic.value())
                                    : "disabled");

  if (supports_functional_safety(arg.sensor_model)) {
    os << '\n';
    os << "Functional Safety: ";
    if (arg.functional_safety) {
      os << *arg.functional_safety;
    } else {
      os << "basic";
    }
  }
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

    // NOTE: Slightly widen the FOV padding because some LiDARs do not transmit blocks near the end
    // of the FOV. If these blocks are missing, the point cloud may not be published at the desired
    // timing.
    min -= 1.0f;
    max += 1.0f;

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

/// @brief Angle correction data of the FTX140 (point cloud UDP protocol 7.3)
///
/// Instead of per-pixel angles, the sensor provides a fisheye camera model: focal length, principal
/// point, four radial distortion coefficients and a tilt of the image plane. The ray direction of
/// each pixel is computed from this model on load.
///
/// The model and its constants follow `FT::CorrectionV3::generate_pixel_vectors()` of the Hesai
/// LiDAR SDK 2.0 (libhesai/UdpProtocol/udp_protocol_v7_3.h, Copyright (C) 2023 Hesai Technology
/// Co., Ltd., Modified BSD License), so that points match the ones produced by Hesai's tools.
struct HesaiFtxCalibration : public HesaiCalibrationConfigurationBase
{
  static constexpr size_t max_rows = 192;
  static constexpr size_t max_columns = 256;
  /// @brief Header (10 bytes), 9 float model parameters, SHA-256 checksum (32 bytes)
  static constexpr size_t correction_size = 10 + 9 * sizeof(float) + 32;

  uint16_t rows{};
  uint16_t columns{};
  float focal_length{};
  float cx{};
  float cy{};
  std::array<float, 4> k{};
  float p1{};
  float p2{};

  /// @brief Unit ray direction (x, y, z) of each pixel, indexed by `row * max_columns + column`.
  /// Pixels outside of the model's valid area are all zero.
  std::vector<std::array<float, 3>> pixel_directions;

  inline nebula::Status load_from_bytes(const std::vector<uint8_t> & buf) override
  {
    if (
      buf.size() < correction_size || buf[0] != 0xEE || buf[1] != 0xFF || buf[2] != 0x07 ||
      buf[3] != 0x03) {
      return Status::INVALID_CALIBRATION_FILE;
    }

    rows = static_cast<uint16_t>(buf[6] | (buf[7] << 8));
    columns = static_cast<uint16_t>(buf[8] | (buf[9] << 8));
    if (rows == 0 || rows > max_rows || columns == 0 || columns > max_columns) {
      return Status::INVALID_CALIBRATION_FILE;
    }

    // The model parameters are little-endian floats
    std::array<float, 9> params{};
    std::memcpy(params.data(), buf.data() + 10, sizeof(params));
    focal_length = params[0];
    cx = params[1];
    cy = params[2];
    std::copy(params.begin() + 3, params.begin() + 7, k.begin());
    p1 = params[7];
    p2 = params[8];

    if (!(focal_length > 0)) {
      return Status::INVALID_CALIBRATION_FILE;
    }

    compute_pixel_directions();
    return Status::OK;
  }

  inline nebula::Status load_from_file(const std::string & calibration_file) override
  {
    std::ifstream ifs(calibration_file, std::ios::in | std::ios::binary);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }
    std::vector<uint8_t> buf(
      (std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    return load_from_bytes(buf);
  }

  inline nebula::Status save_to_file_from_bytes(
    const std::string & calibration_file, const std::vector<uint8_t> & buf) override
  {
    std::ofstream ofs(calibration_file, std::ios::trunc | std::ios::binary);
    if (!ofs) {
      std::cerr << "Could not create file: " << calibration_file << "\n";
      return Status::CANNOT_SAVE_FILE;
    }
    ofs.write(reinterpret_cast<const char *>(buf.data()), static_cast<std::streamsize>(buf.size()));
    return Status::OK;
  }

  /// @brief The FTX140 has no lidar range setting, so no padding is needed
  [[nodiscard]] std::tuple<float, float> get_fov_padding() const override { return {0, 0}; }

  /// @brief Get the unit ray direction of a pixel, or all zeros if the pixel is not usable
  [[nodiscard]] const std::array<float, 3> & pixel_direction(size_t row, size_t column) const
  {
    return pixel_directions.at(row * max_columns + column);
  }

private:
  /// @brief Radial distortion: theta_d = theta + k1 theta^3 + k2 theta^5 + k3 theta^7 + k4 theta^9
  [[nodiscard]] float distort(float theta) const
  {
    return theta + k[0] * std::pow(theta, 3) + k[1] * std::pow(theta, 5) +
           k[2] * std::pow(theta, 7) + k[3] * std::pow(theta, 9);
  }

  void compute_pixel_directions()
  {
    // The model is defined on the 768 x 576 detector image. Each output pixel covers 3 x 3 detector
    // pixels and is sampled at the center one.
    constexpr float detector_max_x = 767;
    constexpr float detector_max_y = 575;
    constexpr int binning = 3;
    constexpr auto pi = static_cast<float>(M_PI);

    // Tilt of the image plane: rotation by alpha about an in-plane axis at angle phi
    float alpha = 2 * std::atan(p1);
    float phi = p2;
    if (alpha < 0) {
      alpha = -alpha;
      phi = p2 + pi;
    }
    phi = (phi / pi / 2 - static_cast<int>(phi / pi / 2)) * 2 * pi + pi;

    const float cos_a = std::cos(alpha);
    const float sin_a = std::sin(alpha);
    const float cos_p = std::cos(phi);
    const float sin_p = std::sin(phi);
    const float r00 = cos_a + (1 - cos_a) * cos_p * cos_p;
    const float r01 = (1 - cos_a) * cos_p * sin_p;
    const float r10 = (1 - cos_a) * cos_p * sin_p;
    const float r11 = cos_a + (1 - cos_a) * sin_p * sin_p;
    const float r20 = -sin_a * sin_p;
    const float r21 = sin_a * cos_p;

    const float theta_d_max =
      std::hypot(std::max(cx, detector_max_x - cx), std::max(cy, detector_max_y - cy)) /
      focal_length;
    const float tan_theta_max = 750.0f / focal_length;

    // Tabulate the distortion function (up to where it stops increasing) to invert it
    constexpr int n_interp = 1024;
    std::vector<float> theta_table;
    std::vector<float> theta_d_table;
    for (int i = 0; i < n_interp; ++i) {
      float theta = static_cast<float>(i) * tan_theta_max / n_interp;
      float theta_d = distort(theta);
      if (!theta_d_table.empty() && theta_d < theta_d_table.back()) {
        break;
      }
      theta_table.push_back(theta);
      theta_d_table.push_back(theta_d);
    }

    pixel_directions.assign(max_rows * max_columns, {0, 0, 0});
    const int row_offset = static_cast<int>(max_rows - rows) / 2;
    const int column_offset = static_cast<int>(max_columns - columns) / 2;

    for (int i = row_offset; i < rows + row_offset; ++i) {
      for (int j = column_offset; j < columns + column_offset; ++j) {
        const float x = (static_cast<float>(j * binning + 1) - cx) / focal_length;
        const float y = (detector_max_y - static_cast<float>(i * binning + 1) - cy) / focal_length;
        const float x1 = x * r00 + y * r01;
        const float y1 = x * r10 + y * r11;
        const float z1 = x * r20 + y * r21;
        const float x2 = -x1 / (z1 - 1);
        const float y2 = -y1 / (z1 - 1);
        const float theta_d = std::sqrt(x2 * x2 + y2 * y2);

        std::array<float, 3> direction{0, 0, 1};
        if (theta_d != 0) {
          if (
            theta_d > theta_d_max || theta_d_table.empty() || theta_d < theta_d_table.front() ||
            theta_d > theta_d_table.back()) {
            continue;
          }
          // Last table entry strictly below theta_d
          auto pos = std::lower_bound(theta_d_table.begin(), theta_d_table.end(), theta_d) -
                     theta_d_table.begin() - 1;
          if (pos < 0) {
            continue;
          }
          const float theta = (theta_table[pos + 1] - theta_table[pos]) /
                                (theta_d_table[pos + 1] - theta_d_table[pos]) *
                                (theta_d - theta_d_table[pos]) +
                              theta_table[pos];
          const float z2 = theta_d / std::tan(theta);
          const float norm = std::sqrt(x2 * x2 + y2 * y2 + z2 * z2);
          direction = {x2 / norm, y2 / norm, z2 / norm};
        }

        // Axis order and signs of the SDK's output frame
        pixel_directions[(i - row_offset) * max_columns + (j - column_offset)] = {
          direction[2], -direction[0], -direction[1]};
      }
    }
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
    case SensorModel::HESAI_AT128P:
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
      if (return_mode == "Dual" || return_mode == "LastStrongest")
        return ReturnMode::DUAL_LAST_STRONGEST;
      if (return_mode == "First") return ReturnMode::FIRST;
      if (return_mode == "FirstStrongest") return ReturnMode::DUAL_FIRST_STRONGEST;
      break;
    case SensorModel::HESAI_FTX140:
      // Dual return modes are not supported
      if (return_mode == "First") return ReturnMode::FIRST;
      if (return_mode == "Last") return ReturnMode::LAST;
      if (return_mode == "Strongest") return ReturnMode::STRONGEST;
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
    case SensorModel::HESAI_AT128P:
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
    case SensorModel::HESAI_FTX140:
      // FTX TCP API, section 3.1.3
      if (return_mode == 0) return ReturnMode::LAST;
      if (return_mode == 1) return ReturnMode::STRONGEST;
      if (return_mode == 2) return ReturnMode::DUAL_LAST_STRONGEST;
      if (return_mode == 3) return ReturnMode::FIRST;
      if (return_mode == 4) return ReturnMode::DUAL_LAST_FIRST;
      if (return_mode == 5) return ReturnMode::DUAL_FIRST_STRONGEST;
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
    case SensorModel::HESAI_AT128P:
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
    case SensorModel::HESAI_FTX140:
      // Dual return modes are not supported
      if (return_mode == ReturnMode::LAST) return 0;
      if (return_mode == ReturnMode::STRONGEST) return 1;
      if (return_mode == ReturnMode::FIRST) return 3;
      break;
    default:
      throw std::runtime_error("Unsupported sensor model: " + util::to_string(sensor_model));
  }

  return -1;
}

/// @brief Whether the given sensor model supports lidar monitor requests
/// @param sensor_model Sensor model
/// @return True if the sensor model supports lidar monitor, false otherwise
inline bool supports_lidar_monitor(const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case drivers::SensorModel::HESAI_PANDARAT128:
    case drivers::SensorModel::HESAI_PANDAR40P:
    case drivers::SensorModel::HESAI_PANDAR64:
    case drivers::SensorModel::HESAI_FTX140:
      return false;
    default:
      return true;
  }
}

/// @brief Whether the given sensor model supports functional safety
/// @param sensor_model Sensor model
/// @return True if the sensor model supports functional safety, false otherwise
inline bool supports_functional_safety(const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
    case SensorModel::HESAI_PANDARQT128:
      return true;
    default:
      return false;
  }
}

/// @brief Whether the given sensor model supports packet loss detection
/// @param sensor_model Sensor model
/// @return True if the sensor model supports packet loss detection, false otherwise
inline bool supports_packet_loss_detection(const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDAR64:
    case SensorModel::HESAI_PANDAR40P:
    case SensorModel::HESAI_PANDAR40M:
    case SensorModel::HESAI_PANDARQT128:
    case SensorModel::HESAI_PANDARAT128:
    case SensorModel::HESAI_AT128P:
    case SensorModel::HESAI_FTX140:
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
      return true;
    default:
      return false;
  }
}

/// @brief Whether the given sensor model supports blockage mask output
///
/// Blockage mask output is only supported for OT128.
///
/// The QT128 datasheet mentions support for blockage detection, but its output does not distinguish
/// between sky (no return) and blockage (too-close return). Thus, the output is not usable in the
/// same way as OT128's.
///
/// @param sensor_model Sensor model
/// @return True if the sensor model supports blockage mask output, false otherwise
inline bool supports_blockage_mask(const SensorModel & sensor_model)
{
  switch (sensor_model) {
    case SensorModel::HESAI_PANDAR128_E4X:
      return true;
    default:
      return false;
  }
}

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_COMMON__HESAI_COMMON_HPP_
