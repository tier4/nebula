// Copyright 2026 TIER IV, Inc.
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

#include <nebula_seyond_decoders/seyond_decoder.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace nebula::drivers
{

namespace
{
const uint8_t robinw_channel_mapping[48] = {
  0, 4, 8,  12, 16, 20, 24, 28, 32, 36, 40, 44, 1, 5, 9,  13, 17, 21, 25, 29, 33, 37, 41, 45,
  2, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46, 3, 7, 11, 15, 19, 23, 27, 31, 35, 39, 43, 47,
};

const uint8_t robinelite_channel_mapping[96] = {
  1,  17, 33, 48, 64, 80, 3,  19, 35, 50, 66, 82, 5,  21, 37, 52, 68, 84, 7,  23, 39, 54, 70, 86,
  9,  25, 41, 56, 72, 88, 11, 27, 43, 58, 74, 90, 13, 29, 45, 60, 76, 92, 15, 31, 47, 62, 78, 94,
  0,  2,  4,  49, 51, 53, 6,  8,  10, 55, 57, 59, 12, 14, 16, 61, 63, 65, 18, 20, 22, 67, 69, 71,
  24, 26, 28, 73, 75, 77, 30, 32, 34, 79, 81, 83, 36, 38, 40, 85, 87, 89, 42, 44, 46, 91, 93, 95,
};

// Calibration Table Constants
const int kPolygonMaxFacets = 4;
const int kPolygonTableSize = 65;
const int kInnoRobinWMaxSetNumber = 6;
const int kInnoRobinELiteMaxSetNumber = 12;
const int kMaxReceiverInSet = 8;
const int kHBHTableSize = 256;
const int kHBVTableSize = 192;
const int kEncoderTableShift = 8;
const int kEncoderTableMask = 255;
const int kEncoderTableStep = 256;
const int kPolygonMinAngle = -8192;  // -45 degrees in InnoAngleUnit (32768/180 * 45)

struct AngleHV
{
  int16_t v;
  int16_t h;
};
}  // namespace

SeyondDecoder::SeyondDecoder(
  const SeyondSensorConfiguration & config, pointcloud_callback_t pointcloud_cb,
  const SeyondCalibrationData & calibration)
: config_(config), calibration_(calibration), pointcloud_callback_(pointcloud_cb)
{
  current_scan_cloud_ = std::make_shared<NebulaPointCloud>();
}

SeyondPacketDecodeResult SeyondDecoder::unpack(const std::vector<uint8_t> & packet_data)
{
  if (packet_data.size() < sizeof(SeyondDataPacket)) {
    return {0, 0};
  }

  const auto * packet = reinterpret_cast<const SeyondDataPacket *>(packet_data.data());
  size_t initial_points = current_scan_cloud_->size();

  switch (config_.sensor_model) {
    case SeyondSensorModel::FALCON_K:
      parse_falcon_k(packet);
      break;
    case SeyondSensorModel::ROBIN_W:
    case SeyondSensorModel::ROBIN_E1X:
      parse_robin_w_e1x(packet);
      break;
    case SeyondSensorModel::HUMMINGBIRD_D1:
      parse_hummingbird_d1(packet);
      break;
    default:
      break;
  }

  size_t points_unpacked = current_scan_cloud_->size() - initial_points;

  // Publication logic (simplified: publish after each packet for now)
  if (!current_scan_cloud_->empty()) {
    pointcloud_callback_(current_scan_cloud_, packet->common.ts_start_us * 1000);
    current_scan_cloud_ = std::make_shared<NebulaPointCloud>();
  }

  return {packet->common.ts_start_us * 1000, points_unpacked};
}

void SeyondDecoder::parse_falcon_k(const SeyondDataPacket * packet)
{
  const auto * payload = reinterpret_cast<const uint8_t *>(packet) + sizeof(SeyondDataPacket);
  const auto * blocks = reinterpret_cast<const SeyondBlock *>(payload);

  const double kRadPerInnoAngleUnit = M_PI / 32768.0;
  const double kMeterPerUnit = 1.0 / 200.0;

  // Use calibration if available, else fallback to default 196
  int v_angle_diff_base =
    (calibration_.v_angle_offset != 0.0) ? static_cast<int>(calibration_.v_angle_offset) : 196;

  for (uint32_t i = 0; i < packet->item_number; ++i) {
    // Each block contains 4 points for Falcon K (single return)
    const auto & block = blocks[i];

    for (uint32_t ch = 0; ch < 4; ++ch) {
      // Accessing points via pointer because of variable block size and pedantic [1] array
      const auto * points = reinterpret_cast<const SeyondChannelPoint *>(
        payload + i * packet->item_size + sizeof(SeyondBlockHeader));
      const auto & pt = points[ch];

      if (pt.radius == 0) continue;

      int32_t ha_raw = block.header.h_angle;
      int32_t va_raw = block.header.v_angle;

      uint16_t raw_val = (config_.reflectance_mode == SeyondReflectanceMode::REFLECTIVITY)
                           ? pt.reflectance
                           : pt.reflectance;  // Falcon only has reflectance field
      uint8_t intensity = static_cast<uint8_t>((static_cast<uint32_t>(raw_val) * 255) / 4095);

      if (ch == 1) {
        ha_raw += block.header.h_angle_diff_1;
        va_raw += block.header.v_angle_diff_1 + 1 * v_angle_diff_base;
      } else if (ch == 2) {
        ha_raw += block.header.h_angle_diff_2;
        va_raw += block.header.v_angle_diff_2 + 2 * v_angle_diff_base;
      } else if (ch == 3) {
        ha_raw += block.header.h_angle_diff_3;
        va_raw += block.header.v_angle_diff_3 + 3 * v_angle_diff_base;
      }

      double ha = ha_raw * kRadPerInnoAngleUnit;
      double va = va_raw * kRadPerInnoAngleUnit;
      double radius = pt.radius * kMeterPerUnit;

      double cos_va = std::cos(va);
      float x = static_cast<float>(radius * cos_va * std::cos(ha));
      float y = static_cast<float>(-radius * cos_va * std::sin(ha));
      float z = static_cast<float>(radius * std::sin(va));

      add_point(
        x, y, z, intensity, static_cast<uint16_t>(block.header.scan_id),
        static_cast<uint32_t>(block.header.ts_10us * 10000));
    }
  }
}

void SeyondDecoder::parse_robin_w_e1x(const SeyondDataPacket * packet)
{
  const auto * payload = reinterpret_cast<const uint8_t *>(packet) + sizeof(SeyondDataPacket);

  const double kRadPerInnoAngleUnit = M_PI / 32768.0;
  const double kMeterPerUnit = 1.0 / 400.0;

  bool use_calibration = !calibration_.angle_hv_table.empty();
  int v_base = 0;
  int max_set = kInnoRobinWMaxSetNumber;
  if (config_.sensor_model == SeyondSensorModel::ROBIN_W) {
    v_base =
      (calibration_.v_angle_offset != 0.0) ? static_cast<int>(calibration_.v_angle_offset) : 240;
    max_set = kInnoRobinWMaxSetNumber;
  } else if (config_.sensor_model == SeyondSensorModel::ROBIN_E1X) {
    max_set = kInnoRobinELiteMaxSetNumber;
  }

  for (uint32_t i = 0; i < packet->item_number; ++i) {
    const auto * block_ptr =
      reinterpret_cast<const SeyondEnBlock *>(payload + i * packet->item_size);
    const auto & header = block_ptr->header;

    for (uint32_t ch = 0; ch < 4; ++ch) {
      const auto & pt = block_ptr->points[ch];
      if (pt.radius == 0) continue;

      uint16_t raw_val = (config_.reflectance_mode == SeyondReflectanceMode::REFLECTIVITY)
                           ? pt.reflectance
                           : pt.intensity;
      uint8_t intensity = static_cast<uint8_t>((static_cast<uint32_t>(raw_val) * 255) / 4095);

      int32_t ha_raw, va_raw;

      if (use_calibration) {
        // High-precision interpolation
        int polygon_mod =
          header.h_angle;  // For Robin, h_angle in header is the polygon modular angle
        int h_offset_total = polygon_mod - kPolygonMinAngle;
        if (h_offset_total < 0) h_offset_total = 0;
        int h_idx = std::min(kPolygonTableSize - 2, h_offset_total >> kEncoderTableShift);
        int h_offset = h_offset_total & kEncoderTableMask;
        int h_offset2 = kEncoderTableStep - h_offset;

        using TableType = AngleHV[kPolygonMaxFacets][kPolygonTableSize][12][kMaxReceiverInSet];
        const TableType & table =
          *reinterpret_cast<const TableType *>(calibration_.angle_hv_table.data() + 10);

        int set_num = header.scan_id % max_set;
        const AngleHV & b1 = table[header.facet][h_idx][set_num][ch];
        const AngleHV & b2 = table[header.facet][h_idx + 1][set_num][ch];

        ha_raw = (b1.h * h_offset2 + b2.h * h_offset) >> kEncoderTableShift;
        va_raw = (b1.v * h_offset2 + b2.v * h_offset) >> kEncoderTableShift;

        if (config_.sensor_model == SeyondSensorModel::ROBIN_E1X && header.scan_id >= max_set) {
          va_raw += 18;  // Elite inset line offset
        }
      } else {
        ha_raw = header.h_angle;
        va_raw = header.v_angle;

        if (ch == 1) {
          ha_raw += header.h_angle_diff_1;
          va_raw += header.v_angle_diff_1 + 1 * v_base;
        } else if (ch == 2) {
          ha_raw += header.h_angle_diff_2;
          va_raw += header.v_angle_diff_2 + 2 * v_base;
        } else if (ch == 3) {
          ha_raw += header.h_angle_diff_3;
          va_raw += header.v_angle_diff_3 + 3 * v_base;
        }
      }

      double ha = ha_raw * kRadPerInnoAngleUnit;
      double va = va_raw * kRadPerInnoAngleUnit;
      double radius = pt.radius * kMeterPerUnit;

      double cos_va = std::cos(va);
      float x = static_cast<float>(radius * cos_va * std::cos(ha));
      float y = static_cast<float>(-radius * cos_va * std::sin(ha));
      float z = static_cast<float>(radius * std::sin(va));

      uint16_t physical_channel = ch;
      if (config_.sensor_model == SeyondSensorModel::ROBIN_W) {
        physical_channel =
          robinw_channel_mapping[(header.scan_id % 12) * 4 + ch] + header.facet * 48;
      } else if (config_.sensor_model == SeyondSensorModel::ROBIN_E1X) {
        physical_channel = robinelite_channel_mapping[(header.scan_id % 24) * 4 + ch];
      }

      add_point(
        x, y, z, intensity, physical_channel, static_cast<uint32_t>(header.ts_10us * 10000));
    }
  }
}

void SeyondDecoder::parse_hummingbird_d1(const SeyondDataPacket * packet)
{
  const auto * payload = reinterpret_cast<const uint8_t *>(packet) + sizeof(SeyondDataPacket);
  bool use_calibration = !calibration_.angle_hv_table.empty();

  for (uint32_t i = 0; i < packet->item_number; ++i) {
    const auto * block_ptr =
      reinterpret_cast<const SeyondCoBlock *>(payload + i * packet->item_size);
    const auto & header = block_ptr->header;
    uint16_t base_channel = static_cast<uint16_t>(header.scan_id % 4) * 8;  // Simplified mapping

    for (uint32_t ch = 0; ch < 8; ++ch) {
      const auto & pt = block_ptr->points[ch];
      if (pt.radius == 0) continue;

      double ha, va;
      if (use_calibration) {
        using TableType = AngleHV[kHBVTableSize][kHBHTableSize];
        const TableType & table =
          *reinterpret_cast<const TableType *>(calibration_.angle_hv_table.data() + 10);
        ha = table[header.scan_id][header.scan_idx + ch].h * (M_PI / 32768.0);
        va = table[header.scan_id][header.scan_idx + ch].v * (M_PI / 32768.0);
      } else {
        ha = header.p_angle * (M_PI / 32768.0);
        va = header.g_angle * (M_PI / 32768.0);
      }

      double radius = pt.radius * (1.0 / 400.0);

      double cos_va = std::cos(va);
      float x = static_cast<float>(radius * cos_va * std::cos(ha));
      float y = static_cast<float>(-radius * cos_va * std::sin(ha));
      float z = static_cast<float>(radius * std::sin(va));

      uint8_t intensity = static_cast<uint8_t>((static_cast<uint32_t>(pt.refl) * 255) / 4095);

      add_point(
        x, y, z, intensity, static_cast<uint16_t>(base_channel + ch),
        static_cast<uint32_t>(header.ts_10us * 10000));
    }
  }
}

void SeyondDecoder::add_point(
  float x, float y, float z, uint8_t intensity, uint16_t channel, uint32_t timestamp_ns)
{
  NebulaPoint point{};
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.return_type = 0;
  point.channel = channel;
  point.azimuth = 0.0f;
  point.elevation = 0.0f;
  point.distance = std::sqrt(x * x + y * y + z * z);
  point.time_stamp = timestamp_ns;

  current_scan_cloud_->emplace_back(point);
}

}  // namespace nebula::drivers
