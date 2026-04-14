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

#include <nebula_seyond_decoders/falcon_nps_adjustment.hpp>
#include <nebula_seyond_decoders/robin_e2x_nps_adjustment.hpp>
#include <nebula_seyond_decoders/robin_w_nps_adjustment.hpp>
#include <nebula_seyond_decoders/seyond_decoder.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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

const int kPolygonMaxFacets = 4;
const int kPolygonTableSize = 65;
const int kRobinWMaxSetNumber = 6;
const int kRobinELiteMaxSetNumber = 12;
const int kRobinE2XMaxSetNumber = 24;
const int kMaxReceiverInSet = 8;
const int kHBHTableSize = 256;
const int kHBVTableSize = 192;
const int kEncoderTableShift = 8;
const int kEncoderTableMask = 255;
const int kEncoderTableStep = 256;
const int kPolygonMinAngle = -8192;
const int kRobinNpsTableShift = 9;
const int kRobinNpsTableStep = 1 << kRobinNpsTableShift;
const int kRobinNpsTableHalfStep = 1 << (kRobinNpsTableShift - 1);
const int kRobinNpsTableMask = kRobinNpsTableStep - 1;
const int kRobinNpsTableSize = 64;
const int kRobinNpsEffectiveHalfSize = 27;
const double kRobinNpsAdjustmentUnitMeters = 0.001;

const size_t kAngleHVTableHeaderSize = 10;
const size_t kRobinWTableMinSize =
  kAngleHVTableHeaderSize + sizeof(int16_t) * 2 * kPolygonMaxFacets * kPolygonTableSize *
                              kRobinWMaxSetNumber * kMaxReceiverInSet;
const size_t kRobinELiteTableMinSize =
  kAngleHVTableHeaderSize + sizeof(int16_t) * 2 * kPolygonMaxFacets * kPolygonTableSize *
                              kRobinELiteMaxSetNumber * kMaxReceiverInSet;
const size_t kRobinE2XTableMinSize =
  kAngleHVTableHeaderSize + sizeof(int16_t) * 2 * kPolygonMaxFacets * kPolygonTableSize *
                              kRobinE2XMaxSetNumber * kMaxReceiverInSet;
const size_t kHummingbirdTableMinSize =
  kAngleHVTableHeaderSize + sizeof(int16_t) * 2 * kHBVTableSize * kHBHTableSize;

const int kFalconKVAngleDiffBase = 196;
const int kRobinWVAngleDiffBase = 240;
const int kRobinE1XInsetLineOffset = 18;
constexpr double packet_angle_units_per_degree = 32768.0 / 180.0;
const uint16_t seyond_data_packet_magic_number = 0x176A;
const uint8_t kItemTypeSpherePointCloud = 1;
const uint8_t kItemTypeRobinESpherePointCloud = 5;
const uint8_t kItemTypeRobinWSpherePointCloud = 7;
const uint8_t kItemTypeRobinWCompactPointCloud = 13;
const uint8_t kItemTypeRobinE2XCompactPointCloud = 19;
const uint8_t kItemTypeHummingbirdCompactPointCloud = 22;
constexpr size_t kCompactChannelCount = 8;
const int kFalconNpsTableShift = 9;
const int kFalconNpsTableSizeH = 64;
const int kFalconNpsTableSizeV = 16;
const int kFalconNpsEffectiveHalfSizeH = 22;
const int kFalconNpsEffectiveHalfSizeV = 6;
const double kFalconNpsAdjustmentUnitMeters = 0.0025;

struct AngleHV
{
  int16_t v;
  int16_t h;
};

struct RobinAdjustment
{
  double x;
  double y;
  double z;
};

struct FalconAdjustment
{
  double x;
  double z;
};

bool is_supported_compact_item_size(uint16_t item_size)
{
  const size_t payload_size = item_size - sizeof(SeyondCoBlockHeader);
  return item_size >=
           sizeof(SeyondCoBlockHeader) + sizeof(SeyondCoChannelPoint) * kCompactChannelCount &&
         payload_size % (sizeof(SeyondCoChannelPoint) * kCompactChannelCount) == 0;
}

bool is_robin_inside_compact_fov(const AngleHV & angle)
{
  constexpr int fov_left = static_cast<int>(-60.0 * packet_angle_units_per_degree);
  constexpr int fov_right = static_cast<int>(60.0 * packet_angle_units_per_degree);
  return angle.h >= fov_left && angle.h <= fov_right;
}

bool is_hummingbird_inside_compact_fov(const AngleHV & angle)
{
  constexpr int fov_left = static_cast<int>(-70.0 * packet_angle_units_per_degree);
  constexpr int fov_right = static_cast<int>(70.0 * packet_angle_units_per_degree);
  constexpr int fov_low = static_cast<int>(-50.0 * packet_angle_units_per_degree);
  constexpr int fov_high = static_cast<int>(50.0 * packet_angle_units_per_degree);
  return angle.h >= fov_left && angle.h <= fov_right && angle.v >= fov_low && angle.v <= fov_high;
}

size_t compact_return_count(uint16_t item_size)
{
  if (!is_supported_compact_item_size(item_size)) {
    return 0;
  }
  return (item_size - sizeof(SeyondCoBlockHeader)) /
         (sizeof(SeyondCoChannelPoint) * kCompactChannelCount);
}

template <int MaxSetNumber>
std::array<AngleHV, kCompactChannelCount> interpolate_robin_compact_angles_impl(
  const SeyondCoBlockHeader & header, const std::vector<uint8_t> & angle_hv_table,
  int max_set_number)
{
  std::array<AngleHV, kCompactChannelCount> angles{};
  if (angle_hv_table.size() < kAngleHVTableHeaderSize) {
    return angles;
  }

  int h_offset_total = header.p_angle - kPolygonMinAngle;
  if (h_offset_total < 0) {
    h_offset_total = 0;
  }

  int h_idx = h_offset_total >> kEncoderTableShift;
  int h_offset = h_offset_total & kEncoderTableMask;
  int h_offset2 = kEncoderTableStep - h_offset;
  h_idx = std::min(kPolygonTableSize - 2, h_idx);

  int set_num = header.scan_id % max_set_number;
  using TableType = AngleHV[kPolygonMaxFacets][kPolygonTableSize][MaxSetNumber][kMaxReceiverInSet];
  const auto & table =
    *reinterpret_cast<const TableType *>(angle_hv_table.data() + kAngleHVTableHeaderSize);

  for (size_t channel = 0; channel < kCompactChannelCount; ++channel) {
    const auto & b1 = table[header.facet][h_idx][set_num][channel];
    const auto & b2 = table[header.facet][h_idx + 1][set_num][channel];
    angles[channel].h =
      static_cast<int16_t>((b1.h * h_offset2 + b2.h * h_offset) >> kEncoderTableShift);
    angles[channel].v =
      static_cast<int16_t>((b1.v * h_offset2 + b2.v * h_offset) >> kEncoderTableShift);
    if (header.scan_id >= static_cast<uint32_t>(max_set_number)) {
      angles[channel].v = static_cast<int16_t>(angles[channel].v + kRobinE1XInsetLineOffset);
    }
  }

  return angles;
}

std::array<AngleHV, kCompactChannelCount> interpolate_robin_compact_angles(
  const SeyondCoBlockHeader & header, const std::vector<uint8_t> & angle_hv_table,
  int max_set_number)
{
  switch (max_set_number) {
    case kRobinWMaxSetNumber:
      return interpolate_robin_compact_angles_impl<kRobinWMaxSetNumber>(
        header, angle_hv_table, max_set_number);
    case kRobinE2XMaxSetNumber:
      return interpolate_robin_compact_angles_impl<kRobinE2XMaxSetNumber>(
        header, angle_hv_table, max_set_number);
    default:
      return {};
  }
}

FalconAdjustment lookup_falcon_adjustment(int h_angle, int v_angle, uint32_t channel)
{
  FalconAdjustment adjustment{};
  if (channel >= 4) {
    return adjustment;
  }

  int v_index = (v_angle >> kFalconNpsTableShift) + kFalconNpsEffectiveHalfSizeV;
  int h_index = (h_angle >> kFalconNpsTableShift) + kFalconNpsEffectiveHalfSizeH;
  v_index &= (kFalconNpsTableSizeV - 1);
  h_index &= (kFalconNpsTableSizeH - 1);

  const auto quantize = [](double value) {
    return static_cast<int>(std::floor(value / kFalconNpsAdjustmentUnitMeters + 0.5));
  };

  adjustment.x =
    quantize(falcon_ps_to_nps_adjustment[0][channel][v_index][h_index]) *
    kFalconNpsAdjustmentUnitMeters;
  adjustment.z =
    quantize(falcon_ps_to_nps_adjustment[1][channel][v_index][h_index]) *
    kFalconNpsAdjustmentUnitMeters;
  return adjustment;
}

RobinAdjustment interpolate_robin_w_adjustment(int h_angle, uint32_t scan_id)
{
  RobinAdjustment adjustment{};
  if (scan_id >= 192) {
    return adjustment;
  }

  int adjusted_h_angle = h_angle + (kRobinNpsEffectiveHalfSize << kRobinNpsTableShift);
  int h_index = adjusted_h_angle >> kRobinNpsTableShift;
  h_index &= (kRobinNpsTableSize - 1);
  if (h_index > kRobinNpsTableSize - 2) {
    h_index = kRobinNpsTableSize - 2;
  }

  const int h_offset = adjusted_h_angle & kRobinNpsTableMask;
  const int h_offset2 = kRobinNpsTableStep - h_offset;

  const auto interpolate_axis = [&](size_t axis) {
    const int u =
      static_cast<int>(std::floor(robin_w_ps_to_nps_adjustment[axis][scan_id][h_index] + 0.5));
    const int v =
      static_cast<int>(std::floor(robin_w_ps_to_nps_adjustment[axis][scan_id][h_index + 1] + 0.5));
    const int blended =
      (u * h_offset2 + v * h_offset + kRobinNpsTableHalfStep) >> kRobinNpsTableShift;
    return blended * kRobinNpsAdjustmentUnitMeters;
  };

  adjustment.x = interpolate_axis(0);
  adjustment.y = interpolate_axis(1);
  adjustment.z = interpolate_axis(2);
  return adjustment;
}

RobinAdjustment interpolate_robin_e2x_adjustment(int h_angle)
{
  RobinAdjustment adjustment{};

  int adjusted_h_angle = h_angle + (kRobinNpsEffectiveHalfSize << kRobinNpsTableShift);
  int h_index = adjusted_h_angle >> kRobinNpsTableShift;
  h_index &= (kRobinNpsTableSize - 1);
  if (h_index > kRobinNpsTableSize - 2) {
    h_index = kRobinNpsTableSize - 2;
  }

  const int h_offset = adjusted_h_angle & kRobinNpsTableMask;
  const int h_offset2 = kRobinNpsTableStep - h_offset;

  const auto interpolate_axis = [&](size_t axis) {
    const int u = static_cast<int>(std::floor(robin_e2x_ps_to_nps_adjustment[axis][h_index] + 0.5));
    const int v =
      static_cast<int>(std::floor(robin_e2x_ps_to_nps_adjustment[axis][h_index + 1] + 0.5));
    const int blended =
      (u * h_offset2 + v * h_offset + kRobinNpsTableHalfStep) >> kRobinNpsTableShift;
    return blended * kRobinNpsAdjustmentUnitMeters;
  };

  adjustment.x = interpolate_axis(0);
  adjustment.y = interpolate_axis(1);
  adjustment.z = interpolate_axis(2);
  return adjustment;
}

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
  if (packet->common.magic_number != seyond_data_packet_magic_number) {
    return {0, 0};
  }
  if (packet->common.size < sizeof(SeyondDataPacket) || packet->common.size > packet_data.size()) {
    return {0, 0};
  }

  const auto payload_size = static_cast<size_t>(packet->common.size) - sizeof(SeyondDataPacket);
  const auto required_payload_size =
    static_cast<uint64_t>(packet->item_number) * static_cast<uint64_t>(packet->item_size);
  if (required_payload_size > payload_size) {
    return {0, 0};
  }

  bool supported_layout = false;
  switch (config_.sensor_model) {
    case SeyondSensorModel::FALCON_K:
      supported_layout =
        packet->type == kItemTypeSpherePointCloud && packet->item_size == sizeof(SeyondBlock);
      break;
    case SeyondSensorModel::ROBIN_W:
      supported_layout = (packet->type == kItemTypeRobinWSpherePointCloud &&
                          packet->item_size == sizeof(SeyondEnBlock)) ||
                         (packet->type == kItemTypeRobinWCompactPointCloud &&
                          is_supported_compact_item_size(packet->item_size));
      break;
    case SeyondSensorModel::ROBIN_E1X:
      supported_layout = (packet->type == kItemTypeRobinESpherePointCloud &&
                          packet->item_size == sizeof(SeyondEnBlock)) ||
                         (packet->type == kItemTypeRobinE2XCompactPointCloud &&
                          is_supported_compact_item_size(packet->item_size));
      break;
    case SeyondSensorModel::HUMMINGBIRD_D1:
      supported_layout = packet->type == kItemTypeHummingbirdCompactPointCloud &&
                         is_supported_compact_item_size(packet->item_size);
      break;
    default:
      break;
  }

  if (!supported_layout) {
    return {packet->common.ts_start_us * 1000, 0};
  }

  size_t initial_points = current_scan_cloud_->size();

  switch (config_.sensor_model) {
    case SeyondSensorModel::FALCON_K:
      parse_falcon_k(packet);
      break;
    case SeyondSensorModel::ROBIN_W:
      if (packet->type == kItemTypeRobinWCompactPointCloud) {
        parse_robin_compact(packet);
      } else {
        parse_robin_w_e1x(packet);
      }
      break;
    case SeyondSensorModel::ROBIN_E1X:
      if (packet->type == kItemTypeRobinE2XCompactPointCloud) {
        parse_robin_compact(packet);
      } else {
        parse_robin_w_e1x(packet);
      }
      break;
    case SeyondSensorModel::HUMMINGBIRD_D1:
      parse_hummingbird_d1(packet);
      break;
    default:
      break;
  }

  size_t points_unpacked = current_scan_cloud_->size() - initial_points;
  const uint64_t base_ts_ns = packet->common.ts_start_us * 1000;

  if (packet->is_last_sub_frame && !current_scan_cloud_->empty()) {
    pointcloud_callback_(current_scan_cloud_, base_ts_ns);
    current_scan_cloud_ = std::make_shared<NebulaPointCloud>();
  }

  return {base_ts_ns, points_unpacked};
}

void SeyondDecoder::parse_falcon_k(const SeyondDataPacket * packet)
{
  const auto * payload = reinterpret_cast<const uint8_t *>(packet) + sizeof(SeyondDataPacket);
  const auto * blocks = reinterpret_cast<const SeyondBlock *>(payload);

  const double radians_per_packet_angle_unit = M_PI / 32768.0;
  const double kMeterPerUnit = packet->long_distance_mode ? (1.0 / 100.0) : (1.0 / 200.0);

  int v_angle_diff_base = (calibration_.v_angle_offset != 0.0)
                            ? static_cast<int>(calibration_.v_angle_offset)
                            : kFalconKVAngleDiffBase;

  for (uint32_t i = 0; i < packet->item_number; ++i) {
    const auto & block = blocks[i];

    for (uint32_t ch = 0; ch < 4; ++ch) {
      const auto * points = reinterpret_cast<const SeyondChannelPoint *>(
        payload + i * packet->item_size + sizeof(SeyondBlockHeader));
      const auto & pt = points[ch];

      if (pt.radius == 0) continue;

      int32_t ha_raw = block.header.h_angle;
      int32_t va_raw = block.header.v_angle;

      uint8_t intensity = static_cast<uint8_t>((static_cast<uint32_t>(pt.refl) * 255) / 255);

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

      double ha = ha_raw * radians_per_packet_angle_unit;
      double va = va_raw * radians_per_packet_angle_unit;
      double radius = pt.radius * kMeterPerUnit;

      double cos_va = std::cos(va);
      float x = static_cast<float>(radius * cos_va * std::cos(ha));
      float y = static_cast<float>(-radius * cos_va * std::sin(ha));
      float z = static_cast<float>(radius * std::sin(va));
      const auto adjustment = lookup_falcon_adjustment(ha_raw, va_raw, ch);
      x = static_cast<float>(x + adjustment.z);
      z = static_cast<float>(z + adjustment.x);

      add_point(
        x, y, z, intensity, static_cast<uint16_t>(block.header.scan_id),
        static_cast<uint32_t>(block.header.ts_10us * 10000));
    }
  }
}

void SeyondDecoder::parse_robin_w_e1x(const SeyondDataPacket * packet)
{
  const auto * payload = reinterpret_cast<const uint8_t *>(packet) + sizeof(SeyondDataPacket);

  const double radians_per_packet_angle_unit = M_PI / 32768.0;
  const double kMeterPerUnit = 1.0 / 400.0;

  const bool is_robin_w = (config_.sensor_model == SeyondSensorModel::ROBIN_W);
  const bool is_robin_e1x = (config_.sensor_model == SeyondSensorModel::ROBIN_E1X);

  int v_base = 0;
  int max_set = kRobinWMaxSetNumber;
  size_t table_min_size = kRobinWTableMinSize;

  if (is_robin_w) {
    v_base = (calibration_.v_angle_offset != 0.0) ? static_cast<int>(calibration_.v_angle_offset)
                                                  : kRobinWVAngleDiffBase;
    max_set = kRobinWMaxSetNumber;
    table_min_size = kRobinWTableMinSize;
  } else if (is_robin_e1x) {
    max_set = kRobinELiteMaxSetNumber;
    table_min_size = kRobinELiteTableMinSize;
  }

  bool use_calibration = (calibration_.angle_hv_table.size() >= table_min_size);

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
        int h_offset_total = header.h_angle - kPolygonMinAngle;
        if (h_offset_total < 0) h_offset_total = 0;
        int h_idx = std::min(kPolygonTableSize - 2, h_offset_total >> kEncoderTableShift);
        int h_offset = h_offset_total & kEncoderTableMask;
        int h_offset2 = kEncoderTableStep - h_offset;
        int set_num = header.scan_id % max_set;
        if (is_robin_w) {
          using RobinWTableType = AngleHV[kPolygonMaxFacets][kPolygonTableSize]
                                         [kRobinWMaxSetNumber][kMaxReceiverInSet];
          const auto & table =
            *reinterpret_cast<const RobinWTableType *>(calibration_.angle_hv_table.data() + 10);
          const AngleHV & b1 = table[header.facet][h_idx][set_num][ch];
          const AngleHV & b2 = table[header.facet][h_idx + 1][set_num][ch];
          ha_raw = (b1.h * h_offset2 + b2.h * h_offset) >> kEncoderTableShift;
          va_raw = (b1.v * h_offset2 + b2.v * h_offset) >> kEncoderTableShift;
        } else {
          using RobinETableType = AngleHV[kPolygonMaxFacets][kPolygonTableSize]
                                         [kRobinELiteMaxSetNumber][kMaxReceiverInSet];
          const auto & table =
            *reinterpret_cast<const RobinETableType *>(calibration_.angle_hv_table.data() + 10);
          const AngleHV & b1 = table[header.facet][h_idx][set_num][ch];
          const AngleHV & b2 = table[header.facet][h_idx + 1][set_num][ch];
          ha_raw = (b1.h * h_offset2 + b2.h * h_offset) >> kEncoderTableShift;
          va_raw = (b1.v * h_offset2 + b2.v * h_offset) >> kEncoderTableShift;
        }

        if (is_robin_e1x && header.scan_id >= static_cast<uint32_t>(max_set)) {
          va_raw += kRobinE1XInsetLineOffset;
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

      double ha = ha_raw * radians_per_packet_angle_unit;
      double va = va_raw * radians_per_packet_angle_unit;
      double radius = pt.radius * kMeterPerUnit;

      double cos_va = std::cos(va);
      float x = static_cast<float>(radius * cos_va * std::cos(ha));
      float y = static_cast<float>(-radius * cos_va * std::sin(ha));
      float z = static_cast<float>(radius * std::sin(va));

      uint16_t physical_channel = static_cast<uint16_t>(ch);
      if (is_robin_w) {
        size_t map_idx = static_cast<size_t>((header.scan_id % 12) * 4 + ch);
        if (map_idx < sizeof(robinw_channel_mapping)) {
          physical_channel = static_cast<uint16_t>(robinw_channel_mapping[map_idx]) +
                             static_cast<uint16_t>(header.facet * 48);
        }
      } else if (is_robin_e1x) {
        size_t map_idx = static_cast<size_t>((header.scan_id % 24) * 4 + ch);
        if (map_idx < sizeof(robinelite_channel_mapping)) {
          physical_channel = robinelite_channel_mapping[map_idx];
        }
      }
      if (is_robin_w) {
        const auto adjustment = interpolate_robin_w_adjustment(ha_raw, physical_channel);
        x = static_cast<float>(x + adjustment.z);
        y = static_cast<float>(y - adjustment.y);
        z = static_cast<float>(z + adjustment.x);
        y = -y;
        z = -z;
      }

      add_point(
        x, y, z, intensity, physical_channel, static_cast<uint32_t>(header.ts_10us * 10000));
    }
  }
}

void SeyondDecoder::parse_robin_compact(const SeyondDataPacket * packet)
{
  const auto * payload = reinterpret_cast<const uint8_t *>(packet) + sizeof(SeyondDataPacket);
  const auto return_count = compact_return_count(packet->item_size);
  if (return_count == 0) {
    return;
  }

  const bool is_robin_w_compact = packet->type == kItemTypeRobinWCompactPointCloud;
  const bool is_robin_e2x_compact = packet->type == kItemTypeRobinE2XCompactPointCloud;
  const int max_set_number =
    is_robin_w_compact ? kRobinWMaxSetNumber : kRobinE2XMaxSetNumber;
  const size_t table_min_size = is_robin_w_compact ? kRobinWTableMinSize : kRobinE2XTableMinSize;
  const bool use_calibration = calibration_.angle_hv_table.size() >= table_min_size;
  const double radians_per_packet_angle_unit = M_PI / 32768.0;
  const double kMeterPerUnit = 1.0 / 400.0;

  for (uint32_t i = 0; i < packet->item_number; ++i) {
    const auto * block_ptr =
      reinterpret_cast<const SeyondCoBlock *>(payload + i * packet->item_size);
    const auto & header = block_ptr->header;
    const auto * points = reinterpret_cast<const SeyondCoChannelPoint *>(
      payload + i * packet->item_size + sizeof(SeyondCoBlockHeader));

    std::array<AngleHV, kCompactChannelCount> angles{};
    if (use_calibration) {
      angles =
        interpolate_robin_compact_angles(header, calibration_.angle_hv_table, max_set_number);
    }

    for (size_t channel = 0; channel < kCompactChannelCount; ++channel) {
      for (size_t return_idx = 0; return_idx < return_count; ++return_idx) {
        const auto & pt = points[channel + return_idx * kCompactChannelCount];
        if (pt.radius == 0) {
          continue;
        }

        double ha = 0.0;
        double va = 0.0;
        if (use_calibration) {
          ha = static_cast<double>(angles[channel].h) * radians_per_packet_angle_unit;
          va = static_cast<double>(angles[channel].v) * radians_per_packet_angle_unit;
          if (!is_robin_inside_compact_fov(angles[channel])) {
            continue;
          }
        } else {
          ha = static_cast<double>(header.p_angle) * radians_per_packet_angle_unit;
          va = static_cast<double>(header.g_angle) * radians_per_packet_angle_unit;
        }

        double radius = pt.radius * kMeterPerUnit;
        double cos_va = std::cos(va);
        float x = static_cast<float>(radius * cos_va * std::cos(ha));
        float y = static_cast<float>(-radius * cos_va * std::sin(ha));
        float z = static_cast<float>(radius * std::sin(va));
        uint16_t physical_channel =
          static_cast<uint16_t>(header.scan_id * kCompactChannelCount + channel);
        if (is_robin_w_compact) {
          const size_t map_idx =
            static_cast<size_t>(header.scan_id * kCompactChannelCount + channel);
          if (map_idx < sizeof(robinw_channel_mapping)) {
            physical_channel = static_cast<uint16_t>(robinw_channel_mapping[map_idx]) +
                               static_cast<uint16_t>(header.facet * 48);
          }
          const auto adjustment = interpolate_robin_w_adjustment(
            use_calibration ? angles[channel].h : header.p_angle, physical_channel);
          x = static_cast<float>(x + adjustment.z);
          y = static_cast<float>(y - adjustment.y);
          z = static_cast<float>(z + adjustment.x);
          y = -y;
          z = -z;
        } else if (is_robin_e2x_compact) {
          const int h_angle_raw = use_calibration ? angles[channel].h : header.p_angle;
          const auto adjustment = interpolate_robin_e2x_adjustment(h_angle_raw);
          x = static_cast<float>(x + adjustment.z);
          y = static_cast<float>(y - adjustment.y);
          z = static_cast<float>(z + adjustment.x);
        }
        uint8_t intensity = static_cast<uint8_t>((static_cast<uint32_t>(pt.refl) * 255) / 4095);

        add_point(
          x, y, z, intensity, physical_channel, static_cast<uint32_t>(header.ts_10us * 10000));
      }
    }
  }
}

void SeyondDecoder::parse_hummingbird_d1(const SeyondDataPacket * packet)
{
  const auto * payload = reinterpret_cast<const uint8_t *>(packet) + sizeof(SeyondDataPacket);
  bool use_calibration = (calibration_.angle_hv_table.size() >= kHummingbirdTableMinSize);
  const auto return_count = compact_return_count(packet->item_size);
  if (return_count == 0) {
    return;
  }

  for (uint32_t i = 0; i < packet->item_number; ++i) {
    const auto * block_ptr =
      reinterpret_cast<const SeyondCoBlock *>(payload + i * packet->item_size);
    const auto & header = block_ptr->header;
    const auto * points = reinterpret_cast<const SeyondCoChannelPoint *>(
      payload + i * packet->item_size + sizeof(SeyondCoBlockHeader));
    uint16_t base_channel = static_cast<uint16_t>(header.scan_id % 4) * 8;

    for (size_t ch = 0; ch < kCompactChannelCount; ++ch) {
      for (size_t return_idx = 0; return_idx < return_count; ++return_idx) {
        const auto & pt = points[ch + return_idx * kCompactChannelCount];
        if (pt.radius == 0) continue;

        double ha, va;
        if (use_calibration) {
          using TableType = AngleHV[kHBVTableSize][kHBHTableSize];
          const TableType & table =
            *reinterpret_cast<const TableType *>(calibration_.angle_hv_table.data() + 10);
          const AngleHV & angle = table[header.scan_id][header.scan_idx + ch];
          if (!is_hummingbird_inside_compact_fov(angle)) continue;
          ha = angle.h * (M_PI / 32768.0);
          va = angle.v * (M_PI / 32768.0);
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
}

void SeyondDecoder::add_point(
  float x, float y, float z, uint8_t intensity, uint16_t channel, uint32_t timestamp_ns)
{
  const float azimuth_deg =
    normalize_angle(std::atan2(-y, x) * (180.0f / static_cast<float>(M_PI)), 360.0f);
  const float elevation_deg =
    std::atan2(z, std::sqrt(x * x + y * y)) * (180.0f / static_cast<float>(M_PI));

  const bool azimuth_full_circle = (config_.fov.azimuth.start == config_.fov.azimuth.end);
  const bool elevation_full_circle = (config_.fov.elevation.start == config_.fov.elevation.end);

  if (
    !azimuth_full_circle &&
    !angle_is_between(config_.fov.azimuth.start, config_.fov.azimuth.end, azimuth_deg)) {
    return;
  }
  if (
    !elevation_full_circle &&
    !angle_is_between(config_.fov.elevation.start, config_.fov.elevation.end, elevation_deg)) {
    return;
  }

  NebulaPoint point{};
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.return_type = 0;
  point.channel = channel;
  point.azimuth = azimuth_deg;
  point.elevation = elevation_deg;
  point.distance = std::sqrt(x * x + y * y + z * z);
  point.time_stamp = timestamp_ns;

  current_scan_cloud_->emplace_back(point);
}

}  // namespace nebula::drivers
