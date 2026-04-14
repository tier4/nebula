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
#include <limits>
#include <memory>
#include <stdexcept>
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

const int polygon_max_facets = 4;
const int polygon_table_size = 65;
const int robinw_max_set_number = 6;
const int robine_lite_max_set_number = 12;
const int robine2x_max_set_number = 24;
const int max_receiver_in_set = 8;
const int hummingbird_table_width = 256;
const int hummingbird_table_height = 192;
const int encoder_table_shift = 8;
const int encoder_table_mask = 255;
const int encoder_table_step = 256;
const int polygon_min_angle = -8192;
const int robin_nps_table_shift = 9;
const int robin_nps_table_step = 1 << robin_nps_table_shift;
const int robin_nps_table_half_step = 1 << (robin_nps_table_shift - 1);
const int robin_nps_table_mask = robin_nps_table_step - 1;
const int robin_nps_table_size = 64;
const int robin_nps_effective_half_size = 27;
const double robin_nps_adjustment_unit_meters = 0.001;

const size_t angle_hv_table_header_size = 10;
const size_t robinw_table_min_size =
  angle_hv_table_header_size + sizeof(int16_t) * 2 * polygon_max_facets * polygon_table_size *
                                 robinw_max_set_number * max_receiver_in_set;
const size_t robine_lite_table_min_size =
  angle_hv_table_header_size + sizeof(int16_t) * 2 * polygon_max_facets * polygon_table_size *
                                 robine_lite_max_set_number * max_receiver_in_set;
const size_t robine2x_table_min_size =
  angle_hv_table_header_size + sizeof(int16_t) * 2 * polygon_max_facets * polygon_table_size *
                                 robine2x_max_set_number * max_receiver_in_set;
const size_t hummingbird_table_min_size = angle_hv_table_header_size + sizeof(int16_t) * 2 *
                                                                         hummingbird_table_height *
                                                                         hummingbird_table_width;

const int falconk_v_angle_diff_base = 196;
const int robinw_v_angle_diff_base = 240;
const int robine1x_inset_line_offset = 18;
constexpr double packet_angle_units_per_degree = 32768.0 / 180.0;
const uint16_t seyond_data_packet_magic_number = 0x176A;
const uint8_t item_type_sphere_pointcloud = 1;
const uint8_t item_type_robine_sphere_pointcloud = 5;
const uint8_t item_type_robinw_sphere_pointcloud = 7;
const uint8_t item_type_robinw_compact_pointcloud = 13;
const uint8_t item_type_robine2x_compact_pointcloud = 19;
const uint8_t item_type_hummingbird_compact_pointcloud = 22;
constexpr size_t compact_channel_count = 8;
const int falcon_nps_table_shift = 9;
const int falcon_nps_table_size_h = 64;
const int falcon_nps_table_size_v = 16;
const int falcon_nps_effective_half_size_h = 22;
const int falcon_nps_effective_half_size_v = 6;
const double falcon_nps_adjustment_unit_meters = 0.0025;

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
           sizeof(SeyondCoBlockHeader) + sizeof(SeyondCoChannelPoint) * compact_channel_count &&
         payload_size % (sizeof(SeyondCoChannelPoint) * compact_channel_count) == 0;
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
         (sizeof(SeyondCoChannelPoint) * compact_channel_count);
}

uint32_t to_scan_relative_timestamp_ns(
  uint64_t scan_start_timestamp_ns, uint64_t packet_start_timestamp_ns,
  uint32_t packet_relative_timestamp_ns)
{
  const uint64_t packet_offset_ns = packet_start_timestamp_ns >= scan_start_timestamp_ns
                                      ? packet_start_timestamp_ns - scan_start_timestamp_ns
                                      : 0;
  const uint64_t point_timestamp_ns = packet_offset_ns + packet_relative_timestamp_ns;
  return static_cast<uint32_t>(
    std::min<uint64_t>(point_timestamp_ns, std::numeric_limits<uint32_t>::max()));
}

uint64_t packet_timestamp_us_to_ns(double packet_timestamp_us)
{
  if (!std::isfinite(packet_timestamp_us) || packet_timestamp_us < 0.0) {
    return 0;
  }

  const long double packet_timestamp_ns = static_cast<long double>(packet_timestamp_us) * 1000.0L;
  return static_cast<uint64_t>(std::min<long double>(
    packet_timestamp_ns, static_cast<long double>(std::numeric_limits<uint64_t>::max())));
}

template <int MaxSetNumber>
std::array<AngleHV, compact_channel_count> interpolate_robin_compact_angles_impl(
  const SeyondCoBlockHeader & header, const std::vector<uint8_t> & angle_hv_table,
  int max_set_number)
{
  std::array<AngleHV, compact_channel_count> angles{};
  if (angle_hv_table.size() < angle_hv_table_header_size) {
    return angles;
  }

  int h_offset_total = header.p_angle - polygon_min_angle;
  if (h_offset_total < 0) {
    h_offset_total = 0;
  }

  int h_idx = h_offset_total >> encoder_table_shift;
  int h_offset = h_offset_total & encoder_table_mask;
  int h_offset2 = encoder_table_step - h_offset;
  h_idx = std::min(polygon_table_size - 2, h_idx);

  int set_num = header.scan_id % max_set_number;
  using TableType =
    AngleHV[polygon_max_facets][polygon_table_size][MaxSetNumber][max_receiver_in_set];
  const auto & table =
    *reinterpret_cast<const TableType *>(angle_hv_table.data() + angle_hv_table_header_size);

  for (size_t channel = 0; channel < compact_channel_count; ++channel) {
    const auto & b1 = table[header.facet][h_idx][set_num][channel];
    const auto & b2 = table[header.facet][h_idx + 1][set_num][channel];
    angles[channel].h =
      static_cast<int16_t>((b1.h * h_offset2 + b2.h * h_offset) >> encoder_table_shift);
    angles[channel].v =
      static_cast<int16_t>((b1.v * h_offset2 + b2.v * h_offset) >> encoder_table_shift);
    if (header.scan_id >= static_cast<uint32_t>(max_set_number)) {
      angles[channel].v = static_cast<int16_t>(angles[channel].v + robine1x_inset_line_offset);
    }
  }

  return angles;
}

std::array<AngleHV, compact_channel_count> interpolate_robin_compact_angles(
  const SeyondCoBlockHeader & header, const std::vector<uint8_t> & angle_hv_table,
  int max_set_number)
{
  switch (max_set_number) {
    case robinw_max_set_number:
      return interpolate_robin_compact_angles_impl<robinw_max_set_number>(
        header, angle_hv_table, max_set_number);
    case robine2x_max_set_number:
      return interpolate_robin_compact_angles_impl<robine2x_max_set_number>(
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

  int v_index = (v_angle >> falcon_nps_table_shift) + falcon_nps_effective_half_size_v;
  int h_index = (h_angle >> falcon_nps_table_shift) + falcon_nps_effective_half_size_h;
  v_index &= (falcon_nps_table_size_v - 1);
  h_index &= (falcon_nps_table_size_h - 1);

  const auto quantize = [](double value) {
    return static_cast<int>(std::floor(value / falcon_nps_adjustment_unit_meters + 0.5));
  };

  adjustment.x = quantize(falcon_ps_to_nps_adjustment[0][channel][v_index][h_index]) *
                 falcon_nps_adjustment_unit_meters;
  adjustment.z = quantize(falcon_ps_to_nps_adjustment[1][channel][v_index][h_index]) *
                 falcon_nps_adjustment_unit_meters;
  return adjustment;
}

RobinAdjustment interpolate_robin_w_adjustment(int h_angle, uint32_t scan_id)
{
  RobinAdjustment adjustment{};
  if (scan_id >= 192) {
    return adjustment;
  }

  int adjusted_h_angle = h_angle + (robin_nps_effective_half_size << robin_nps_table_shift);
  int h_index = adjusted_h_angle >> robin_nps_table_shift;
  h_index &= (robin_nps_table_size - 1);
  if (h_index > robin_nps_table_size - 2) {
    h_index = robin_nps_table_size - 2;
  }

  const int h_offset = adjusted_h_angle & robin_nps_table_mask;
  const int h_offset2 = robin_nps_table_step - h_offset;

  const auto interpolate_axis = [&](size_t axis) {
    const int u =
      static_cast<int>(std::floor(robin_w_ps_to_nps_adjustment[axis][scan_id][h_index] + 0.5));
    const int v =
      static_cast<int>(std::floor(robin_w_ps_to_nps_adjustment[axis][scan_id][h_index + 1] + 0.5));
    const int blended =
      (u * h_offset2 + v * h_offset + robin_nps_table_half_step) >> robin_nps_table_shift;
    return blended * robin_nps_adjustment_unit_meters;
  };

  adjustment.x = interpolate_axis(0);
  adjustment.y = interpolate_axis(1);
  adjustment.z = interpolate_axis(2);
  return adjustment;
}

RobinAdjustment interpolate_robin_e2x_adjustment(int h_angle)
{
  RobinAdjustment adjustment{};

  int adjusted_h_angle = h_angle + (robin_nps_effective_half_size << robin_nps_table_shift);
  int h_index = adjusted_h_angle >> robin_nps_table_shift;
  h_index &= (robin_nps_table_size - 1);
  if (h_index > robin_nps_table_size - 2) {
    h_index = robin_nps_table_size - 2;
  }

  const int h_offset = adjusted_h_angle & robin_nps_table_mask;
  const int h_offset2 = robin_nps_table_step - h_offset;

  const auto interpolate_axis = [&](size_t axis) {
    const int u = static_cast<int>(std::floor(robin_e2x_ps_to_nps_adjustment[axis][h_index] + 0.5));
    const int v =
      static_cast<int>(std::floor(robin_e2x_ps_to_nps_adjustment[axis][h_index + 1] + 0.5));
    const int blended =
      (u * h_offset2 + v * h_offset + robin_nps_table_half_step) >> robin_nps_table_shift;
    return blended * robin_nps_adjustment_unit_meters;
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
  current_scan_frame_idx_ = 0;
  current_scan_start_timestamp_ns_ = 0;
  has_current_scan_frame_ = false;
}

SeyondPacketDecodeResult SeyondDecoder::unpack(const std::vector<uint8_t> & packet_data)
{
  if (packet_data.size() < sizeof(SeyondDataPacket)) {
    return {0, 0, false};
  }

  const auto * common = reinterpret_cast<const SeyondPacketCommon *>(packet_data.data());
  if (common->magic_number != seyond_data_packet_magic_number) {
    return {0, 0, false};
  }

  bool supported_layout = false;
  uint64_t packet_index = 0;
  bool is_first_sub_frame = false;
  bool is_last_sub_frame = false;
  uint64_t packet_start_timestamp_ns = 0;
  size_t initial_points = 0;
  bool scan_complete = false;

  switch (config_.sensor_model) {
    case SeyondSensorModel::FALCON_K: {
      const auto * packet = reinterpret_cast<const SeyondFalconDataPacket *>(packet_data.data());
      if (
        packet->common.size < sizeof(SeyondFalconDataPacket) ||
        packet->common.size > packet_data.size()) {
        return {0, 0, false};
      }
      const auto payload_size =
        static_cast<size_t>(packet->common.size) - sizeof(SeyondFalconDataPacket);
      const auto required_payload_size =
        static_cast<uint64_t>(packet->item_number) * static_cast<uint64_t>(packet->item_size);
      if (required_payload_size > payload_size) {
        return {0, 0, false};
      }
      supported_layout =
        packet->type == item_type_sphere_pointcloud &&
        (packet->item_size == sizeof(SeyondBlock) || packet->item_size == sizeof(SeyondBlockDual));
      if (!supported_layout) {
        return {packet_timestamp_us_to_ns(packet->common.ts_start_us), 0, false};
      }
      packet_index = packet->idx;
      is_first_sub_frame = packet->is_first_sub_frame;
      is_last_sub_frame = packet->is_last_sub_frame;
      packet_start_timestamp_ns = packet_timestamp_us_to_ns(packet->common.ts_start_us);

      if (has_current_scan_frame_ && packet_index != current_scan_frame_idx_) {
        if (!current_scan_cloud_->empty()) {
          pointcloud_callback_(current_scan_cloud_, current_scan_start_timestamp_ns_);
          current_scan_cloud_ = std::make_shared<NebulaPointCloud>();
          scan_complete = true;
        }
        current_scan_start_timestamp_ns_ = 0;
        has_current_scan_frame_ = false;
      }

      if (
        !has_current_scan_frame_ || current_scan_start_timestamp_ns_ == 0 || is_first_sub_frame ||
        current_scan_cloud_->empty() ||
        packet_start_timestamp_ns < current_scan_start_timestamp_ns_) {
        current_scan_frame_idx_ = packet_index;
        current_scan_start_timestamp_ns_ = packet_start_timestamp_ns;
        has_current_scan_frame_ = true;
      }

      initial_points = current_scan_cloud_->size();
      parse_falcon_k(packet);
      break;
    }
    case SeyondSensorModel::ROBIN_W:
    case SeyondSensorModel::ROBIN_E1X:
    case SeyondSensorModel::HUMMINGBIRD_D1: {
      const auto * packet = reinterpret_cast<const SeyondDataPacket *>(packet_data.data());
      if (
        packet->common.size < sizeof(SeyondDataPacket) ||
        packet->common.size > packet_data.size()) {
        return {0, 0, false};
      }

      const auto payload_size = static_cast<size_t>(packet->common.size) - sizeof(SeyondDataPacket);
      const auto required_payload_size =
        static_cast<uint64_t>(packet->item_number) * static_cast<uint64_t>(packet->item_size);
      if (required_payload_size > payload_size) {
        return {0, 0, false};
      }

      if (config_.sensor_model == SeyondSensorModel::ROBIN_W) {
        supported_layout = (packet->type == item_type_robinw_sphere_pointcloud &&
                            packet->item_size == sizeof(SeyondEnBlock)) ||
                           (packet->type == item_type_robinw_compact_pointcloud &&
                            is_supported_compact_item_size(packet->item_size));
      } else if (config_.sensor_model == SeyondSensorModel::ROBIN_E1X) {
        supported_layout = (packet->type == item_type_robine_sphere_pointcloud &&
                            packet->item_size == sizeof(SeyondEnBlock)) ||
                           (packet->type == item_type_robine2x_compact_pointcloud &&
                            is_supported_compact_item_size(packet->item_size));
      } else {
        supported_layout = packet->type == item_type_hummingbird_compact_pointcloud &&
                           is_supported_compact_item_size(packet->item_size);
      }
      if (!supported_layout) {
        return {packet_timestamp_us_to_ns(packet->common.ts_start_us), 0, false};
      }

      packet_index = packet->idx;
      is_first_sub_frame = packet->is_first_sub_frame;
      is_last_sub_frame = packet->is_last_sub_frame;
      packet_start_timestamp_ns = packet_timestamp_us_to_ns(packet->common.ts_start_us);

      if (has_current_scan_frame_ && packet_index != current_scan_frame_idx_) {
        if (!current_scan_cloud_->empty()) {
          pointcloud_callback_(current_scan_cloud_, current_scan_start_timestamp_ns_);
          current_scan_cloud_ = std::make_shared<NebulaPointCloud>();
          scan_complete = true;
        }
        current_scan_start_timestamp_ns_ = 0;
        has_current_scan_frame_ = false;
      }

      if (
        !has_current_scan_frame_ || current_scan_start_timestamp_ns_ == 0 || is_first_sub_frame ||
        current_scan_cloud_->empty() ||
        packet_start_timestamp_ns < current_scan_start_timestamp_ns_) {
        current_scan_frame_idx_ = packet_index;
        current_scan_start_timestamp_ns_ = packet_start_timestamp_ns;
        has_current_scan_frame_ = true;
      }

      initial_points = current_scan_cloud_->size();
      if (config_.sensor_model == SeyondSensorModel::ROBIN_W) {
        if (packet->type == item_type_robinw_compact_pointcloud) {
          parse_robin_compact(packet);
        } else {
          parse_robin_w_e1x(packet);
        }
      } else if (config_.sensor_model == SeyondSensorModel::ROBIN_E1X) {
        if (packet->type == item_type_robine2x_compact_pointcloud) {
          parse_robin_compact(packet);
        } else {
          parse_robin_w_e1x(packet);
        }
      } else {
        parse_hummingbird_d1(packet);
      }
      break;
    }
    default:
      return {0, 0, false};
  }

  size_t points_unpacked = current_scan_cloud_->size() - initial_points;

  if (is_last_sub_frame && !current_scan_cloud_->empty()) {
    pointcloud_callback_(current_scan_cloud_, current_scan_start_timestamp_ns_);
    current_scan_cloud_ = std::make_shared<NebulaPointCloud>();
    current_scan_frame_idx_ = 0;
    current_scan_start_timestamp_ns_ = 0;
    has_current_scan_frame_ = false;
    scan_complete = true;
  }

  return {packet_start_timestamp_ns, points_unpacked, scan_complete};
}

void SeyondDecoder::parse_falcon_k(const SeyondFalconDataPacket * packet)
{
  const auto * payload = reinterpret_cast<const uint8_t *>(packet) + sizeof(SeyondFalconDataPacket);
  const uint64_t packet_start_timestamp_ns = packet_timestamp_us_to_ns(packet->common.ts_start_us);
  const uint32_t return_count = packet->item_size == sizeof(SeyondBlockDual) ? 2U : 1U;
  const uint32_t channel_count = 4U;

  const double radians_per_packet_angle_unit = M_PI / 32768.0;
  const double meter_per_unit = packet->long_distance_mode ? (1.0 / 100.0) : (1.0 / 200.0);

  int v_angle_diff_base = (calibration_.v_angle_offset != 0.0)
                            ? static_cast<int>(calibration_.v_angle_offset)
                            : falconk_v_angle_diff_base;

  for (uint32_t i = 0; i < packet->item_number; ++i) {
    const auto * block_ptr =
      reinterpret_cast<const SeyondBlockHeader *>(payload + i * packet->item_size);
    const auto & block = *block_ptr;
    const auto * points = reinterpret_cast<const SeyondChannelPoint *>(
      payload + i * packet->item_size + sizeof(SeyondBlockHeader));

    for (uint32_t return_idx = 0; return_idx < return_count; ++return_idx) {
      for (uint32_t channel = 0; channel < channel_count; ++channel) {
        const auto & point = points[channel + return_idx * channel_count];

        if (point.radius == 0) continue;

        int32_t h_angle_raw = block.h_angle;
        int32_t v_angle_raw = block.v_angle;

        uint8_t intensity = static_cast<uint8_t>((static_cast<uint32_t>(point.refl) * 255) / 255);

        if (channel == 1) {
          h_angle_raw += block.h_angle_diff_1;
          v_angle_raw += block.v_angle_diff_1 + 1 * v_angle_diff_base;
        } else if (channel == 2) {
          h_angle_raw += block.h_angle_diff_2;
          v_angle_raw += block.v_angle_diff_2 + 2 * v_angle_diff_base;
        } else if (channel == 3) {
          h_angle_raw += block.h_angle_diff_3;
          v_angle_raw += block.v_angle_diff_3 + 3 * v_angle_diff_base;
        }

        double h_angle = h_angle_raw * radians_per_packet_angle_unit;
        double v_angle = v_angle_raw * radians_per_packet_angle_unit;
        double radius = point.radius * meter_per_unit;

        double cos_v_angle = std::cos(v_angle);
        float x = static_cast<float>(radius * cos_v_angle * std::cos(h_angle));
        float y = static_cast<float>(-radius * cos_v_angle * std::sin(h_angle));
        float z = static_cast<float>(radius * std::sin(v_angle));
        const auto adjustment = lookup_falcon_adjustment(h_angle_raw, v_angle_raw, channel);
        x = static_cast<float>(x + adjustment.z);
        z = static_cast<float>(z + adjustment.x);

        add_point(
          x, y, z, intensity, static_cast<uint16_t>(block.scan_id),
          to_scan_relative_timestamp_ns(
            current_scan_start_timestamp_ns_, packet_start_timestamp_ns,
            static_cast<uint32_t>(block.ts_10us) * 10000U));
      }
    }
  }
}

void SeyondDecoder::parse_robin_w_e1x(const SeyondDataPacket * packet)
{
  const auto * payload = reinterpret_cast<const uint8_t *>(packet) + sizeof(SeyondDataPacket);
  const uint64_t packet_start_timestamp_ns = packet_timestamp_us_to_ns(packet->common.ts_start_us);

  const double radians_per_packet_angle_unit = M_PI / 32768.0;
  const double meter_per_unit = 1.0 / 400.0;

  const bool is_robin_w = (config_.sensor_model == SeyondSensorModel::ROBIN_W);
  const bool is_robin_e1x = (config_.sensor_model == SeyondSensorModel::ROBIN_E1X);

  int v_base = 0;
  int max_set = robinw_max_set_number;
  size_t table_min_size = robinw_table_min_size;

  if (is_robin_w) {
    v_base = (calibration_.v_angle_offset != 0.0) ? static_cast<int>(calibration_.v_angle_offset)
                                                  : robinw_v_angle_diff_base;
    max_set = robinw_max_set_number;
    table_min_size = robinw_table_min_size;
  } else if (is_robin_e1x) {
    max_set = robine_lite_max_set_number;
    table_min_size = robine_lite_table_min_size;
  }

  bool use_calibration = (calibration_.angle_hv_table.size() >= table_min_size);

  for (uint32_t i = 0; i < packet->item_number; ++i) {
    const auto * block_ptr =
      reinterpret_cast<const SeyondEnBlock *>(payload + i * packet->item_size);
    const auto & header = block_ptr->header;

    for (uint32_t channel = 0; channel < 4; ++channel) {
      const auto & point = block_ptr->points[channel];
      if (point.radius == 0) continue;

      uint16_t raw_val = (config_.reflectance_mode == SeyondReflectanceMode::REFLECTIVITY)
                           ? point.reflectance
                           : point.intensity;
      uint8_t intensity = static_cast<uint8_t>((static_cast<uint32_t>(raw_val) * 255) / 4095);

      int32_t h_angle_raw, v_angle_raw;

      if (use_calibration) {
        int h_offset_total = header.h_angle - polygon_min_angle;
        if (h_offset_total < 0) h_offset_total = 0;
        int h_idx = std::min(polygon_table_size - 2, h_offset_total >> encoder_table_shift);
        int h_offset = h_offset_total & encoder_table_mask;
        int h_offset2 = encoder_table_step - h_offset;
        int set_num = header.scan_id % max_set;
        if (is_robin_w) {
          using RobinWTableType = AngleHV[polygon_max_facets][polygon_table_size]
                                         [robinw_max_set_number][max_receiver_in_set];
          const auto & table =
            *reinterpret_cast<const RobinWTableType *>(calibration_.angle_hv_table.data() + 10);
          const AngleHV & b1 = table[header.facet][h_idx][set_num][channel];
          const AngleHV & b2 = table[header.facet][h_idx + 1][set_num][channel];
          h_angle_raw = (b1.h * h_offset2 + b2.h * h_offset) >> encoder_table_shift;
          v_angle_raw = (b1.v * h_offset2 + b2.v * h_offset) >> encoder_table_shift;
        } else {
          using RobinETableType = AngleHV[polygon_max_facets][polygon_table_size]
                                         [robine_lite_max_set_number][max_receiver_in_set];
          const auto & table =
            *reinterpret_cast<const RobinETableType *>(calibration_.angle_hv_table.data() + 10);
          const AngleHV & b1 = table[header.facet][h_idx][set_num][channel];
          const AngleHV & b2 = table[header.facet][h_idx + 1][set_num][channel];
          h_angle_raw = (b1.h * h_offset2 + b2.h * h_offset) >> encoder_table_shift;
          v_angle_raw = (b1.v * h_offset2 + b2.v * h_offset) >> encoder_table_shift;
        }

        if (is_robin_e1x && header.scan_id >= static_cast<uint32_t>(max_set)) {
          v_angle_raw += robine1x_inset_line_offset;
        }
      } else {
        h_angle_raw = header.h_angle;
        v_angle_raw = header.v_angle;

        if (channel == 1) {
          h_angle_raw += header.h_angle_diff_1;
          v_angle_raw += header.v_angle_diff_1 + 1 * v_base;
        } else if (channel == 2) {
          h_angle_raw += header.h_angle_diff_2;
          v_angle_raw += header.v_angle_diff_2 + 2 * v_base;
        } else if (channel == 3) {
          h_angle_raw += header.h_angle_diff_3;
          v_angle_raw += header.v_angle_diff_3 + 3 * v_base;
        }
      }

      double h_angle = h_angle_raw * radians_per_packet_angle_unit;
      double v_angle = v_angle_raw * radians_per_packet_angle_unit;
      double radius = point.radius * meter_per_unit;

      double cos_v_angle = std::cos(v_angle);
      float x = static_cast<float>(radius * cos_v_angle * std::cos(h_angle));
      float y = static_cast<float>(-radius * cos_v_angle * std::sin(h_angle));
      float z = static_cast<float>(radius * std::sin(v_angle));

      uint16_t physical_channel = static_cast<uint16_t>(channel);
      if (is_robin_w) {
        size_t map_idx = static_cast<size_t>((header.scan_id % 12) * 4 + channel);
        if (map_idx < sizeof(robinw_channel_mapping)) {
          physical_channel = static_cast<uint16_t>(robinw_channel_mapping[map_idx]) +
                             static_cast<uint16_t>(header.facet * 48);
        }
      } else if (is_robin_e1x) {
        size_t map_idx = static_cast<size_t>((header.scan_id % 24) * 4 + channel);
        if (map_idx < sizeof(robinelite_channel_mapping)) {
          physical_channel = robinelite_channel_mapping[map_idx];
        }
      }
      if (is_robin_w) {
        const auto adjustment = interpolate_robin_w_adjustment(h_angle_raw, physical_channel);
        x = static_cast<float>(x + adjustment.z);
        y = static_cast<float>(y - adjustment.y);
        z = static_cast<float>(z + adjustment.x);
        y = -y;
        z = -z;
      }

      add_point(
        x, y, z, intensity, physical_channel,
        to_scan_relative_timestamp_ns(
          current_scan_start_timestamp_ns_, packet_start_timestamp_ns,
          static_cast<uint32_t>(header.ts_10us) * 10000U));
    }
  }
}

void SeyondDecoder::parse_robin_compact(const SeyondDataPacket * packet)
{
  const auto * payload = reinterpret_cast<const uint8_t *>(packet) + sizeof(SeyondDataPacket);
  const uint64_t packet_start_timestamp_ns = packet_timestamp_us_to_ns(packet->common.ts_start_us);
  const auto return_count = compact_return_count(packet->item_size);
  if (return_count == 0) {
    return;
  }

  const bool is_robin_w_compact = packet->type == item_type_robinw_compact_pointcloud;
  const bool is_robin_e2x_compact = packet->type == item_type_robine2x_compact_pointcloud;
  const int max_set_number = is_robin_w_compact ? robinw_max_set_number : robine2x_max_set_number;
  const size_t table_min_size =
    is_robin_w_compact ? robinw_table_min_size : robine2x_table_min_size;
  const bool use_calibration = calibration_.angle_hv_table.size() >= table_min_size;
  const double radians_per_packet_angle_unit = M_PI / 32768.0;
  const double meter_per_unit = 1.0 / 400.0;

  for (uint32_t i = 0; i < packet->item_number; ++i) {
    const auto * block_ptr =
      reinterpret_cast<const SeyondCoBlock *>(payload + i * packet->item_size);
    const auto & header = block_ptr->header;
    const auto * points = reinterpret_cast<const SeyondCoChannelPoint *>(
      payload + i * packet->item_size + sizeof(SeyondCoBlockHeader));

    std::array<AngleHV, compact_channel_count> angles{};
    if (use_calibration) {
      angles =
        interpolate_robin_compact_angles(header, calibration_.angle_hv_table, max_set_number);
    }

    for (size_t channel = 0; channel < compact_channel_count; ++channel) {
      for (size_t return_idx = 0; return_idx < return_count; ++return_idx) {
        const auto & point = points[channel + return_idx * compact_channel_count];
        if (point.radius == 0) {
          continue;
        }

        double h_angle = 0.0;
        double v_angle = 0.0;
        if (use_calibration) {
          h_angle = static_cast<double>(angles[channel].h) * radians_per_packet_angle_unit;
          v_angle = static_cast<double>(angles[channel].v) * radians_per_packet_angle_unit;
          if (!is_robin_inside_compact_fov(angles[channel])) {
            continue;
          }
        } else {
          h_angle = static_cast<double>(header.p_angle) * radians_per_packet_angle_unit;
          v_angle = static_cast<double>(header.g_angle) * radians_per_packet_angle_unit;
        }

        double radius = point.radius * meter_per_unit;
        double cos_v_angle = std::cos(v_angle);
        float x = static_cast<float>(radius * cos_v_angle * std::cos(h_angle));
        float y = static_cast<float>(-radius * cos_v_angle * std::sin(h_angle));
        float z = static_cast<float>(radius * std::sin(v_angle));
        uint16_t physical_channel =
          static_cast<uint16_t>(header.scan_id * compact_channel_count + channel);
        if (is_robin_w_compact) {
          const size_t map_idx =
            static_cast<size_t>(header.scan_id * compact_channel_count + channel);
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
        uint8_t intensity = static_cast<uint8_t>((static_cast<uint32_t>(point.refl) * 255) / 4095);

        add_point(
          x, y, z, intensity, physical_channel,
          to_scan_relative_timestamp_ns(
            current_scan_start_timestamp_ns_, packet_start_timestamp_ns,
            static_cast<uint32_t>(header.ts_10us) * 10000U));
      }
    }
  }
}

void SeyondDecoder::parse_hummingbird_d1(const SeyondDataPacket * packet)
{
  const auto * payload = reinterpret_cast<const uint8_t *>(packet) + sizeof(SeyondDataPacket);
  const uint64_t packet_start_timestamp_ns = packet_timestamp_us_to_ns(packet->common.ts_start_us);
  bool use_calibration = (calibration_.angle_hv_table.size() >= hummingbird_table_min_size);
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

    for (size_t channel = 0; channel < compact_channel_count; ++channel) {
      for (size_t return_idx = 0; return_idx < return_count; ++return_idx) {
        const auto & point = points[channel + return_idx * compact_channel_count];
        if (point.radius == 0) continue;

        double h_angle, v_angle;
        if (use_calibration) {
          using TableType = AngleHV[hummingbird_table_height][hummingbird_table_width];
          const TableType & table =
            *reinterpret_cast<const TableType *>(calibration_.angle_hv_table.data() + 10);
          const AngleHV & angle = table[header.scan_id][header.scan_idx + channel];
          if (!is_hummingbird_inside_compact_fov(angle)) continue;
          h_angle = angle.h * (M_PI / 32768.0);
          v_angle = angle.v * (M_PI / 32768.0);
        } else {
          h_angle = header.p_angle * (M_PI / 32768.0);
          v_angle = header.g_angle * (M_PI / 32768.0);
        }

        double radius = point.radius * (1.0 / 400.0);
        double cos_v_angle = std::cos(v_angle);
        float x = static_cast<float>(radius * cos_v_angle * std::cos(h_angle));
        float y = static_cast<float>(-radius * cos_v_angle * std::sin(h_angle));
        float z = static_cast<float>(radius * std::sin(v_angle));

        uint8_t intensity = static_cast<uint8_t>((static_cast<uint32_t>(point.refl) * 255) / 4095);

        add_point(
          x, y, z, intensity, static_cast<uint16_t>(base_channel + channel),
          to_scan_relative_timestamp_ns(
            current_scan_start_timestamp_ns_, packet_start_timestamp_ns,
            static_cast<uint32_t>(header.ts_10us) * 10000U));
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
