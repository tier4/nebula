// Copyright 2024 TIER IV, Inc.

#include "nebula_decoders/nebula_decoders_velodyne/decoders/vlp32_decoder.hpp"

#include <angles/angles.h>

#include <cmath>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace nebula
{
namespace drivers::vlp32
{
Vlp32Decoder::Vlp32Decoder(
  const std::shared_ptr<const drivers::VelodyneSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> &
    calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  calibration_configuration_ = calibration_configuration;

  scan_timestamp_ = -1;

  scan_pc_.reset(new NebulaPointCloud);
  overflow_pc_.reset(new NebulaPointCloud);

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < g_rotation_max_units; ++rot_index) {
    float rotation = angles::from_degrees(g_rotation_resolution * rot_index);
    rotation_radians_[rot_index] = rotation;
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }
  phase_ = (uint16_t)round(sensor_configuration_->scan_phase * 100);

  timing_offsets_.resize(12);
  for (size_t i = 0; i < timing_offsets_.size(); ++i) {
    timing_offsets_[i].resize(32);
  }
  // constants
  double full_firing_cycle = 55.296 * 1e-6;  // seconds
  double single_firing = 2.304 * 1e-6;       // seconds
  double dataBlockIndex, dataPointIndex;
  bool dual_mode = sensor_configuration_->return_mode == ReturnMode::DUAL;
  // compute timing offsets
  for (size_t x = 0; x < timing_offsets_.size(); ++x) {
    for (size_t y = 0; y < timing_offsets_[x].size(); ++y) {
      if (dual_mode) {
        dataBlockIndex = x / 2;
      } else {
        dataBlockIndex = x;
      }
      dataPointIndex = y / 2;
      timing_offsets_[x][y] =
        (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
    }
  }
}

std::tuple<drivers::NebulaPointCloudPtr, double> Vlp32Decoder::get_pointcloud()
{
  double phase = angles::from_degrees(sensor_configuration_->scan_phase);
  if (!scan_pc_->points.empty()) {
    auto current_azimuth = scan_pc_->points.back().azimuth;
    auto phase_diff = (2 * M_PI + current_azimuth - phase);
    while (phase_diff < M_PI_2 && !scan_pc_->points.empty()) {
      overflow_pc_->points.push_back(scan_pc_->points.back());
      scan_pc_->points.pop_back();
      current_azimuth = scan_pc_->points.back().azimuth;
      phase_diff = (2 * M_PI + current_azimuth - phase);
    }
    overflow_pc_->width = overflow_pc_->points.size();
    scan_pc_->width = scan_pc_->points.size();
    scan_pc_->height = 1;
  }
  return std::make_tuple(scan_pc_, scan_timestamp_);
}

int Vlp32Decoder::points_per_packet()
{
  return g_blocks_per_packet * g_scans_per_block;
}

void Vlp32Decoder::reset_pointcloud(double time_stamp)
{
  scan_pc_->clear();
  reset_overflow(time_stamp);  // transfer existing overflow points to the cleared pointcloud
}

void Vlp32Decoder::reset_overflow(double time_stamp)
{
  if (overflow_pc_->points.size() == 0) {
    scan_timestamp_ = -1;
    overflow_pc_->points.reserve(max_pts_);
    return;
  }

  // Compute the absolute time stamp of the last point of the overflow pointcloud
  const double last_overflow_time_stamp =
    scan_timestamp_ + 1e-9 * overflow_pc_->points.back().time_stamp;

  // Detect cases where there is an unacceptable time difference between the last overflow point and
  // the first point of the next packet. In that case, there was probably a packet drop so it is
  // better to ignore the overflow pointcloud
  if (time_stamp - last_overflow_time_stamp > 0.05) {
    scan_timestamp_ = -1;
    overflow_pc_->points.clear();
    overflow_pc_->points.reserve(max_pts_);
    return;
  }

  // Add the overflow buffer points
  while (overflow_pc_->points.size() > 0) {
    auto overflow_point = overflow_pc_->points.back();

    // The overflow points had the stamps from the previous pointcloud. These need to be changed to
    // be relative to the overflow's packet timestamp
    double new_timestamp_seconds =
      scan_timestamp_ + 1e-9 * overflow_point.time_stamp - last_block_timestamp_;
    overflow_point.time_stamp =
      static_cast<uint32_t>(new_timestamp_seconds < 0.0 ? 0.0 : 1e9 * new_timestamp_seconds);

    scan_pc_->points.emplace_back(overflow_point);
    overflow_pc_->points.pop_back();
  }

  // When there is overflow, the timestamp becomes the overflow packets' one
  scan_timestamp_ = last_block_timestamp_;
  overflow_pc_->points.clear();
  overflow_pc_->points.reserve(max_pts_);
}

void Vlp32Decoder::unpack(const std::vector<uint8_t> & packet, double packet_seconds)
{
  check_and_handle_scan_complete(packet, packet_seconds, phase_);

  const raw_packet_t * raw = (const raw_packet_t *)packet.data();
  float last_azimuth_diff = 0;
  uint16_t azimuth_next;
  uint8_t return_mode = packet[g_return_mode_index];
  const bool dual_return = (return_mode == g_return_mode_dual);

  for (uint i = 0; i < g_blocks_per_packet; i++) {
    int bank_origin = 0;
    if (raw->blocks[i].header == g_lower_bank) {
      // lower bank lasers are [32..63]
      bank_origin = 32;
    }
    float azimuth_diff;
    uint16_t azimuth;

    // Calculate difference between current and next block's azimuth angle.
    if (i == 0) {
      azimuth = raw->blocks[i].rotation;
    } else {
      azimuth = azimuth_next;
    }
    if (i < static_cast<uint>(g_blocks_per_packet - (1 + dual_return))) {
      // Get the next block rotation to calculate how far we rotate between blocks
      azimuth_next = raw->blocks[i + (1 + dual_return)].rotation;

      // Finds the difference between two successive blocks
      azimuth_diff = static_cast<float>((36000 + azimuth_next - azimuth) % 36000);

      // This is used when the last block is next to predict rotation amount
      last_azimuth_diff = azimuth_diff;
    } else {
      // This makes the assumption the difference between the last block and the next packet is the
      // same as the last to the second to last.
      // Assumes RPM doesn't change much between blocks.
      azimuth_diff = (i == static_cast<uint>(g_blocks_per_packet - (4 * dual_return) - 1))
                       ? 0
                       : last_azimuth_diff;
    }

    for (uint j = 0, k = 0; j < g_scans_per_block; j++, k += g_raw_scan_size) {
      float x, y, z;
      uint8_t intensity;
      const uint8_t laser_number = j + bank_origin;

      const VelodyneLaserCorrection & corrections =
        calibration_configuration_->velodyne_calibration.laser_corrections[laser_number];

      /** Position Calculation */
      const raw_block_t & block = raw->blocks[i];
      union two_bytes current_return;
      current_return.bytes[0] = block.data[k];
      current_return.bytes[1] = block.data[k + 1];

      union two_bytes other_return;
      if (dual_return) {
        other_return.bytes[0] = i % 2 ? raw->blocks[i - 1].data[k] : raw->blocks[i + 1].data[k];
        other_return.bytes[1] =
          i % 2 ? raw->blocks[i - 1].data[k + 1] : raw->blocks[i + 1].data[k + 1];
      }
      // Apply timestamp if this is the first new packet in the scan.
      auto block_timestamp = packet_seconds;
      if (scan_timestamp_ < 0) {
        scan_timestamp_ = block_timestamp;
      }
      // Do not process if there is no return, or in dual return mode and the first and last echos
      // are the same.
      if (
        (current_return.bytes[0] == 0 && current_return.bytes[1] == 0) ||
        (dual_return && i % 2 && other_return.bytes[0] == current_return.bytes[0] &&
         other_return.bytes[1] == current_return.bytes[1])) {
        continue;
      }

      float distance = current_return.uint *
                       calibration_configuration_->velodyne_calibration.distance_resolution_m;
      if (distance > 1e-6) {
        distance += corrections.dist_correction;
      }

      if (
        distance > sensor_configuration_->min_range &&
        distance < sensor_configuration_->max_range) {
        /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
        if (
          (block.rotation >= sensor_configuration_->cloud_min_angle * 100 &&
           block.rotation <= sensor_configuration_->cloud_max_angle * 100 &&
           sensor_configuration_->cloud_min_angle < sensor_configuration_->cloud_max_angle) ||
          (sensor_configuration_->cloud_min_angle > sensor_configuration_->cloud_max_angle &&
           (raw->blocks[i].rotation <= sensor_configuration_->cloud_max_angle * 100 ||
            raw->blocks[i].rotation >= sensor_configuration_->cloud_min_angle * 100))) {
          const float cos_vert_angle = corrections.cos_vert_correction;
          const float sin_vert_angle = corrections.sin_vert_correction;
          float azimuth_corrected_f =
            azimuth + (azimuth_diff * g_vlp32_channel_duration / g_vlp32_seq_duration * j) -
            corrections.rot_correction * 180.0 / M_PI * 100;
          if (azimuth_corrected_f < 0) {
            azimuth_corrected_f += 36000;
          }
          const uint16_t azimuth_corrected =
            (static_cast<uint16_t>(std::round(azimuth_corrected_f))) % 36000;

          const float cos_rot_angle = cos_rot_table_[azimuth_corrected];
          const float sin_rot_angle = sin_rot_table_[azimuth_corrected];

          const float horiz_offset = corrections.horiz_offset_correction;
          const float vert_offset = corrections.vert_offset_correction;

          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathematical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) {
            xx = -xx;
          }
          if (yy < 0) {
            yy = -yy;
          }

          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = (corrections.dist_correction - corrections.dist_correction_x) *
                                (xx - 2.4) / (25.04 - 2.4) +
                              corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = (corrections.dist_correction - corrections.dist_correction_y) *
                                (yy - 1.93) / (25.04 - 1.93) +
                              corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }

          const float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathematical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
          /// the expression with '-' is proved to be better than the one with '+'
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

          const float distance_y = distance + distance_corr_y;
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathematical
           * model we used.
           */
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathematical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

          /** Use standard ROS coordinate system (right-hand rule) */
          const float x_coord = y;
          const float y_coord = -x;
          const float z_coord = z;

          /** Intensity Calculation */
          const float min_intensity = corrections.min_intensity;
          const float max_intensity = corrections.max_intensity;

          intensity = raw->blocks[i].data[k + 2];

          last_block_timestamp_ = block_timestamp;

          const float focal_offset = 256 * (1 - corrections.focal_distance / 13100) *
                                     (1 - corrections.focal_distance / 13100);
          const float focal_slope = corrections.focal_slope;
          float sqr = (1 - static_cast<float>(current_return.uint) / 65535) *
                      (1 - static_cast<float>(current_return.uint) / 65535);
          intensity += focal_slope * (std::abs(focal_offset - 256 * sqr));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          double point_time_offset = timing_offsets_[i][j];

          nebula::drivers::ReturnType return_type;
          switch (return_mode) {
            case g_return_mode_dual:
              if (
                (other_return.bytes[0] == 0 && other_return.bytes[1] == 0) ||
                (other_return.bytes[0] == current_return.bytes[0] &&
                 other_return.bytes[1] == current_return.bytes[1])) {
                return_type = drivers::ReturnType::IDENTICAL;
              } else {
                const float other_intensity =
                  i % 2 ? raw->blocks[i - 1].data[k + 2] : raw->blocks[i + 1].data[k + 2];
                bool first = other_return.uint < current_return.uint ? 0 : 1;
                bool strongest = other_intensity < intensity ? 1 : 0;
                if (other_intensity == intensity) {
                  strongest = first ? 0 : 1;
                }
                if (first && strongest) {
                  return_type = drivers::ReturnType::FIRST_STRONGEST;
                } else if (!first && strongest) {
                  return_type = drivers::ReturnType::LAST_STRONGEST;
                } else if (first && !strongest) {
                  return_type = drivers::ReturnType::FIRST_WEAK;
                } else if (!first && !strongest) {
                  return_type = drivers::ReturnType::LAST_WEAK;
                } else {
                  return_type = drivers::ReturnType::UNKNOWN;
                }
              }
              break;
            case g_return_mode_strongest:
              return_type = drivers::ReturnType::STRONGEST;
              break;
            case g_return_mode_last:
              return_type = drivers::ReturnType::LAST;
              break;
            default:
              return_type = drivers::ReturnType::UNKNOWN;
          }
          drivers::NebulaPoint current_point{};
          current_point.x = x_coord;
          current_point.y = y_coord;
          current_point.z = z_coord;
          current_point.return_type = static_cast<uint8_t>(return_type);
          current_point.channel = corrections.laser_ring;
          current_point.azimuth = rotation_radians_[block.rotation];
          current_point.elevation = sin_vert_angle;
          auto point_ts = block_timestamp - scan_timestamp_ + point_time_offset;
          if (point_ts < 0) point_ts = 0;
          current_point.time_stamp = static_cast<uint32_t>(point_ts * 1e9);
          current_point.distance = distance;
          current_point.intensity = intensity;
          scan_pc_->points.emplace_back(current_point);
        }
      }
    }
  }
}

bool Vlp32Decoder::parse_packet(
  [[maybe_unused]] const velodyne_msgs::msg::VelodynePacket & velodyne_packet)
{
  return 0;
}

}  // namespace drivers::vlp32
}  // namespace nebula
