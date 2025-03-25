// Copyright 2024 TIER IV, Inc.

#include "nebula_decoders/nebula_decoders_velodyne/decoders/vlp16_decoder.hpp"

#include <angles/angles.h>

#include <cmath>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace nebula
{
namespace drivers::vlp16
{
Vlp16Decoder::Vlp16Decoder(
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
  // timing table calculation, from velodyne user manual p.64
  timing_offsets_.resize(g_blocks_per_packet);
  for (size_t i = 0; i < timing_offsets_.size(); ++i) {
    timing_offsets_[i].resize(32);
  }
  double full_firing_cycle_s = 55.296 * 1e-6;
  double single_firing_s = 2.304 * 1e-6;
  double data_block_index, data_point_index;
  bool dual_mode = sensor_configuration_->return_mode == ReturnMode::DUAL;
  // compute timing offsets
  for (size_t x = 0; x < timing_offsets_.size(); ++x) {
    for (size_t y = 0; y < timing_offsets_[x].size(); ++y) {
      if (dual_mode) {
        data_block_index = (x - (x % 2)) + (y / 16);
      } else {
        data_block_index = (x * 2) + (y / 16);
      }
      data_point_index = y % 16;
      timing_offsets_[x][y] =
        (full_firing_cycle_s * data_block_index) + (single_firing_s * data_point_index);
    }
  }

  phase_ = (uint16_t)round(sensor_configuration_->scan_phase * 100);
}

std::tuple<drivers::NebulaPointCloudPtr, double> Vlp16Decoder::get_pointcloud()
{
  double phase = angles::from_degrees(sensor_configuration_->scan_phase);
  if (!scan_pc_->points.empty()) {
    auto current_azimuth = scan_pc_->points.back().azimuth;
    auto phase_diff =
      static_cast<size_t>(angles::to_degrees(2 * M_PI + current_azimuth - phase)) % 360;
    while (phase_diff < M_PI_2 && !scan_pc_->points.empty()) {
      overflow_pc_->points.push_back(scan_pc_->points.back());
      scan_pc_->points.pop_back();
      current_azimuth = scan_pc_->points.back().azimuth;
      phase_diff =
        static_cast<size_t>(angles::to_degrees(2 * M_PI + current_azimuth - phase)) % 360;
    }
    overflow_pc_->width = overflow_pc_->points.size();
    scan_pc_->width = scan_pc_->points.size();
    scan_pc_->height = 1;
  }
  return std::make_tuple(scan_pc_, scan_timestamp_);
}

int Vlp16Decoder::points_per_packet()
{
  return g_blocks_per_packet * g_vlp16_firings_per_block * g_vlp16_scans_per_firing;
}

void Vlp16Decoder::reset_pointcloud(double time_stamp)
{
  scan_pc_->clear();
  reset_overflow(time_stamp);  // transfer existing overflow points to the cleared pointcloud
}

void Vlp16Decoder::reset_overflow(double time_stamp)
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

void Vlp16Decoder::unpack(const std::vector<uint8_t> & packet, double packet_seconds)
{
  check_and_handle_scan_complete(packet, packet_seconds, phase_);

  const raw_packet_t * raw = (const raw_packet_t *)packet.data();
  float last_azimuth_diff = 0;
  uint16_t azimuth_next;
  const uint8_t return_mode = packet[g_return_mode_index];
  const bool dual_return = (return_mode == g_return_mode_dual);

  for (uint block = 0; block < g_blocks_per_packet; block++) {
    // Cache block for use.
    const raw_block_t & current_block = raw->blocks[block];
    if (g_upper_bank != raw->blocks[block].header) {
      // Do not flood the log with messages, only issue at most one
      // of these warnings per minute.
      return;  // bad packet: skip the rest
    }

    float azimuth_diff;
    uint16_t azimuth;

    // Calculate difference between current and next block's azimuth angle.
    if (block == 0) {
      azimuth = current_block.rotation;
    } else {
      azimuth = azimuth_next;
    }
    if (block < static_cast<uint>(g_blocks_per_packet - (1 + dual_return))) {
      // Get the next block rotation to calculate how far we rotate between blocks.
      azimuth_next = raw->blocks[block + (1 + dual_return)].rotation;

      // Finds the difference between two successive blocks.
      azimuth_diff = static_cast<float>((36000 + azimuth_next - azimuth) % 36000);

      // This is used when the last block is next to predict rotation amount
      last_azimuth_diff = azimuth_diff;
    } else {
      // This makes the assumption the difference between the last block and the next packet is the
      // same as the last to the second to last.
      // Assumes RPM doesn't change much between blocks.
      azimuth_diff =
        (block == static_cast<uint>(g_blocks_per_packet - dual_return - 1) ? 0 : last_azimuth_diff);
    }

    // Condition added to avoid calculating points which are not in the interesting defined area
    // (min_angle < area < max_angle).
    if (
      (sensor_configuration_->cloud_min_angle < sensor_configuration_->cloud_max_angle &&
       azimuth >= sensor_configuration_->cloud_min_angle * 100 &&
       azimuth <= sensor_configuration_->cloud_max_angle * 100) ||
      (sensor_configuration_->cloud_min_angle > sensor_configuration_->cloud_max_angle)) {
      for (int firing = 0, k = 0; firing < g_vlp16_firings_per_block; ++firing) {
        for (int dsr = 0; dsr < g_vlp16_scans_per_firing; dsr++, k += g_raw_scan_size) {
          union two_bytes current_return;
          union two_bytes other_return;
          // Distance extraction.
          current_return.bytes[0] = current_block.data[k];
          current_return.bytes[1] = current_block.data[k + 1];

          if (dual_return) {
            other_return.bytes[0] =
              block % 2 ? raw->blocks[block - 1].data[k] : raw->blocks[block + 1].data[k];
            other_return.bytes[1] =
              block % 2 ? raw->blocks[block - 1].data[k + 1] : raw->blocks[block + 1].data[k + 1];
          }
          // Apply timestamp if this is the first new packet in the scan.
          auto block_timestamp = packet_seconds;
          if (scan_timestamp_ < 0) {
            scan_timestamp_ = block_timestamp;
          }
          // Do not process if there is no return, or in dual return mode and the first and last
          // echos are the same.
          if (
            (current_return.bytes[0] == 0 && current_return.bytes[1] == 0) ||
            (dual_return && block % 2 && other_return.bytes[0] == current_return.bytes[0] &&
             other_return.bytes[1] == current_return.bytes[1])) {
            continue;
          }
          {
            const VelodyneLaserCorrection & corrections =
              calibration_configuration_->velodyne_calibration.laser_corrections[dsr];
            float distance = current_return.uint *
                             calibration_configuration_->velodyne_calibration.distance_resolution_m;
            if (distance > 1e-6) {
              distance += corrections.dist_correction;
            }

            if (
              distance > sensor_configuration_->min_range &&
              distance < sensor_configuration_->max_range) {
              // Correct for the laser rotation as a function of timing during the firings.
              float azimuth_corrected_f =
                azimuth +
                (azimuth_diff * ((dsr * g_vlp16_dsr_toffset) + (firing * g_vlp16_firing_toffset)) /
                 g_vlp16_block_duration) -
                corrections.rot_correction * 180.0 / M_PI * 100;

              if (azimuth_corrected_f < 0.0) {
                azimuth_corrected_f += 36000.0;
              }
              const uint16_t azimuth_corrected =
                (static_cast<uint16_t>(round(azimuth_corrected_f))) % 36000;

              // Condition added to avoid calculating points which are not in the interesting
              // defined area (min_angle < area < max_angle).
              if (
                (azimuth_corrected >= sensor_configuration_->cloud_min_angle * 100 &&
                 azimuth_corrected <= sensor_configuration_->cloud_max_angle * 100 &&
                 sensor_configuration_->cloud_min_angle < sensor_configuration_->cloud_max_angle) ||
                (sensor_configuration_->cloud_min_angle > sensor_configuration_->cloud_max_angle &&
                 (azimuth_corrected <= sensor_configuration_->cloud_max_angle * 100 ||
                  azimuth_corrected >= sensor_configuration_->cloud_min_angle * 100))) {
                // Convert polar coordinates to Euclidean XYZ.
                const float cos_vert_angle = corrections.cos_vert_correction;
                const float sin_vert_angle = corrections.sin_vert_correction;
                const float cos_rot_angle = cos_rot_table_[azimuth_corrected];
                const float sin_rot_angle = sin_rot_table_[azimuth_corrected];

                // Compute the distance in the xy plane (w/o accounting for rotation).
                const float xy_distance = distance * cos_vert_angle;

                // Use standard ROS coordinate system (right-hand rule).
                const float x_coord = xy_distance * cos_rot_angle;     // velodyne y
                const float y_coord = -(xy_distance * sin_rot_angle);  // velodyne x
                const float z_coord = distance * sin_vert_angle;       // velodyne z
                const uint8_t intensity = current_block.data[k + 2];

                last_block_timestamp_ = block_timestamp;

                double point_time_offset = timing_offsets_[block][firing * 16 + dsr];

                // Determine return type.
                uint8_t return_type;
                switch (return_mode) {
                  case g_return_mode_dual:
                    if (
                      (other_return.bytes[0] == 0 && other_return.bytes[1] == 0) ||
                      (other_return.bytes[0] == current_return.bytes[0] &&
                       other_return.bytes[1] == current_return.bytes[1])) {
                      return_type = static_cast<uint8_t>(drivers::ReturnType::IDENTICAL);
                    } else {
                      const uint8_t other_intensity = block % 2
                                                        ? raw->blocks[block - 1].data[k + 2]
                                                        : raw->blocks[block + 1].data[k + 2];
                      bool first = current_return.uint > other_return.uint;
                      bool strongest = intensity > other_intensity;
                      if (other_intensity == intensity) {
                        strongest = !first;
                      }
                      if (first && strongest) {
                        return_type = static_cast<uint8_t>(drivers::ReturnType::FIRST_STRONGEST);
                      } else if (!first && strongest) {
                        return_type = static_cast<uint8_t>(drivers::ReturnType::LAST_STRONGEST);
                      } else if (first && !strongest) {
                        return_type = static_cast<uint8_t>(drivers::ReturnType::FIRST_WEAK);
                      } else if (!first && !strongest) {
                        return_type = static_cast<uint8_t>(drivers::ReturnType::LAST_WEAK);
                      } else {
                        return_type = static_cast<uint8_t>(drivers::ReturnType::UNKNOWN);
                      }
                    }
                    break;
                  case g_return_mode_strongest:
                    return_type = static_cast<uint8_t>(drivers::ReturnType::STRONGEST);
                    break;
                  case g_return_mode_last:
                    return_type = static_cast<uint8_t>(drivers::ReturnType::LAST);
                    break;
                  default:
                    return_type = static_cast<uint8_t>(drivers::ReturnType::UNKNOWN);
                }
                drivers::NebulaPoint current_point{};
                current_point.x = x_coord;
                current_point.y = y_coord;
                current_point.z = z_coord;
                current_point.return_type = return_type;
                current_point.channel = corrections.laser_ring;
                current_point.azimuth = rotation_radians_[azimuth_corrected];
                current_point.elevation = sin_vert_angle;
                auto point_ts = block_timestamp - scan_timestamp_ + point_time_offset;
                if (point_ts < 0) point_ts = 0;
                current_point.time_stamp = static_cast<uint32_t>(point_ts * 1e9);
                current_point.intensity = intensity;
                current_point.distance = distance;
                scan_pc_->points.emplace_back(current_point);
              }
            }
          }
        }
      }
    }
  }
}

bool Vlp16Decoder::parse_packet(
  [[maybe_unused]] const velodyne_msgs::msg::VelodynePacket & velodyne_packet)
{
  return 0;
}

}  // namespace drivers::vlp16
}  // namespace nebula
