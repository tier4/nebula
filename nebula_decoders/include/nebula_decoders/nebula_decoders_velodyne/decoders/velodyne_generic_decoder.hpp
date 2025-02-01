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

#pragma once

#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_packet.hpp"
#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_scan_decoder.hpp"

#include <nebula_common/nebula_common.hpp>

#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <angles/angles.h>

#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

namespace nebula::drivers
{
/// @brief Velodyne LiDAR decoder
template <typename SensorT>
class VelodyneDecoder : public VelodyneScanDecoder
{
public:
  explicit VelodyneDecoder(
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
    for (uint16_t rot_index = 0; rot_index < velodyne_packet::g_rotation_max_units; ++rot_index) {
      float rotation = angles::from_degrees(velodyne_packet::g_rotation_resolution * rot_index);
      rotation_radians_[rot_index] = rotation;
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }

    phase_ = static_cast<uint16_t>(round(sensor_configuration_->scan_phase * 100));

    // fill vls128 laser azimuth cache
    sensor_.fill_azimuth_cache();  // TODO(ike-kazu): predefine azimuth cache and remove this for
                                   // performance?

    // timing table calculation, from velodyne user manual p.64
    timing_offsets_.resize(velodyne_packet::g_blocks_per_packet);  // x dir size
    for (auto & timing_offset : timing_offsets_) {
      timing_offset.resize(
        velodyne_packet::g_channels_per_block + SensorT::num_maintenance_periods);  // y dir size
    }

    double full_firing_cycle_s = SensorT::full_firing_cycle_s;
    double single_firing_s = SensorT::single_firing_s;
    double offset_packet_time = SensorT::offset_packet_time;
    bool dual_mode = sensor_configuration_->return_mode == ReturnMode::DUAL;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets_.size(); ++x) {
      for (size_t y = 0; y < timing_offsets_[x].size(); ++y) {
        double firing_sequence_index;
        if (dual_mode) {
          // cast to long to make double multiplication integer division
          firing_sequence_index =
            static_cast<uint32_t>(x * SensorT::firing_sequences_per_block / 2) +
            (y / SensorT::channels_per_firing_sequence);
        } else {
          firing_sequence_index = static_cast<uint32_t>(x * SensorT::firing_sequences_per_block) +
                                  (y / SensorT::channels_per_firing_sequence);
        }
        double data_point_index =
          (y % SensorT::channels_per_firing_sequence) / SensorT::num_simultaneous_firings -
          offset_packet_time;
        timing_offsets_[x][y] =
          (full_firing_cycle_s * firing_sequence_index) + (single_firing_s * data_point_index);
      }
    }
  }

  bool has_scanned() override { return has_scanned_; }

  std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() override
  {
    float phase = angles::from_degrees(sensor_configuration_->scan_phase);
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

  void reset_pointcloud(double time_stamp) override
  {
    //  scan_pc_.reset(new NebulaPointCloud);
    scan_pc_->points.clear();
    scan_pc_->points.reserve(max_pts_);
    reset_overflow(time_stamp);  // transfer existing overflow points to the cleared pointcloud
  }

  void reset_overflow(double time_stamp) override
  {
    if (overflow_pc_->points.size() == 0) {
      scan_timestamp_ = -1;
      overflow_pc_->points.reserve(max_pts_);
      return;
    }

    // Compute the absolute time stamp of the last point of the overflow pointcloud
    const double last_overflow_time_stamp =
      scan_timestamp_ + 1e-9 * overflow_pc_->points.back().time_stamp;

    // Detect cases where there is an unacceptable time difference between the last overflow point
    // and the first point of the next packet. In that case, there was probably a packet drop so it
    // is better to ignore the overflow pointcloud
    if (time_stamp - last_overflow_time_stamp > 0.05) {
      scan_timestamp_ = -1;
      overflow_pc_->points.clear();
      overflow_pc_->points.reserve(max_pts_);
      return;
    }

    // Add the overflow buffer points
    while (overflow_pc_->points.size() > 0) {
      auto overflow_point = overflow_pc_->points.back();

      // The overflow points had the stamps from the previous pointcloud. These need to be changed
      // to be relative to the overflow's packet timestamp
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

  void unpack(const std::vector<uint8_t> & velodyne_packet, double packet_seconds) override
  {
    velodyne_packet::raw_packet_t raw_instance{};
    std::memcpy(&raw_instance, velodyne_packet.data(), sizeof(velodyne_packet::raw_packet_t));

    float last_azimuth_diff = 0;
    uint16_t azimuth_next;
    const uint8_t return_mode = velodyne_packet[velodyne_packet::g_return_mode_index];
    const bool dual_return = (return_mode == velodyne_packet::g_return_mode_dual);

    int num_padding_blocks = sensor_.get_num_padding_blocks(dual_return);

    for (uint block = 0;
         block < static_cast<uint>(velodyne_packet::g_blocks_per_packet - num_padding_blocks);
         block++) {
      // Cache block for use.
      const velodyne_packet::raw_block_t & current_block = raw_instance.blocks[block];

      uint bank_origin = 0;
      bank_origin = sensor_.get_bank(bank_origin, current_block.flag);

      // Calculate difference between current and next block's azimuth angle.
      uint16_t azimuth = (block == 0) ? current_block.rotation : azimuth_next;

      float azimuth_diff = NAN;
      if (block < static_cast<uint>(velodyne_packet::g_blocks_per_packet - (1 + dual_return))) {
        // Get the next block rotation to calculate how far we rotate between blocks
        azimuth_next = raw_instance.blocks[block + (1 + dual_return)].rotation;

        // Finds the difference between two successive blocks
        azimuth_diff = static_cast<float>((36000 + azimuth_next - azimuth) % 36000);

        // This is used when the last block is next to predict rotation amount
        last_azimuth_diff = azimuth_diff;
      } else {
        // This makes the assumption the difference between the last block and the next packet is
        // the same as the last to the second to last. Assumes RPM doesn't change much between
        // blocks.
        azimuth_diff = (block == static_cast<float>(
                                   velodyne_packet::g_blocks_per_packet - (4 * dual_return) - 1))
                         ? 0
                         : last_azimuth_diff;
      }
      assert(!std::isnan(azimuth_diff));

      // Condition added to avoid calculating points which are not in the interesting defined area
      // (cloud_min_angle <= area <= cloud_max_angle).
      if (
        (sensor_configuration_->cloud_min_angle >= sensor_configuration_->cloud_max_angle ||
         azimuth < sensor_configuration_->cloud_min_angle * 100 ||
         azimuth > sensor_configuration_->cloud_max_angle * 100) &&
        (sensor_configuration_->cloud_min_angle <= sensor_configuration_->cloud_max_angle ||
         (azimuth > sensor_configuration_->cloud_max_angle * 100 &&
          azimuth < sensor_configuration_->cloud_min_angle * 100))) {
        continue;
      }

      for (int unit_idx = 0; unit_idx < velodyne_packet::g_channels_per_block; ++unit_idx) {
        uint16_t other_distance = 0;
        int firing_seq = sensor_.get_firing_order(unit_idx, SensorT::channels_per_firing_sequence);
        int channel = sensor_.get_channel_number(unit_idx);

        // Distance extraction.
        uint16_t current_distance = current_block.units[unit_idx].distance;
        if (dual_return) {
          other_distance = block % 2 ? raw_instance.blocks[block - 1].units[unit_idx].distance
                                     : raw_instance.blocks[block + 1].units[unit_idx].distance;
        }

        // Apply timestamp if this is the first new packet in the scan.
        auto block_timestamp = packet_seconds;
        if (scan_timestamp_ < 0) {
          scan_timestamp_ = block_timestamp;
        }

        // Do not process if there is no return, or in dual return mode and the first and last
        // echos are the same.
        if (
          (current_distance == 0) ||
          (dual_return && block % 2 && current_distance == other_distance)) {
          continue;
        }

        const uint laser_number =
          channel + bank_origin;  // offset the laser in this block by which block it's in
        const uint firing_order =
          laser_number / SensorT::num_simultaneous_firings;  // VLS-128 fires 8 lasers at a
                                                             // time, VLP32 = 2, VLP16 = 1

        const VelodyneLaserCorrection & corrections =
          calibration_configuration_->velodyne_calibration.laser_corrections[laser_number];

        float distance = current_distance * SensorT::distance_resolution_m;
        if (distance > 1e-6) {
          distance += corrections.dist_correction;
        }

        if (
          distance <= sensor_configuration_->min_range ||
          distance >= sensor_configuration_->max_range) {
          continue;
        }
        // Correct for the laser rotation as a function of timing during the firings.
        uint16_t azimuth_corrected =
          sensor_.get_azimuth_corrected(azimuth, azimuth_diff, firing_seq, firing_order);

        // convert polar coordinates to Euclidean XYZ.
        const float cos_vert_angle = corrections.cos_vert_correction;
        const float sin_vert_angle = corrections.sin_vert_correction;
        const float cos_rot_correction = corrections.cos_rot_correction;
        const float sin_rot_correction = corrections.sin_rot_correction;

        // select correct azimuth if vlp32 current_block.rotation, if vlp128 and vlp16
        // azimuth_corrected
        azimuth_corrected = sensor_.get_true_rotation(azimuth_corrected, current_block.rotation);
        const float cos_rot_angle = cos_rot_table_[azimuth_corrected] * cos_rot_correction +
                                    sin_rot_table_[azimuth_corrected] * sin_rot_correction;
        const float sin_rot_angle = sin_rot_table_[azimuth_corrected] * cos_rot_correction -
                                    cos_rot_table_[azimuth_corrected] * sin_rot_correction;

        // Compute the distance in the xy plane (w/o accounting for rotation).
        const float xy_distance = distance * cos_vert_angle;

        // Use standard ROS coordinate system (right-hand rule).
        const float x_coord = xy_distance * cos_rot_angle;     // velodyne y
        const float y_coord = -(xy_distance * sin_rot_angle);  // velodyne x
        const float z_coord = distance * sin_vert_angle;       // velodyne z

        const uint8_t intensity = current_block.units[unit_idx].reflectivity;

        last_block_timestamp_ = block_timestamp;

        double point_time_offset = timing_offsets_[block][channel];

        // Determine return type.
        auto return_type = static_cast<uint8_t>(ReturnType::UNKNOWN);
        switch (return_mode) {
          case velodyne_packet::g_return_mode_dual:
            if ((other_distance == 0) || (other_distance == current_distance)) {
              return_type = static_cast<uint8_t>(drivers::ReturnType::IDENTICAL);
            } else {
              const uint8_t other_intensity =
                block % 2 ? raw_instance.blocks[block - 1].units[unit_idx].reflectivity
                          : raw_instance.blocks[block + 1].units[unit_idx].reflectivity;

              // TODO(ike-kazu): understand why this differs from VLP16 original decoder
              bool first = other_distance >= current_distance;

              bool strongest = other_intensity < intensity;
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
          case velodyne_packet::g_return_mode_strongest:
            return_type = static_cast<uint8_t>(drivers::ReturnType::STRONGEST);
            break;
          case velodyne_packet::g_return_mode_last:
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
        current_point.distance = distance;
        auto point_ts = block_timestamp - scan_timestamp_ + point_time_offset;
        if (point_ts < 0) point_ts = 0;
        current_point.time_stamp = static_cast<uint32_t>(point_ts * 1e9);
        current_point.intensity = intensity;
        scan_pc_->points.emplace_back(current_point);
      }
    }
  }

  bool parse_packet(
    [[maybe_unused]] const velodyne_msgs::msg::VelodynePacket & velodyne_packet) override
  {
    return false;
  }

private:
  /// @brief Parsing VelodynePacket based on packet structure
  /// @param velodyne_packet
  /// @return Resulting flag

  // params used by all velodyne decoders
  float sin_rot_table_[velodyne_packet::g_rotation_max_units]{};
  float cos_rot_table_[velodyne_packet::g_rotation_max_units]{};
  float rotation_radians_[velodyne_packet::g_rotation_max_units]{};
  int phase_;
  int max_pts_{};
  double last_block_timestamp_{};
  std::vector<std::vector<float>> timing_offsets_;

  SensorT sensor_;
};

}  // namespace nebula::drivers
