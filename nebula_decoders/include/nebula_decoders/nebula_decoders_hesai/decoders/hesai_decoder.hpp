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

#include "nebula_decoders/nebula_decoders_common/angles.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/point_types.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace nebula::drivers
{

struct HesaiDecodeFilteredInfo
{
  uint64_t distance_filtered_count = 0;
  uint64_t fov_filtered_count = 0;
  uint64_t invalid_point_count = 0;
  uint64_t identical_filtered_count = 0;
  uint64_t multiple_return_filtered_count = 0;
  uint64_t total_kept_point_count = 0;
  uint64_t invalid_packet_count = 0;
  float cloud_distance_min_m = std::numeric_limits<float>::infinity();
  float cloud_distance_max_m = std::numeric_limits<float>::lowest();
  float cloud_azimuth_min_deg = std::numeric_limits<float>::infinity();
  float cloud_azimuth_max_rad = std::numeric_limits<float>::lowest();
  uint64_t packet_timestamp_min_ns = std::numeric_limits<uint64_t>::max();
  uint64_t packet_timestamp_max_ns = std::numeric_limits<uint64_t>::min();

  [[nodiscard]] nlohmann::ordered_json to_json() const
  {
    nlohmann::json distance_j;
    distance_j["name"] = "distance";
    distance_j["filtered_count"] = distance_filtered_count;
    // distance_j["cloud_distance_min_m"] = cloud_distance_min_m;
    // distance_j["cloud_distance_max_m"] = cloud_distance_max_m;
    nlohmann::json fov_j;
    fov_j["name"] = "fov";
    fov_j["filtered_count"] = fov_filtered_count;
    // fov_j["cloud_azimuth_min_deg"] = cloud_azimuth_min_deg;
    // fov_j["cloud_azimuth_max_rad"] = cloud_azimuth_max_rad;
    nlohmann::json identical_j;
    identical_j["name"] = "identical";
    identical_j["filtered_count"] = identical_filtered_count;
    nlohmann::json multiple_j;
    multiple_j["name"] = "multiple";
    multiple_j["filtered_count"] = multiple_return_filtered_count;
    nlohmann::json invalid_j;
    invalid_j["name"] = "invalid";
    invalid_j["invalid_point_count"] = invalid_point_count;
    invalid_j["invalid_packet_count"] = invalid_packet_count;
    nlohmann::json pointcloud_bounds_azimuth_j;
    pointcloud_bounds_azimuth_j["min"] = cloud_azimuth_min_deg;
    pointcloud_bounds_azimuth_j["max"] = cloud_azimuth_max_rad;
    nlohmann::json pointcloud_bounds_distance_j;
    pointcloud_bounds_distance_j["min"] = cloud_distance_min_m;
    pointcloud_bounds_distance_j["max"] = cloud_distance_max_m;
    nlohmann::json pointcloud_bounds_timestamp_j;
    pointcloud_bounds_timestamp_j["min"] = packet_timestamp_min_ns;
    pointcloud_bounds_timestamp_j["max"] = packet_timestamp_max_ns;

    nlohmann::json j;
    j["filter_pipeline"] = nlohmann::json::array({
      distance_j,
      fov_j,
      identical_j,
      multiple_j,
    });
    j["pointcloud_bounds"] ={
      j["azimuth_deg"] = pointcloud_bounds_azimuth_j,
      j["distance_m"] = pointcloud_bounds_distance_j,
      j["timestamp_ns"] = pointcloud_bounds_timestamp_j,
    };
    j["invalid_filter"] = invalid_j;
    j["total_kept_point_count"] = total_kept_point_count;

    return j;
  }

  void update_pointcloud_bounds(const NebulaPoint & point)
  {
    cloud_azimuth_min_deg = std::min(cloud_azimuth_min_deg, point.azimuth);
    cloud_azimuth_max_rad = std::max(cloud_azimuth_max_rad, point.azimuth);
    packet_timestamp_min_ns =
      std::min(packet_timestamp_min_ns, static_cast<uint64_t>(point.time_stamp));
    packet_timestamp_max_ns =
      std::max(packet_timestamp_max_ns, static_cast<uint64_t>(point.time_stamp));
    cloud_distance_min_m = std::min(cloud_distance_min_m, point.distance);
    cloud_distance_max_m = std::max(cloud_distance_max_m, point.distance);
  }
};

template <typename SensorT>
class HesaiDecoder : public HesaiScanDecoder
{
  struct ScanCutAngles
  {
    float fov_min;
    float fov_max;
    float scan_emit_angle;
  };

protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<const drivers::HesaiSensorConfiguration> sensor_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  /// @brief Decodes azimuth/elevation angles given calibration/correction data
  typename SensorT::angle_corrector_t angle_corrector_;

  /// @brief The point cloud new points get added to
  NebulaPointCloudPtr decode_pc_;
  /// @brief The point cloud that is returned when a scan is complete
  NebulaPointCloudPtr output_pc_;

  /// @brief The last decoded packet
  typename SensorT::packet_t packet_;

  /// @brief The timestamp of the last completed scan in nanoseconds
  uint64_t output_scan_timestamp_ns_ = 0;
  /// @brief The timestamp of the scan currently in progress
  uint64_t decode_scan_timestamp_ns_ = 0;
  /// @brief Whether a full scan has been processed
  bool has_scanned_ = false;

  ScanCutAngles scan_cut_angles_;
  uint32_t last_azimuth_ = 0;

  rclcpp::Logger logger_;

  // filtered pointcloud counter
  HesaiDecodeFilteredInfo decode_filtered_info_;

  /// @brief For each channel, its firing offset relative to the block in nanoseconds
  std::array<int, SensorT::packet_t::n_channels> channel_firing_offset_ns_;
  /// @brief For each return mode, the firing offset of each block relative to its packet in
  /// nanoseconds
  std::array<std::array<int, SensorT::packet_t::n_blocks>, SensorT::packet_t::max_returns>
    block_firing_offset_ns_;

  /// @brief Validates and parse PandarPacket. Currently only checks size, not checksums etc.
  /// @param packet The incoming PandarPacket
  /// @return Whether the packet was parsed successfully
  bool parse_packet(const std::vector<uint8_t> & packet)
  {
    if (packet.size() < sizeof(typename SensorT::packet_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch: " << packet.size() << " | Expected at least: "
                                          << sizeof(typename SensorT::packet_t));
      return false;
    }
    if (std::memcpy(&packet_, packet.data(), sizeof(typename SensorT::packet_t))) {
      // FIXME(mojomex) do validation?
      // RCLCPP_DEBUG(logger_, "Packet parsed successfully");
      return true;
    }

    RCLCPP_ERROR(logger_, "Packet memcopy failed");
    return false;
  }

  /// @brief Converts a group of returns (i.e. 1 for single return, 2 for dual return, etc.) to
  /// points and appends them to the point cloud
  /// @param start_block_id The first block in the group of returns
  /// @param n_blocks The number of returns in the group (has to align with the `n_returns` field in
  /// the packet footer)
  void convert_returns(size_t start_block_id, size_t n_blocks)
  {
    uint64_t packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    uint32_t raw_azimuth = packet_.body.blocks[start_block_id].get_azimuth();

    std::vector<const typename SensorT::packet_t::body_t::block_t::unit_t *> return_units;

    for (size_t channel_id = 0; channel_id < SensorT::packet_t::n_channels; ++channel_id) {
      // Find the units corresponding to the same return group as the current one.
      // These are used to find duplicates in multi-return mode.
      return_units.clear();
      for (size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
        return_units.push_back(
          &packet_.body.blocks[block_offset + start_block_id].units[channel_id]);
      }

      for (size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
        auto & unit = *return_units[block_offset];

        if (unit.distance == 0) {
          decode_filtered_info_.invalid_point_count++;
          continue;
        }

        float distance = get_distance(unit);

        if (
          distance < SensorT::min_range || SensorT::max_range < distance ||
          distance < sensor_configuration_->min_range ||
          sensor_configuration_->max_range < distance) {
          decode_filtered_info_.distance_filtered_count++;
          continue;
        }

        auto return_type = sensor_.get_return_type(
          static_cast<hesai_packet::return_mode::ReturnMode>(packet_.tail.return_mode),
          block_offset, return_units);

        // Keep only last of multiple identical points
        if (return_type == ReturnType::IDENTICAL && block_offset != n_blocks - 1) {
          decode_filtered_info_.identical_filtered_count++;
          continue;
        }

        // Keep only last (if any) of multiple points that are too close
        if (block_offset != n_blocks - 1) {
          bool is_below_multi_return_threshold = false;

          for (size_t return_idx = 0; return_idx < n_blocks; ++return_idx) {
            if (return_idx == block_offset) {
              continue;
            }

            if (
              fabsf(get_distance(*return_units[return_idx]) - distance) <
              sensor_configuration_->dual_return_distance_threshold) {
              is_below_multi_return_threshold = true;
              break;
            }
          }

          if (is_below_multi_return_threshold) {
            decode_filtered_info_.multiple_return_filtered_count++;
            continue;
          }
        }

        CorrectedAngleData corrected_angle_data =
          angle_corrector_.get_corrected_angle_data(raw_azimuth, channel_id);
        float azimuth = corrected_angle_data.azimuth_rad;

        bool in_fov = angle_is_between(scan_cut_angles_.fov_min, scan_cut_angles_.fov_max, azimuth);
        if (!in_fov) {
          decode_filtered_info_.fov_filtered_count++;
          continue;
        }

        bool in_current_scan = true;

        if (
          angle_corrector_.is_inside_overlap(last_azimuth_, raw_azimuth) &&
          angle_is_between(
            scan_cut_angles_.scan_emit_angle, scan_cut_angles_.scan_emit_angle + deg2rad(20),
            azimuth)) {
          in_current_scan = false;
        }

        auto pc = in_current_scan ? decode_pc_ : output_pc_;
        uint64_t scan_timestamp_ns =
          in_current_scan ? decode_scan_timestamp_ns_ : output_scan_timestamp_ns_;

        NebulaPoint & point = pc->emplace_back();
        point.distance = distance;
        point.intensity = unit.reflectivity;
        point.time_stamp = get_point_time_relative(
          scan_timestamp_ns, packet_timestamp_ns, block_offset + start_block_id, channel_id);

        point.return_type = static_cast<uint8_t>(return_type);
        point.channel = channel_id;

        // The raw_azimuth and channel are only used as indices, sin/cos functions use the precise
        // corrected angles
        float xy_distance = distance * corrected_angle_data.cos_elevation;
        point.x = xy_distance * corrected_angle_data.sin_azimuth;
        point.y = xy_distance * corrected_angle_data.cos_azimuth;
        point.z = distance * corrected_angle_data.sin_elevation;

        // The driver wrapper converts to degrees, expects radians
        point.azimuth = corrected_angle_data.azimuth_rad;
        point.elevation = corrected_angle_data.elevation_rad;

        decode_filtered_info_.update_pointcloud_bounds(point);
        decode_filtered_info_.total_kept_point_count++;
      }
    }
  }

  /// @brief Get the distance of the given unit in meters
  float get_distance(const typename SensorT::packet_t::body_t::block_t::unit_t & unit)
  {
    return unit.distance * hesai_packet::get_dis_unit(packet_);
  }

  /// @brief Get timestamp of point in nanoseconds, relative to scan timestamp. Includes firing time
  /// offset correction for channel and block
  /// @param scan_timestamp_ns Start timestamp of the current scan in nanoseconds
  /// @param packet_timestamp_ns The timestamp of the current PandarPacket in nanoseconds
  /// @param block_id The block index of the point
  /// @param channel_id The channel index of the point
  uint32_t get_point_time_relative(
    uint64_t scan_timestamp_ns, uint64_t packet_timestamp_ns, size_t block_id, size_t channel_id)
  {
    auto point_to_packet_offset_ns =
      sensor_.get_packet_relative_point_time_offset(block_id, channel_id, packet_);
    auto packet_to_scan_offset_ns = static_cast<uint32_t>(packet_timestamp_ns - scan_timestamp_ns);
    return packet_to_scan_offset_ns + point_to_packet_offset_ns;
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param correction_data Calibration data for this decoder
  explicit HesaiDecoder(
    const std::shared_ptr<const HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const typename SensorT::angle_corrector_t::correction_data_t> &
      correction_data)
  : sensor_configuration_(sensor_configuration),
    angle_corrector_(
      correction_data, sensor_configuration_->cloud_min_angle,
      sensor_configuration_->cloud_max_angle, sensor_configuration_->cut_angle),
    logger_(rclcpp::get_logger("HesaiDecoder"))
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);

    decode_pc_ = std::make_shared<NebulaPointCloud>();
    output_pc_ = std::make_shared<NebulaPointCloud>();

    decode_pc_->reserve(SensorT::max_scan_buffer_points);
    output_pc_->reserve(SensorT::max_scan_buffer_points);

    scan_cut_angles_ = {
      deg2rad(sensor_configuration_->cloud_min_angle),
      deg2rad(sensor_configuration_->cloud_max_angle), deg2rad(sensor_configuration_->cut_angle)};
  }

  int unpack(const std::vector<uint8_t> & packet) override
  {
    if (!parse_packet(packet)) {
      decode_filtered_info_.invalid_packet_count++;
      return -1;
    }

    // This is the first scan, set scan timestamp to whatever packet arrived first
    if (decode_scan_timestamp_ns_ == 0) {
      decode_scan_timestamp_ns_ = hesai_packet::get_timestamp_ns(packet_) +
                                  sensor_.get_earliest_point_time_offset_for_block(0, packet_);
    }

    if (has_scanned_) {
      output_pc_->clear();
      has_scanned_ = false;
    }

    const size_t n_returns = hesai_packet::get_n_returns(packet_.tail.return_mode);
    for (size_t block_id = 0; block_id < SensorT::packet_t::n_blocks; block_id += n_returns) {
      auto block_azimuth = packet_.body.blocks[block_id].get_azimuth();

      if (angle_corrector_.passed_timestamp_reset_angle(last_azimuth_, block_azimuth)) {
        uint64_t new_scan_timestamp_ns =
          hesai_packet::get_timestamp_ns(packet_) +
          sensor_.get_earliest_point_time_offset_for_block(block_id, packet_);

        if (sensor_configuration_->cut_angle == sensor_configuration_->cloud_max_angle) {
          // In the non-360 deg case, if the cut angle and FoV end coincide, the old pointcloud has
          // already been swapped and published before the timestamp reset angle is reached. Thus,
          // the `decode` pointcloud is now empty and will be decoded to. Reset its timestamp.
          decode_scan_timestamp_ns_ = new_scan_timestamp_ns;
        } else {
          /// When not cutting at the end of the FoV (i.e. the FoV is 360 deg or a cut occurs
          /// somewhere within a non-360 deg FoV), the current scan is still being decoded to the
          /// `decode` pointcloud but at the same time, points for the next pointcloud are arriving
          /// and will be decoded to the `output` pointcloud (please forgive the naming for now).
          /// Thus, reset the output pointcloud's timestamp.
          output_scan_timestamp_ns_ = new_scan_timestamp_ns;
        }
      }

      if (!angle_corrector_.is_inside_fov(last_azimuth_, block_azimuth)) {
        last_azimuth_ = block_azimuth;
        continue;
      }

      convert_returns(block_id, n_returns);

      if (angle_corrector_.passed_emit_angle(last_azimuth_, block_azimuth)) {
        // The current `decode` pointcloud is ready for publishing, swap buffers to continue with
        // the last `output` pointcloud as the `decode pointcloud.
        std::swap(decode_pc_, output_pc_);
        std::swap(decode_scan_timestamp_ns_, output_scan_timestamp_ns_);
        has_scanned_ = true;
        nlohmann::ordered_json j = decode_filtered_info_.to_json();
        std::cout << "=======================" << std::endl;
        for (const auto & [key, value] : j.items()) {
          std::cout << key << ": " << std::endl;
          for (const auto & [k, v] : value.items()) {
            std::cout << k << ": " << v << std::endl;
          }
        }
        std::cout << "=======================" << std::endl;
        decode_filtered_info_ = HesaiDecodeFilteredInfo{};
      }

      last_azimuth_ = block_azimuth;
    }

    return last_azimuth_;
  }

  bool has_scanned() override { return has_scanned_; }

  std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() override
  {
    double scan_timestamp_s = static_cast<double>(output_scan_timestamp_ns_) * 1e-9;
    return std::make_pair(output_pc_, scan_timestamp_s);
  }
};

}  // namespace nebula::drivers
