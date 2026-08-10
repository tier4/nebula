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

#include "nebula_core_decoders/point_filters/blockage_mask.hpp"
#include "nebula_core_decoders/point_filters/downsample_mask.hpp"
#include "nebula_core_decoders/scan_cutter.hpp"
#include "nebula_hesai_decoders/decoders/angle_corrector.hpp"
#include "nebula_hesai_decoders/decoders/functional_safety.hpp"
#include "nebula_hesai_decoders/decoders/hesai_packet.hpp"
#include "nebula_hesai_decoders/decoders/hesai_scan_decoder.hpp"
#include "nebula_hesai_decoders/decoders/packet_loss_detector.hpp"

#ifdef NEBULA_CUDA_ENABLED
#include "nebula_hesai_decoders/cuda/hesai_cuda_decoder.hpp"
#endif

#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/stopwatch.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers
{

template <typename SensorT>
class HesaiDecoder : public HesaiScanDecoder
{
private:
  struct DecodeFrame
  {
    NebulaPointCloudPtr pointcloud;
    uint64_t scan_timestamp_ns{0};
    std::optional<point_filters::BlockageMask> blockage_mask;
  };

  /// @brief Configuration for this decoder
  const std::shared_ptr<const drivers::HesaiSensorConfiguration> sensor_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  /// @brief A function that is called on each decoded pointcloud frame
  pointcloud_callback_t pointcloud_callback_;

  /// @brief Decodes azimuth/elevation angles given calibration/correction data
  typename SensorT::angle_corrector_t angle_corrector_;

  /// @brief Keeps track of scan cutting state
  ScanCutter<SensorT::packet_t::n_channels, float> scan_cutter_;

  std::shared_ptr<FunctionalSafetyDecoderTypedBase<typename SensorT::packet_t>>
    functional_safety_decoder_;
  std::shared_ptr<PacketLossDetectorTypedBase<typename SensorT::packet_t>> packet_loss_detector_;

  typename SensorT::packet_t packet_;

  /// @brief Accumulated callback time during the current unpack() call (reset per packet)
  uint64_t callback_time_ns_{0};
  /// @brief Whether a scan was completed during the current unpack() call (reset per packet)
  bool did_scan_complete_{false};
  /// @brief The current block being processed (used for timestamp reset calculation)
  size_t current_block_id_{0};

  std::shared_ptr<loggers::Logger> logger_;

  /// @brief For each channel, its firing offset relative to the block in nanoseconds
  std::array<int, SensorT::packet_t::n_channels> channel_firing_offset_ns_;
  /// @brief For each return mode, the firing offset of each block relative to its packet in
  /// nanoseconds
  std::array<std::array<int, SensorT::packet_t::n_blocks>, SensorT::packet_t::max_returns>
    block_firing_offset_ns_;

  std::optional<point_filters::DownsampleMaskFilter> mask_filter_;

  std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin_;

  std::array<DecodeFrame, 2> frame_buffers_{initialize_frame(), initialize_frame()};

#ifdef NEBULA_CUDA_ENABLED
  /// @brief RAII CUDA decoder managing all GPU resources for batched scan decoding.
  /// nullptr when CUDA is disabled or initialization failed.
  std::unique_ptr<cuda::HesaiScanDecoderCuda> cuda_scan_decoder_;

  /// @brief Number of azimuth divisions for angle lookup table (LUT resolution)
  static constexpr uint32_t cuda_n_azimuths_ = 36000;  // 0.01 degree resolution
  /// @brief Sensor's native azimuth range (max_azimuth = 360 * degree_subdivisions)
  static constexpr uint32_t sensor_max_azimuth_ = 360 * SensorT::packet_t::degree_subdivisions;
  /// @brief Scale factor from sensor native azimuth to LUT index
  static constexpr uint32_t azimuth_scale_ = sensor_max_azimuth_ / cuda_n_azimuths_;

  static constexpr uint32_t MAX_PACKETS_PER_SCAN = 4000;
#endif  // NEBULA_CUDA_ENABLED

  /// @brief Validates and parse PandarPacket. Checks size and, if present, CRC checksums.
  /// @param packet The incoming PandarPacket
  /// @return Whether the packet was parsed successfully
  bool parse_packet(const std::vector<uint8_t> & packet)
  {
    if (packet.size() < sizeof(typename SensorT::packet_t)) {
      NEBULA_LOG_STREAM(
        logger_->error, "Packet size mismatch: " << packet.size() << " | Expected at least: "
                                                 << sizeof(typename SensorT::packet_t));
      return false;
    }

    if (!std::memcpy(&packet_, packet.data(), sizeof(typename SensorT::packet_t))) {
      logger_->error("Packet memcopy failed");
      return false;
    }

    return true;
  }

#ifdef NEBULA_CUDA_ENABLED
  /// @brief Accumulate block group data to GPU scan buffer for batch processing
  /// @param start_block_id The first block in the group of returns
  /// @param n_blocks The number of returns in the group
  /// @param scan_state CPU ScanCutter state with per-channel FOV and buffer assignments
  void accumulate_packet_to_gpu_buffer(
    size_t start_block_id, size_t n_blocks,
    const typename decltype(scan_cutter_)::State & scan_state)
  {
    if (cuda_scan_decoder_->packet_count() >= cuda_scan_decoder_->max_packets()) {
      NEBULA_LOG_STREAM(logger_->warn, "GPU scan buffer full, dropping block group");
      return;
    }

    const uint32_t entry_id = cuda_scan_decoder_->packet_count();
    const size_t n_channels = SensorT::packet_t::n_channels;
    const size_t max_returns = SensorT::packet_t::max_returns;

    // Store metadata for this block group
    uint32_t raw_azimuth = packet_.body.blocks[start_block_id].get_azimuth();
    cuda_scan_decoder_->raw_azimuths_staging()[entry_id] = raw_azimuth;
    cuda_scan_decoder_->n_returns_staging()[entry_id] = n_blocks;
    cuda_scan_decoder_->packet_timestamps_staging()[entry_id] =
      hesai_packet::get_timestamp_ns(packet_);

    // Store per-channel scan flags from CPU ScanCutter:
    // - 0xFF = channel not in FOV (GPU will skip)
    // - buffer_index (0 or 1) = which frame buffer this channel belongs to
    // The buffer_index is remapped to in_current_scan (0/1) at flush time.
    const size_t flag_offset = entry_id * n_channels;
    for (size_t ch = 0; ch < n_channels; ++ch) {
      cuda_scan_decoder_->scan_flags_staging()[flag_offset + ch] =
        scan_state.channels_in_fov[ch] ? scan_state.channel_buffer_indices[ch] : 0xFF;
    }

    // Extract distances/reflectivities to pinned host memory
    // Layout: [entry][channel][return] with max_returns stride
    const size_t entry_offset = entry_id * n_channels * max_returns;

    for (size_t ch = 0; ch < n_channels; ++ch) {
      for (size_t blk = 0; blk < n_blocks; ++blk) {
        const auto & unit = packet_.body.blocks[start_block_id + blk].units[ch];
        const size_t idx = entry_offset + ch * max_returns + blk;
        cuda_scan_decoder_->distances_staging()[idx] = unit.distance;
        cuda_scan_decoder_->reflectivities_staging()[idx] = unit.reflectivity;
      }
    }

    cuda_scan_decoder_->increment_packet_count();
  }

  /// @brief Build the CudaDecoderConfig for a batched kernel launch
  /// @param n_entries Number of block-group entries accumulated
  /// @return Populated config ready for GPU upload
  cuda::CudaDecoderConfig build_batch_config(uint32_t n_entries)
  {
    const size_t n_channels = SensorT::packet_t::n_channels;
    const size_t max_returns = SensorT::packet_t::max_returns;

    cuda::CudaDecoderConfig config{};
    config.min_range = sensor_configuration_->min_range;
    config.max_range = sensor_configuration_->max_range;
    config.sensor_min_range = SensorT::min_range;
    config.sensor_max_range = SensorT::max_range;
    config.dual_return_distance_threshold = sensor_configuration_->dual_return_distance_threshold;
    config.n_channels = n_channels;
    config.max_returns = max_returns;
    config.dis_unit = hesai_packet::get_dis_unit(packet_);
    config.azimuth_scale = azimuth_scale_;
    config.max_output_points = n_entries * n_channels * max_returns;

    return config;
  }

  /// @brief Copy GPU results from host buffer and place into correct frame buffers
  /// @param completed_buffer_index The buffer index of the just-completed scan
  /// @param n_entries Number of block-group entries in the scan
  void process_gpu_results(uint8_t completed_buffer_index, uint32_t n_entries)
  {
    const auto & host_buffer = cuda_scan_decoder_->host_point_buffer();
    const uint32_t sparse_buffer_size =
      n_entries * SensorT::packet_t::n_channels * SensorT::packet_t::max_returns;
    const uint32_t copy_size =
      std::min(sparse_buffer_size, static_cast<uint32_t>(host_buffer.size()));

    // Iterate sparse buffer, skip invalid points (distance <= 0)
    for (uint32_t i = 0; i < copy_size; ++i) {
      const auto & cuda_pt = host_buffer[i];

      if (cuda_pt.distance <= 0.0f) {
        continue;
      }

      // in_current_scan=1: belongs to the completed scan (completed_buffer_index)
      // in_current_scan=0: belongs to the next scan (1 - completed_buffer_index)
      auto & frame = cuda_pt.in_current_scan ? frame_buffers_[completed_buffer_index]
                                             : frame_buffers_[1 - completed_buffer_index];

      const uint32_t entry_id = cuda_pt.entry_id;
      const uint64_t packet_timestamp_ns =
        (entry_id < n_entries) ? cuda_scan_decoder_->packet_timestamps_staging()[entry_id]
                               : hesai_packet::get_timestamp_ns(packet_);

      NebulaPoint point;
      point.x = cuda_pt.x;
      point.y = cuda_pt.y;
      point.z = cuda_pt.z;
      point.distance = cuda_pt.distance;
      point.azimuth = cuda_pt.azimuth;
      point.elevation = cuda_pt.elevation;
      point.intensity = cuda_pt.intensity;
      point.return_type = cuda_pt.return_type;
      point.channel = cuda_pt.channel;
      // Compute relative timestamp in signed 64-bit to avoid underflow.
      {
        auto point_to_packet_offset_ns =
          sensor_.get_packet_relative_point_time_offset(0, cuda_pt.channel, packet_);
        int64_t rel_ns = static_cast<int64_t>(packet_timestamp_ns) -
                         static_cast<int64_t>(frame.scan_timestamp_ns) + point_to_packet_offset_ns;
        point.time_stamp = (rel_ns >= 0) ? static_cast<uint32_t>(rel_ns) : 0;
      }

      if (!mask_filter_ || !mask_filter_->excluded(point)) {
        frame.pointcloud->emplace_back(point);
      }
    }
  }

  /// @brief Flush accumulated packets - one batched kernel launch for the entire scan
  /// @param completed_buffer_index The buffer index of the just-completed scan
  void flush_gpu_scan_buffer(uint8_t completed_buffer_index)
  {
    if (cuda_scan_decoder_->packet_count() == 0) return;

    const uint32_t n_entries = cuda_scan_decoder_->packet_count();
    const size_t n_channels = SensorT::packet_t::n_channels;

    // Remap scan_flags: raw buffer_index (0/1) -> in_current_scan (1/0), 0xFF stays
    for (uint32_t e = 0; e < n_entries; ++e) {
      for (size_t ch = 0; ch < n_channels; ++ch) {
        uint8_t & flag = cuda_scan_decoder_->scan_flags_staging()[e * n_channels + ch];
        if (flag == 0xFF) continue;
        flag = (flag == completed_buffer_index) ? 1 : 0;
      }
    }

    cuda::CudaDecoderConfig config = build_batch_config(n_entries);
    cuda_scan_decoder_->transfer_to_device(n_entries);

    uint32_t valid_point_count = cuda_scan_decoder_->launch_and_sync(config, n_entries);
    if (valid_point_count == 0) {
      cuda_scan_decoder_->reset_packet_count();
      return;
    }

    process_gpu_results(completed_buffer_index, n_entries);
    cuda_scan_decoder_->reset_packet_count();
  }

  /// @brief Initialize CUDA decoder and upload angle corrections.
  /// CUDA decode is opt-in: set NEBULA_USE_CUDA=1 environment variable to enable.
  /// Scan boundaries are driven by the CPU ScanCutter, so GPU output is identical to CPU.
  void initialize_cuda()
  {
    const char * cuda_env = std::getenv("NEBULA_USE_CUDA");
    if (!cuda_env || std::string(cuda_env) != "1") {
      NEBULA_LOG_STREAM(logger_->info, "CUDA decode disabled (set NEBULA_USE_CUDA=1 to enable)");
      return;
    }

    const uint32_t n_channels = SensorT::packet_t::n_channels;
    const uint32_t max_returns = SensorT::packet_t::max_returns;
    const size_t max_sparse_buffer_points =
      static_cast<size_t>(MAX_PACKETS_PER_SCAN) * n_channels * max_returns;
    const size_t max_output_points =
      std::max(static_cast<size_t>(SensorT::max_scan_buffer_points), max_sparse_buffer_points);

    try {
      cuda_scan_decoder_ = std::make_unique<cuda::HesaiScanDecoderCuda>(
        n_channels, max_returns, MAX_PACKETS_PER_SCAN, max_output_points, cuda_n_azimuths_,
        azimuth_scale_);
    } catch (const std::runtime_error & e) {
      NEBULA_LOG_STREAM(logger_->warn, "Failed to initialize CUDA decoder: " << e.what());
      return;
    }

    // Build and upload angle correction lookup table
    std::vector<cuda::CudaAngleCorrectionData> angle_lut;
    angle_lut.reserve(cuda_n_azimuths_ * n_channels);

    NEBULA_LOG_STREAM(
      logger_->info, "Building CUDA angle LUT: azimuth_scale="
                       << azimuth_scale_ << " sensor_max_azimuth=" << sensor_max_azimuth_
                       << " cuda_n_azimuths=" << cuda_n_azimuths_);

    for (uint32_t lut_idx = 0; lut_idx < cuda_n_azimuths_; ++lut_idx) {
      uint32_t sensor_azimuth = lut_idx * azimuth_scale_;
      for (uint32_t channel = 0; channel < n_channels; ++channel) {
        CorrectedAngleData cpu_data =
          angle_corrector_.get_corrected_angle_data(sensor_azimuth, channel);
        cuda::CudaAngleCorrectionData gpu_data;
        gpu_data.azimuth_rad = cpu_data.azimuth_rad;
        gpu_data.elevation_rad = cpu_data.elevation_rad;
        gpu_data.sin_azimuth = cpu_data.sin_azimuth;
        gpu_data.cos_azimuth = cpu_data.cos_azimuth;
        gpu_data.sin_elevation = cpu_data.sin_elevation;
        gpu_data.cos_elevation = cpu_data.cos_elevation;
        angle_lut.push_back(gpu_data);
      }
    }

    if (!cuda_scan_decoder_->upload_angle_corrections(angle_lut, cuda_n_azimuths_, n_channels)) {
      NEBULA_LOG_STREAM(logger_->warn, "Failed to upload CUDA angle corrections");
      cuda_scan_decoder_.reset();
      return;
    }

    NEBULA_LOG_STREAM(
      logger_->info, "CUDA decoder initialized successfully with " << n_channels << " channels and "
                                                                   << cuda_n_azimuths_
                                                                   << " azimuth divisions");
  }
#endif  // NEBULA_CUDA_ENABLED

  /// @brief Converts a group of returns (i.e. 1 for single return, 2 for dual return, etc.) to
  /// points and appends them to the point cloud
  /// @param start_block_id The first block in the group of returns
  /// @param n_blocks The number of returns in the group (has to align with the `n_returns` field in
  /// the packet footer)
  void convert_returns(
    size_t start_block_id, size_t n_blocks,
    const typename decltype(scan_cutter_)::State & scan_state)
  {
    uint64_t packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    uint32_t raw_azimuth = packet_.body.blocks[start_block_id].get_azimuth();

    std::vector<const typename SensorT::packet_t::body_t::block_t::unit_t *> return_units;

    // If the blockage mask plugin is not present, we can return early if distance checks fail
    const bool filters_can_return_early = !blockage_mask_plugin_;

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

        bool point_is_valid = true;

        if (unit.distance == 0) {
          point_is_valid = false;
        }

        float distance = get_distance(unit);

        if (
          distance < SensorT::min_range || SensorT::max_range < distance ||
          distance < sensor_configuration_->min_range ||
          sensor_configuration_->max_range < distance) {
          point_is_valid = false;
        }

        auto return_type = sensor_.get_return_type(
          static_cast<hesai_packet::return_mode::ReturnMode>(packet_.tail.return_mode),
          block_offset, return_units);

        // Keep only last of multiple identical points
        if (return_type == ReturnType::IDENTICAL && block_offset != n_blocks - 1) {
          point_is_valid = false;
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
            point_is_valid = false;
          }
        }

        if (filters_can_return_early && !point_is_valid) {
          continue;
        }

        if (!scan_state.channels_in_fov[channel_id]) {
          continue;
        }

        CorrectedAngleData corrected_angle_data =
          angle_corrector_.get_corrected_angle_data(raw_azimuth, channel_id);
        auto & frame = frame_buffers_[scan_state.channel_buffer_indices[channel_id]];

        float azimuth = corrected_angle_data.azimuth_rad;
        if (frame.blockage_mask) {
          frame.blockage_mask->update(
            azimuth, channel_id, sensor_.get_blockage_type(unit.distance));
        }

        if (!point_is_valid) {
          continue;
        }

        NebulaPoint point;
        point.distance = distance;
        point.intensity = unit.reflectivity;
        point.time_stamp = get_point_time_relative(
          frame.scan_timestamp_ns, packet_timestamp_ns, block_offset + start_block_id, channel_id);

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

        if (!mask_filter_ || !mask_filter_->excluded(point)) {
          frame.pointcloud->emplace_back(point);
        }
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

  DecodeFrame initialize_frame() const
  {
    DecodeFrame frame = {std::make_shared<NebulaPointCloud>(), 0, std::nullopt};
    frame.pointcloud->reserve(SensorT::max_scan_buffer_points);

    if (blockage_mask_plugin_) {
      frame.blockage_mask = point_filters::BlockageMask(
        SensorT::fov_mdeg.azimuth, blockage_mask_plugin_->get_bin_width_mdeg(),
        SensorT::packet_t::n_channels);
    }

    return frame;
  }

  /// @brief Called when a scan is complete, published and then clears the output frame.
  void on_scan_complete(uint8_t buffer_index)
  {
    did_scan_complete_ = true;

#ifdef NEBULA_CUDA_ENABLED
    if (cuda_scan_decoder_ && cuda_scan_decoder_->packet_count() > 0) {
      flush_gpu_scan_buffer(buffer_index);
    }
#endif

    auto & completed_frame = frame_buffers_[buffer_index];
    constexpr uint64_t nanoseconds_per_second = 1'000'000'000ULL;
    double scan_timestamp_s =
      static_cast<double>(completed_frame.scan_timestamp_ns / nanoseconds_per_second) +
      (static_cast<double>(completed_frame.scan_timestamp_ns % nanoseconds_per_second) / 1e9);

    if (pointcloud_callback_) {
      util::Stopwatch stopwatch;
      pointcloud_callback_(completed_frame.pointcloud, scan_timestamp_s);
      callback_time_ns_ +=
        stopwatch.elapsed_ns();  // Accumulate in case of multiple scans per packet
    }

    if (blockage_mask_plugin_ && completed_frame.blockage_mask) {
      blockage_mask_plugin_->callback_and_reset(
        completed_frame.blockage_mask.value(), scan_timestamp_s);
    }

    completed_frame.pointcloud->clear();
  }

  void on_set_timestamp(uint8_t buffer_index)
  {
    auto & frame = frame_buffers_[buffer_index];
    frame.scan_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    frame.scan_timestamp_ns +=
      sensor_.get_earliest_point_time_offset_for_block(current_block_id_, packet_);
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param correction_data Calibration data for this decoder
  explicit HesaiDecoder(
    const std::shared_ptr<const HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const typename SensorT::angle_corrector_t::correction_data_t> &
      correction_data,
    const std::shared_ptr<loggers::Logger> & logger,
    const std::shared_ptr<FunctionalSafetyDecoderTypedBase<typename SensorT::packet_t>> &
      functional_safety_decoder,
    const std::shared_ptr<PacketLossDetectorTypedBase<typename SensorT::packet_t>> &
      packet_loss_detector,
    std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin)
  : sensor_configuration_(sensor_configuration),
    angle_corrector_(correction_data),
    scan_cutter_(
      2 * M_PIf, deg2rad(sensor_configuration_->cut_angle),
      deg2rad(sensor_configuration_->cloud_min_angle),
      deg2rad(sensor_configuration_->cloud_max_angle),
      [this](uint8_t buffer_index) { on_scan_complete(buffer_index); },
      [this](uint8_t buffer_index) { on_set_timestamp(buffer_index); }),
    functional_safety_decoder_(functional_safety_decoder),
    packet_loss_detector_(packet_loss_detector),
    logger_(logger),
    blockage_mask_plugin_(std::move(blockage_mask_plugin))
  {
    if (sensor_configuration->downsample_mask_path) {
      mask_filter_ = point_filters::DownsampleMaskFilter(
        sensor_configuration->downsample_mask_path.value(), SensorT::fov_mdeg.azimuth,
        SensorT::peak_resolution_mdeg.azimuth, SensorT::packet_t::n_channels,
        logger_->child("Downsample Mask"), true, sensor_.get_dither_transform());
    }

#ifdef NEBULA_CUDA_ENABLED
    initialize_cuda();
#endif
  }

  void set_pointcloud_callback(pointcloud_callback_t callback) override
  {
    pointcloud_callback_ = std::move(callback);
  }

  PacketDecodeResult unpack(const std::vector<uint8_t> & packet) override
  {
    util::Stopwatch decode_watch;
    callback_time_ns_ = 0;
    did_scan_complete_ = false;

    if (!parse_packet(packet)) {
      return {PerformanceCounters{decode_watch.elapsed_ns()}, DecodeError::PACKET_PARSE_FAILED};
    }

    if (packet_loss_detector_) {
      packet_loss_detector_->update(packet_);
    }

    // Even if the checksums of other parts of the packet are invalid, functional safety info
    // is still checked. This is a null-op for sensors that do not support functional safety.
    if (functional_safety_decoder_) {
      functional_safety_decoder_->update(packet_);
    }

    // FYI: This is where the CRC would be checked. Since this caused performance issues in the
    // past, and since the frame check sequence of the packet is already checked by the NIC, we skip
    // it here.

    const size_t n_returns = hesai_packet::get_n_returns(packet_.tail.return_mode);
    for (size_t block_id = 0; block_id < SensorT::packet_t::n_blocks; block_id += n_returns) {
      auto block_azimuth = packet_.body.blocks[block_id].get_azimuth();

      auto channel_azimuths_out = angle_corrector_.get_corrected_azimuths(block_azimuth);
      // Store current block ID for use in on_set_timestamp() callback
      current_block_id_ = block_id;
      const auto & scan_state = scan_cutter_.step(channel_azimuths_out);

      if (scan_state.does_block_intersect_fov()) {
#ifdef NEBULA_CUDA_ENABLED
        if (cuda_scan_decoder_) {
          accumulate_packet_to_gpu_buffer(block_id, n_returns, scan_state);
        } else {  // NOLINT(readability/braces)
#endif
          convert_returns(block_id, n_returns, scan_state);
#ifdef NEBULA_CUDA_ENABLED
        }
#endif
      }
    }

    uint64_t decode_duration_ns = decode_watch.elapsed_ns();

    PacketMetadata metadata;
    metadata.packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    metadata.did_scan_complete = did_scan_complete_;
    return {PerformanceCounters{decode_duration_ns - callback_time_ns_}, metadata};
  }
};

}  // namespace nebula::drivers
