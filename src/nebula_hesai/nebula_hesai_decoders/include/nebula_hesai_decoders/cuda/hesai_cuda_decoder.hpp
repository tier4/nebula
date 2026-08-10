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

#pragma once

#include <cuda_runtime.h>

#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <vector>

namespace nebula::drivers::cuda
{

/// @brief Point structure optimized for CUDA processing
struct CudaNebulaPoint
{
  float x;
  float y;
  float z;
  float distance;
  float azimuth;
  float elevation;
  float intensity;
  uint8_t return_type;
  uint8_t in_current_scan;  // 1 = belongs to current scan, 0 = belongs to output/next scan
  uint16_t channel;
  uint32_t entry_id;  // Block group ID for batched processing (used for sorting & filtering)
};

/// @brief Angle correction data for CUDA lookup table
struct CudaAngleCorrectionData
{
  float azimuth_rad;
  float elevation_rad;
  float sin_azimuth;
  float cos_azimuth;
  float sin_elevation;
  float cos_elevation;
};

/// @brief Configuration data for CUDA decoder
struct CudaDecoderConfig
{
  float min_range;
  float max_range;
  float sensor_min_range;
  float sensor_max_range;
  float dual_return_distance_threshold;
  uint32_t n_channels;
  uint32_t max_returns;
  float dis_unit;
  uint32_t max_output_points;  // Maximum output buffer size for sparse indexing (batched mode)
  uint32_t azimuth_scale;      // Scale factor: raw_azimuth / azimuth_scale = LUT index
};

}  // namespace nebula::drivers::cuda

// Forward declaration of C-linkage kernel launcher (defined in hesai_cuda_kernels.cu)
extern "C" {
bool launch_decode_hesai_scan_batch(
  const uint16_t * d_distances_batch, const uint8_t * d_reflectivities_batch,
  const uint32_t * d_raw_azimuths, const uint32_t * d_n_returns, const uint8_t * d_scan_flags,
  const nebula::drivers::cuda::CudaAngleCorrectionData * d_angle_lut,
  const nebula::drivers::cuda::CudaDecoderConfig & config,
  nebula::drivers::cuda::CudaNebulaPoint * d_points, uint32_t * d_count, uint32_t n_azimuths,
  uint32_t n_packets, cudaStream_t stream);
}  // extern "C"

namespace nebula::drivers::cuda
{

/// @brief RAII class managing all GPU resources for batched Hesai scan decoding.
///
/// Allocates device and pinned-host buffers in the constructor, frees them in the destructor.
/// Provides methods to accumulate packet data, launch the decode kernel, and retrieve results.
class HesaiScanDecoderCuda
{
public:
  /// @brief Construct and allocate all GPU resources.
  /// @param n_channels Number of LiDAR channels (e.g. 128 for OT128)
  /// @param max_returns Maximum returns per block (e.g. 2 for dual return)
  /// @param max_packets Maximum packets per scan
  /// @param max_output_points Upper bound on output points (for sparse buffer sizing)
  /// @param n_azimuths Number of azimuth divisions in the angle LUT
  /// @param azimuth_scale Scale factor from sensor azimuth to LUT index
  /// @throws std::runtime_error if any CUDA allocation fails
  HesaiScanDecoderCuda(
    uint32_t n_channels, uint32_t max_returns, uint32_t max_packets, size_t max_output_points,
    uint32_t n_azimuths, uint32_t azimuth_scale)
  : n_channels_(n_channels),
    max_returns_(max_returns),
    max_packets_(max_packets),
    max_output_points_(max_output_points),
    n_azimuths_(n_azimuths),
    azimuth_scale_(azimuth_scale)
  {
    auto check = [](cudaError_t err, const char * name) {
      if (err != cudaSuccess) {
        throw std::runtime_error(
          std::string("CUDA allocation failed for ") + name + ": " + cudaGetErrorString(err));
      }
    };

    check(cudaStreamCreate(&stream_), "CUDA stream");

    // Output buffers
    check(
      cudaMalloc(&d_points_, max_output_points_ * sizeof(CudaNebulaPoint)), "output points buffer");
    check(cudaMalloc(&d_count_, sizeof(uint32_t)), "output count buffer");
    host_point_buffer_.resize(max_output_points_);

    // Device buffers for batch data
    const size_t packet_data_size = static_cast<size_t>(n_channels_) * max_returns_;
    check(
      cudaMalloc(&d_distances_batch_, max_packets_ * packet_data_size * sizeof(uint16_t)),
      "scan distances buffer");
    check(
      cudaMalloc(&d_reflectivities_batch_, max_packets_ * packet_data_size * sizeof(uint8_t)),
      "scan reflectivities buffer");
    check(cudaMalloc(&d_raw_azimuths_, max_packets_ * sizeof(uint32_t)), "scan azimuths buffer");
    check(cudaMalloc(&d_n_returns_, max_packets_ * sizeof(uint32_t)), "scan n_returns buffer");
    check(
      cudaMalloc(&d_scan_flags_, max_packets_ * n_channels_ * sizeof(uint8_t)),
      "scan flags buffer");

    // Pinned host staging buffers
    check(
      cudaMallocHost(&h_distances_staging_, max_packets_ * packet_data_size * sizeof(uint16_t)),
      "pinned scan distances staging");
    check(
      cudaMallocHost(&h_reflectivities_staging_, max_packets_ * packet_data_size * sizeof(uint8_t)),
      "pinned scan reflectivities staging");
    check(
      cudaMallocHost(&h_raw_azimuths_staging_, max_packets_ * sizeof(uint32_t)),
      "pinned scan azimuths staging");
    check(
      cudaMallocHost(&h_n_returns_staging_, max_packets_ * sizeof(uint32_t)),
      "pinned scan n_returns staging");
    check(
      cudaMallocHost(&h_scan_flags_staging_, max_packets_ * n_channels_ * sizeof(uint8_t)),
      "pinned scan flags staging");
    check(
      cudaMallocHost(&h_packet_timestamps_staging_, max_packets_ * sizeof(uint64_t)),
      "pinned scan timestamps staging");

    // Angle LUT (allocated when upload_angle_corrections is called)
  }

  ~HesaiScanDecoderCuda()
  {
    auto free_device = [](auto *& ptr) {
      if (ptr) {
        cudaFree(ptr);
        ptr = nullptr;
      }
    };
    auto free_host = [](auto *& ptr) {
      if (ptr) {
        cudaFreeHost(ptr);
        ptr = nullptr;
      }
    };

    free_device(d_points_);
    free_device(d_count_);
    free_device(d_distances_batch_);
    free_device(d_reflectivities_batch_);
    free_device(d_raw_azimuths_);
    free_device(d_n_returns_);
    free_device(d_scan_flags_);
    free_device(d_angle_lut_);

    free_host(h_distances_staging_);
    free_host(h_reflectivities_staging_);
    free_host(h_raw_azimuths_staging_);
    free_host(h_n_returns_staging_);
    free_host(h_scan_flags_staging_);
    free_host(h_packet_timestamps_staging_);

    if (stream_) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
  }

  // Non-copyable, non-movable (raw CUDA pointers)
  HesaiScanDecoderCuda(const HesaiScanDecoderCuda &) = delete;
  HesaiScanDecoderCuda & operator=(const HesaiScanDecoderCuda &) = delete;
  HesaiScanDecoderCuda(HesaiScanDecoderCuda &&) = delete;
  HesaiScanDecoderCuda & operator=(HesaiScanDecoderCuda &&) = delete;

  /// @brief Upload angle correction lookup table to GPU
  bool upload_angle_corrections(
    const std::vector<CudaAngleCorrectionData> & angle_lut, uint32_t n_azimuths,
    uint32_t n_channels)
  {
    if (angle_lut.size() != static_cast<size_t>(n_azimuths) * n_channels) {
      fprintf(
        stderr, "CUDA: Angle LUT size mismatch: %zu vs expected %u\n", angle_lut.size(),
        n_azimuths * n_channels);
      return false;
    }

    if (d_angle_lut_) {
      cudaFree(d_angle_lut_);
      d_angle_lut_ = nullptr;
    }

    const size_t lut_size = angle_lut.size() * sizeof(CudaAngleCorrectionData);
    cudaError_t err = cudaMalloc(&d_angle_lut_, lut_size);
    if (err != cudaSuccess) {
      fprintf(stderr, "CUDA: Failed to allocate angle LUT: %s\n", cudaGetErrorString(err));
      return false;
    }

    err =
      cudaMemcpyAsync(d_angle_lut_, angle_lut.data(), lut_size, cudaMemcpyHostToDevice, stream_);
    if (err != cudaSuccess) {
      fprintf(stderr, "CUDA: Failed to upload angle LUT: %s\n", cudaGetErrorString(err));
      cudaFree(d_angle_lut_);
      d_angle_lut_ = nullptr;
      return false;
    }
    cudaStreamSynchronize(stream_);

    return true;
  }

  /// @brief Get current packet count in the accumulation buffer
  [[nodiscard]] uint32_t packet_count() const { return packet_count_; }

  /// @brief Get maximum packets per scan
  [[nodiscard]] uint32_t max_packets() const { return max_packets_; }

  /// @brief Reset packet counter (call after flush)
  void reset_packet_count() { packet_count_ = 0; }

  // ---- Staging buffer accessors (for accumulation by HesaiDecoder) ----

  uint16_t * distances_staging() { return h_distances_staging_; }
  uint8_t * reflectivities_staging() { return h_reflectivities_staging_; }
  uint32_t * raw_azimuths_staging() { return h_raw_azimuths_staging_; }
  uint32_t * n_returns_staging() { return h_n_returns_staging_; }
  uint8_t * scan_flags_staging() { return h_scan_flags_staging_; }
  uint64_t * packet_timestamps_staging() { return h_packet_timestamps_staging_; }

  void increment_packet_count() { ++packet_count_; }

  /// @brief Get device pointer to angle lookup table
  [[nodiscard]] CudaAngleCorrectionData * get_angle_lut() const { return d_angle_lut_; }

  /// @brief Get the CUDA stream
  [[nodiscard]] cudaStream_t stream() const { return stream_; }

  /// @brief Get number of azimuth divisions
  [[nodiscard]] uint32_t n_azimuths() const { return n_azimuths_; }

  /// @brief Get azimuth scale factor
  [[nodiscard]] uint32_t azimuth_scale() const { return azimuth_scale_; }

  /// @brief Get number of channels
  [[nodiscard]] uint32_t n_channels() const { return n_channels_; }

  /// @brief Get max returns
  [[nodiscard]] uint32_t max_returns() const { return max_returns_; }

  /// @brief Transfer accumulated scan data from pinned host memory to device
  void transfer_to_device(uint32_t n_entries)
  {
    const size_t total_data_size = static_cast<size_t>(n_entries) * n_channels_ * max_returns_;

    cudaMemcpyAsync(
      d_distances_batch_, h_distances_staging_, total_data_size * sizeof(uint16_t),
      cudaMemcpyHostToDevice, stream_);
    cudaMemcpyAsync(
      d_reflectivities_batch_, h_reflectivities_staging_, total_data_size * sizeof(uint8_t),
      cudaMemcpyHostToDevice, stream_);
    cudaMemcpyAsync(
      d_raw_azimuths_, h_raw_azimuths_staging_, n_entries * sizeof(uint32_t),
      cudaMemcpyHostToDevice, stream_);
    cudaMemcpyAsync(
      d_n_returns_, h_n_returns_staging_, n_entries * sizeof(uint32_t), cudaMemcpyHostToDevice,
      stream_);
    cudaMemcpyAsync(
      d_scan_flags_, h_scan_flags_staging_, n_entries * n_channels_ * sizeof(uint8_t),
      cudaMemcpyHostToDevice, stream_);
  }

  /// @brief Launch the decode kernel, synchronize, and copy results to host
  /// @param config Decoder configuration for this scan
  /// @param n_entries Number of accumulated block groups
  /// @return Number of valid decoded points
  uint32_t launch_and_sync(const CudaDecoderConfig & config, uint32_t n_entries)
  {
    const uint32_t sparse_buffer_size = n_entries * n_channels_ * max_returns_;

    // Reset output counter and zero output buffer for deterministic sparse indexing
    cudaMemsetAsync(d_count_, 0, sizeof(uint32_t), stream_);
    cudaMemsetAsync(d_points_, 0, sparse_buffer_size * sizeof(CudaNebulaPoint), stream_);

    // Launch batched kernel via the extern "C" wrapper (defined in hesai_cuda_kernels.cu)
    bool kernel_ok = launch_decode_hesai_scan_batch(
      d_distances_batch_, d_reflectivities_batch_, d_raw_azimuths_, d_n_returns_, d_scan_flags_,
      d_angle_lut_, config, d_points_, d_count_, n_azimuths_, n_entries, stream_);

    if (!kernel_ok) {
      return 0;
    }

    // Copy count and sparse buffer back to host using the same stream.
    // All operations on a stream execute in order, so no explicit sync needed before these copies.
    uint32_t valid_point_count = 0;
    cudaMemcpyAsync(
      &valid_point_count, d_count_, sizeof(uint32_t), cudaMemcpyDeviceToHost, stream_);

    const uint32_t copy_size =
      std::min(sparse_buffer_size, static_cast<uint32_t>(host_point_buffer_.size()));
    cudaMemcpyAsync(
      host_point_buffer_.data(), d_points_, copy_size * sizeof(CudaNebulaPoint),
      cudaMemcpyDeviceToHost, stream_);

    cudaStreamSynchronize(stream_);

    return valid_point_count;
  }

  /// @brief Access the host-side point buffer after launch_and_sync
  [[nodiscard]] const std::vector<CudaNebulaPoint> & host_point_buffer() const
  {
    return host_point_buffer_;
  }

private:
  // Configuration
  uint32_t n_channels_;
  uint32_t max_returns_;
  uint32_t max_packets_;
  size_t max_output_points_;
  uint32_t n_azimuths_;
  uint32_t azimuth_scale_;

  // CUDA stream
  cudaStream_t stream_ = nullptr;

  // Device output buffers
  CudaNebulaPoint * d_points_ = nullptr;
  uint32_t * d_count_ = nullptr;
  std::vector<CudaNebulaPoint> host_point_buffer_;

  // Device batch buffers
  uint16_t * d_distances_batch_ = nullptr;
  uint8_t * d_reflectivities_batch_ = nullptr;
  uint32_t * d_raw_azimuths_ = nullptr;
  uint32_t * d_n_returns_ = nullptr;
  uint8_t * d_scan_flags_ = nullptr;

  // Pinned host staging buffers
  uint16_t * h_distances_staging_ = nullptr;
  uint8_t * h_reflectivities_staging_ = nullptr;
  uint32_t * h_raw_azimuths_staging_ = nullptr;
  uint32_t * h_n_returns_staging_ = nullptr;
  uint8_t * h_scan_flags_staging_ = nullptr;
  uint64_t * h_packet_timestamps_staging_ = nullptr;

  // Angle correction LUT on device
  CudaAngleCorrectionData * d_angle_lut_ = nullptr;

  // Accumulation state
  uint32_t packet_count_ = 0;
};

}  // namespace nebula::drivers::cuda
