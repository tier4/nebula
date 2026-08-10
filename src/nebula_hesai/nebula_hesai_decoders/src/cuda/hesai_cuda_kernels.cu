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

#include "nebula_hesai_decoders/cuda/hesai_cuda_decoder.hpp"

#include <cuda_runtime.h>

#include <cstdio>
#include <cstring>

namespace nebula::drivers::cuda
{

/// @brief Check if a non-last return should be filtered as a duplicate.
/// Returns true if the point should be discarded.
__device__ __forceinline__ bool should_filter_duplicate_return(
  const uint16_t * __restrict__ d_distances_batch,
  const uint8_t * __restrict__ d_reflectivities_batch, uint32_t group_base, uint32_t return_id,
  uint32_t n_returns, uint16_t raw_distance, uint8_t reflectivity, float distance, float dis_unit,
  float threshold)
{
  // Last return is never filtered
  if (return_id >= n_returns - 1) return false;

  if (n_returns == 2) {
    const uint32_t last_idx = group_base + 1;
    const uint16_t last_raw_distance = d_distances_batch[last_idx];
    const uint8_t last_reflectivity = d_reflectivities_batch[last_idx];

    if (raw_distance == last_raw_distance && reflectivity == last_reflectivity) return true;

    const float last_distance = static_cast<float>(last_raw_distance) * dis_unit;
    return (fabsf(distance - last_distance) < threshold);
  } else {
    for (uint32_t other_ret = 0; other_ret < n_returns; ++other_ret) {
      if (other_ret == return_id) continue;

      const uint16_t other_raw_distance = d_distances_batch[group_base + other_ret];
      const uint8_t other_reflectivity = d_reflectivities_batch[group_base + other_ret];

      if (raw_distance == other_raw_distance && reflectivity == other_reflectivity) return true;

      const float other_distance = static_cast<float>(other_raw_distance) * dis_unit;
      if (fabsf(distance - other_distance) < threshold) return true;
    }
  }
  return false;
}

/// @brief Batched kernel for processing an entire scan in one launch.
/// Scan boundaries (FOV and overlap) are determined by the CPU ScanCutter and passed via
/// d_scan_flags (per-entry, per-channel): 0xFF=skip, 1=current scan, 0=next scan.
__global__ void decode_hesai_scan_batch_kernel(
  const uint16_t * __restrict__ d_distances_batch,
  const uint8_t * __restrict__ d_reflectivities_batch, const uint32_t * __restrict__ d_raw_azimuths,
  const uint32_t * __restrict__ d_n_returns, const uint8_t * __restrict__ d_scan_flags,
  const CudaAngleCorrectionData * __restrict__ angle_lut, const CudaDecoderConfig config,
  CudaNebulaPoint * __restrict__ output_points, uint32_t * __restrict__ output_count,
  uint32_t n_azimuths, uint32_t n_packets)
{
  const uint32_t global_tid = blockIdx.x * blockDim.x + threadIdx.x;

  const uint32_t total_work = n_packets * config.n_channels * config.max_returns;
  if (global_tid >= total_work) return;

  const uint32_t packet_id = global_tid / (config.n_channels * config.max_returns);
  const uint32_t channel_id = (global_tid / config.max_returns) % config.n_channels;
  const uint32_t return_id = global_tid % config.max_returns;

  if (return_id >= d_n_returns[packet_id]) return;

  // Read CPU-computed scan flag for this entry/channel
  const uint8_t scan_flag = d_scan_flags[packet_id * config.n_channels + channel_id];
  if (scan_flag == 0xFF) return;  // Not in FOV (determined by CPU ScanCutter)

  const uint32_t data_idx = packet_id * (config.n_channels * config.max_returns) +
                            channel_id * config.max_returns + return_id;

  const uint16_t raw_distance = d_distances_batch[data_idx];
  const uint8_t reflectivity = d_reflectivities_batch[data_idx];

  if (raw_distance == 0) return;

  const float distance = static_cast<float>(raw_distance) * config.dis_unit;

  if (distance < config.min_range || distance > config.max_range) return;
  if (distance < config.sensor_min_range || distance > config.sensor_max_range) return;

  // Dual-return filtering
  const uint32_t n_returns = d_n_returns[packet_id];
  const uint32_t group_base =
    packet_id * (config.n_channels * config.max_returns) + channel_id * config.max_returns;

  if (
    should_filter_duplicate_return(
      d_distances_batch, d_reflectivities_batch, group_base, return_id, n_returns, raw_distance,
      reflectivity, distance, config.dis_unit, config.dual_return_distance_threshold)) {
    return;
  }

  // Compute coordinates using angle LUT
  const uint32_t raw_azimuth = d_raw_azimuths[packet_id];
  const uint32_t azimuth_idx = (raw_azimuth / config.azimuth_scale) % n_azimuths;
  const uint32_t lut_idx = azimuth_idx * config.n_channels + channel_id;
  const CudaAngleCorrectionData angle_data = angle_lut[lut_idx];

  const float xy_distance = distance * angle_data.cos_elevation;
  const float x = xy_distance * angle_data.sin_azimuth;
  const float y = xy_distance * angle_data.cos_azimuth;
  const float z = distance * angle_data.sin_elevation;

  if (global_tid >= config.max_output_points) return;

  CudaNebulaPoint & out_pt = output_points[global_tid];
  out_pt.x = x;
  out_pt.y = y;
  out_pt.z = z;
  out_pt.distance = distance;
  out_pt.azimuth = angle_data.azimuth_rad;
  out_pt.elevation = angle_data.elevation_rad;
  out_pt.intensity = static_cast<float>(reflectivity);
  out_pt.return_type = static_cast<uint8_t>(return_id);
  out_pt.channel = static_cast<uint16_t>(channel_id);
  out_pt.in_current_scan = scan_flag;  // CPU-authoritative: 1=current, 0=next
  out_pt.entry_id = packet_id;

  atomicAdd(output_count, 1);
}

}  // namespace nebula::drivers::cuda

// C-linkage wrapper for batched kernel
extern "C" bool launch_decode_hesai_scan_batch(
  const uint16_t * d_distances_batch, const uint8_t * d_reflectivities_batch,
  const uint32_t * d_raw_azimuths, const uint32_t * d_n_returns, const uint8_t * d_scan_flags,
  const nebula::drivers::cuda::CudaAngleCorrectionData * d_angle_lut,
  const nebula::drivers::cuda::CudaDecoderConfig & config,
  nebula::drivers::cuda::CudaNebulaPoint * d_points, uint32_t * d_count, uint32_t n_azimuths,
  uint32_t n_packets, cudaStream_t stream)
{
  const uint32_t total_work = n_packets * config.n_channels * config.max_returns;
  const uint32_t threads_per_block = 256;
  const uint32_t n_blocks = (total_work + threads_per_block - 1) / threads_per_block;

  dim3 grid(n_blocks);
  dim3 block(threads_per_block);

  nebula::drivers::cuda::decode_hesai_scan_batch_kernel<<<grid, block, 0, stream>>>(
    d_distances_batch, d_reflectivities_batch, d_raw_azimuths, d_n_returns, d_scan_flags,
    d_angle_lut, config, d_points, d_count, n_azimuths, n_packets);

  return cudaGetLastError() == cudaSuccess;
}
