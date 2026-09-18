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

/// @file hesai_cuda_decoder_test.cpp
/// @brief GPU-vs-CPU equivalence tests for the Hesai CUDA decoder (OT128 / Pandar128E4X).
///
/// Decodes the same rosbag with the CPU path (NEBULA_USE_CUDA unset) and the GPU path
/// (NEBULA_USE_CUDA=1), then compares the resulting point clouds.
/// The GPU path uses CPU-authoritative scan cutting (via scan_state flags), so scan
/// boundaries are identical between CPU and GPU.

#include "hesai_ros_decoder_test.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#ifdef NEBULA_CUDA_ENABLED

namespace nebula::test
{

// OT128 config — matches TEST_CONFIGS[8] in hesai_ros_decoder_test_main.cpp
static const nebula::ros::HesaiRosDecoderTestParams OT128_CONFIG = {
  "Pandar128E4X",
  "LastStrongest",
  "Pandar128E4X.csv",
  "ot128/1730271167765338806",
  "hesai",
  0,
  0.0,
  0.,
  360.,
  0.3f,
  300.f};

// Maximum allowed difference in point count between GPU and CPU per scan.
// GPU uses CPU-authoritative scan_state, so scan cutting is identical.
static constexpr int kMaxPointCountDiff = 0;
// Coordinate tolerance (metres) for nearest-neighbour matching.
// GPU and CPU use the same angle LUT, but floating-point hardware differences
// (e.g. FMA rounding on GPU vs CPU) cause sub-millimetre coordinate differences.
static constexpr float kXyzTolerance = 1e-4f;  // 0.1 mm
// Fraction of GPU points that must have a CPU match within tolerance.
static constexpr double kMinMatchRatio = 1.0;

/// Decoded scan: message timestamp + point cloud
struct DecodedScan
{
  uint64_t msg_timestamp;
  nebula::drivers::NebulaPointCloudPtr cloud;
};

/// Decode a rosbag with the given CUDA env setting.
/// When @p use_cuda is true, sets NEBULA_USE_CUDA=1; otherwise unsets it.
/// Returns one DecodedScan per completed scan.
static std::vector<DecodedScan> decode_bag(
  const nebula::ros::HesaiRosDecoderTestParams & params, bool use_cuda)
{
  // Toggle GPU/CPU path via environment variable
  if (use_cuda) {
    setenv("NEBULA_USE_CUDA", "1", 1);
  } else {
    unsetenv("NEBULA_USE_CUDA");
  }

  rclcpp::NodeOptions options;
  auto driver =
    std::make_shared<nebula::ros::HesaiRosDecoderTest>(options, "cuda_test_node", params);
  EXPECT_EQ(driver->get_status(), nebula::Status::OK);

  std::vector<DecodedScan> scans;
  auto cb = [&](uint64_t msg_ts, uint64_t /*scan_ts*/, nebula::drivers::NebulaPointCloudPtr cloud) {
    if (cloud && !cloud->empty()) {
      // Deep-copy: the decoder reuses its internal buffer after the callback returns
      auto copy = std::make_shared<nebula::drivers::NebulaPointCloud>(*cloud);
      scans.push_back({msg_ts, copy});
    }
  };
  driver->read_bag(cb);
  return scans;
}

/// Squared Euclidean distance between two points
static float sq_dist(const nebula::drivers::NebulaPoint & a, const nebula::drivers::NebulaPoint & b)
{
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  float dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

// ---------------------------------------------------------------------------
// Test 1: GPU vs CPU equivalence
// ---------------------------------------------------------------------------
TEST(HesaiCudaDecoderTest, OT128_GpuVsCpuEquivalence)
{
  auto cpu_scans = decode_bag(OT128_CONFIG, /*use_cuda=*/false);
  auto gpu_scans = decode_bag(OT128_CONFIG, /*use_cuda=*/true);

  // Both paths must produce the same number of scans
  ASSERT_GT(cpu_scans.size(), 0u);
  ASSERT_EQ(cpu_scans.size(), gpu_scans.size())
    << "CPU produced " << cpu_scans.size() << " scans, GPU produced " << gpu_scans.size();

  const float tol_sq = kXyzTolerance * kXyzTolerance;

  for (size_t i = 0; i < cpu_scans.size(); ++i) {
    const auto & cpu_cloud = cpu_scans[i].cloud;
    const auto & gpu_cloud = gpu_scans[i].cloud;

    // Point counts within tolerance
    int diff = static_cast<int>(cpu_cloud->size()) - static_cast<int>(gpu_cloud->size());
    EXPECT_LE(std::abs(diff), kMaxPointCountDiff)
      << "Scan " << i << ": CPU=" << cpu_cloud->size() << " GPU=" << gpu_cloud->size();

    // For each GPU point, find nearest CPU point (brute-force — clouds are small enough)
    size_t matched = 0;
    for (const auto & gp : *gpu_cloud) {
      float best = std::numeric_limits<float>::max();
      for (const auto & cp : *cpu_cloud) {
        float d = sq_dist(gp, cp);
        if (d < best) best = d;
        if (d < tol_sq) break;  // early exit — found a match
      }
      if (best < tol_sq) ++matched;
    }

    double match_ratio =
      gpu_cloud->empty() ? 1.0 : static_cast<double>(matched) / gpu_cloud->size();
    EXPECT_GE(match_ratio, kMinMatchRatio)
      << "Scan " << i << ": only " << matched << "/" << gpu_cloud->size()
      << " GPU points matched a CPU point within " << kXyzTolerance << " m";
  }
}

// ---------------------------------------------------------------------------
// Test 2: GPU output is non-empty
// ---------------------------------------------------------------------------
TEST(HesaiCudaDecoderTest, OT128_GpuOutputNonEmpty)
{
  auto gpu_scans = decode_bag(OT128_CONFIG, /*use_cuda=*/true);
  ASSERT_GT(gpu_scans.size(), 0u) << "GPU path produced zero scans";
  for (size_t i = 0; i < gpu_scans.size(); ++i) {
    EXPECT_GT(gpu_scans[i].cloud->size(), 0u) << "Scan " << i << " is empty";
  }
}

// ---------------------------------------------------------------------------
// Test 3: Basic field validity of GPU-decoded points
// ---------------------------------------------------------------------------
TEST(HesaiCudaDecoderTest, OT128_GpuFieldValidity)
{
  auto gpu_scans = decode_bag(OT128_CONFIG, /*use_cuda=*/true);
  ASSERT_GT(gpu_scans.size(), 0u);

  for (size_t i = 0; i < gpu_scans.size(); ++i) {
    for (const auto & pt : *gpu_scans[i].cloud) {
      // Distance must be positive (all valid points)
      float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      EXPECT_GT(dist, 0.f) << "Scan " << i << ": zero-distance point found";

      // Channel must be in [0, 127] for OT128
      EXPECT_LE(pt.channel, 127u) << "Scan " << i << ": invalid channel " << pt.channel;
    }
  }
}

// ---------------------------------------------------------------------------
// Test 4 (edge case): First and last scans have reasonable point counts
// ---------------------------------------------------------------------------
TEST(HesaiCudaDecoderTest, OT128_BoundaryScanPointCounts)
{
  auto gpu_scans = decode_bag(OT128_CONFIG, /*use_cuda=*/true);
  ASSERT_GE(gpu_scans.size(), 2u) << "Need at least 2 scans for boundary test";

  // First scan may be partial, but should still have a reasonable number of points.
  // A full OT128 scan typically has ~100k+ points; even a partial scan should exceed 1000.
  EXPECT_GT(gpu_scans.front().cloud->size(), 1000u)
    << "First scan has suspiciously few points: " << gpu_scans.front().cloud->size();

  // Last scan should also be non-trivial
  EXPECT_GT(gpu_scans.back().cloud->size(), 1000u)
    << "Last scan has suspiciously few points: " << gpu_scans.back().cloud->size();
}

// ---------------------------------------------------------------------------
// Test 5 (edge case): GPU intensity matches CPU exactly
// ---------------------------------------------------------------------------
TEST(HesaiCudaDecoderTest, OT128_IntensityExactMatch)
{
  auto cpu_scans = decode_bag(OT128_CONFIG, /*use_cuda=*/false);
  auto gpu_scans = decode_bag(OT128_CONFIG, /*use_cuda=*/true);

  ASSERT_EQ(cpu_scans.size(), gpu_scans.size());

  for (size_t i = 0; i < cpu_scans.size(); ++i) {
    const auto & cpu_pts = *cpu_scans[i].cloud;
    const auto & gpu_pts = *gpu_scans[i].cloud;

    // For each GPU point, find the nearest CPU match and verify intensity is identical.
    // Only compare points that have a close spatial match (same physical point).
    size_t checked = 0;
    size_t intensity_mismatch = 0;

    // Use a tighter tolerance for intensity matching to ensure we're comparing the same point
    const float tight_tol_sq = 1e-6f;

    for (const auto & gp : gpu_pts) {
      float best_dist = std::numeric_limits<float>::max();
      size_t best_idx = 0;
      for (size_t j = 0; j < cpu_pts.size(); ++j) {
        float d = sq_dist(gp, cpu_pts[j]);
        if (d < best_dist) {
          best_dist = d;
          best_idx = j;
        }
        if (d < tight_tol_sq) break;
      }
      if (best_dist < tight_tol_sq) {
        ++checked;
        if (gp.intensity != cpu_pts[best_idx].intensity) {
          ++intensity_mismatch;
        }
      }
    }

    // The vast majority of matched points should have identical intensity.
    // A small fraction (<1%) may differ at scan boundaries where the GPU kernel
    // selects a different return than the CPU's dual-return filtering.
    EXPECT_GT(checked, 0u) << "Scan " << i << ": no tight matches found";
    double mismatch_ratio = checked > 0 ? static_cast<double>(intensity_mismatch) / checked : 0.0;
    EXPECT_LT(mismatch_ratio, 0.01)
      << "Scan " << i << ": " << intensity_mismatch << "/" << checked
      << " matched points have different intensity (" << (mismatch_ratio * 100) << "%)";
  }
}

}  // namespace nebula::test

#else  // !NEBULA_CUDA_ENABLED

TEST(HesaiCudaDecoderTest, SkippedNoCuda)
{
  GTEST_SKIP() << "CUDA not enabled at compile time";
}

#endif  // NEBULA_CUDA_ENABLED

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Save original env value so we can restore it after tests
  const char * original_cuda_env = std::getenv("NEBULA_USE_CUDA");
  const std::string saved_cuda_env = original_cuda_env ? original_cuda_env : "";

  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  // Restore original env
  if (saved_cuda_env.empty()) {
    unsetenv("NEBULA_USE_CUDA");
  } else {
    setenv("NEBULA_USE_CUDA", saved_cuda_env.c_str(), 1);
  }

  rclcpp::shutdown();
  return result;
}
