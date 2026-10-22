// Copyright 2026 TIER IV, Inc. & 8b-is (https://8b.is)
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

#include "nebula_core_decoders/angles.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace nebula::drivers::phoenix_pq
{

/// @brief Canonical Golden Ratio rational convergent p/q = F_19 / F_18 = 4181 / 2584.
inline constexpr int64_t kPhiP = 4181;
inline constexpr int64_t kPhiQ = 2584;

/// @brief Sentinel value marking a dropped LiDAR UDP packet / missing azimuth firing block.
inline constexpr int32_t kAzimuthDropoutSentinel = -1;

/**
 * @brief 5-state Laser Phoenix Interference Lattice on Z_{2^32}.
 *
 * Classifies angular phase offsets or inter-channel laser firing deltas into 5 canonical
 * harmonic states without floating-point trigonometry:
 *   - Bound       (0 deg):   in-phase lock        (|dtheta| < 22.5 deg)
 *   - Related     (45 deg):  harmonic coupling    (22.5 deg <= |dtheta| < 67.5 deg)
 *   - Independent (90 deg):  orthogonal / quad    (67.5 deg <= |dtheta| < 112.5 deg)
 *   - Contrasting (135 deg): anti-harmonic        (112.5 deg <= |dtheta| < 157.5 deg)
 *   - Conflicting (180 deg): phase opposition     (|dtheta| >= 157.5 deg)
 */
enum class PhoenixInterferenceState : uint8_t {
  Bound = 0,
  Related = 1,
  Independent = 2,
  Contrasting = 3,
  Conflicting = 4
};

/// @brief Canonical 5-state lattice anchors on Z_{2^32} (0, 45, 90, 135, 180 deg).
inline constexpr std::array<uint32_t, 5> kPhoenixLatticeAnchorsQ32 = {
  0x00000000U,  //   0.0 deg
  0x20000000U,  //  45.0 deg
  0x40000000U,  //  90.0 deg
  0x60000000U,  // 135.0 deg
  0x80000000U   // 180.0 deg
};

/**
 * @brief Maps a discrete LiDAR encoder angle in [0, max_angle) (e.g. CentiDegrees = 36000 or
 * MilliDegrees = 360000) to the 32-bit cyclic group Z_{2^32} using exact 64-bit integer p/q math.
 */
template <typename IntT>
[[nodiscard]] inline uint32_t azimuth_to_phase_q32(IntT azimuth, IntT max_angle) noexcept
{
  const int64_t norm = static_cast<int64_t>(normalize_angle(azimuth, max_angle));
  const int64_t m = static_cast<int64_t>(max_angle);
  if (m <= 0) {
    return 0U;
  }
  const uint64_t num = (static_cast<uint64_t>(norm) << 32) + static_cast<uint64_t>(m / 2);
  return static_cast<uint32_t>(num / static_cast<uint64_t>(m));
}

/**
 * @brief Maps a 32-bit cyclic phase on Z_{2^32} back to a discrete LiDAR encoder angle in
 * [0, max_angle) using exact 64-bit integer p/q rounding.
 */
template <typename IntT>
[[nodiscard]] inline IntT phase_q32_to_azimuth(uint32_t phase_q32, IntT max_angle) noexcept
{
  const uint64_t m = static_cast<uint64_t>(max_angle);
  if (m == 0) {
    return static_cast<IntT>(0);
  }
  const uint64_t prod = static_cast<uint64_t>(phase_q32) * m + (1ULL << 31);
  const uint64_t az = (prod >> 32) % m;
  return static_cast<IntT>(az);
}

/**
 * @brief Classifies a signed modular phase offset on Z_{2^32} into the 5-state Laser Phoenix
 * Interference Lattice.
 */
[[nodiscard]] inline PhoenixInterferenceState classify_phoenix_interference(
  int32_t signed_delta_q32) noexcept
{
  const uint32_t abs_delta =
    (signed_delta_q32 < 0)
      ? static_cast<uint32_t>(-static_cast<int64_t>(signed_delta_q32))
      : static_cast<uint32_t>(signed_delta_q32);

  if (abs_delta < 0x10000000U) {
    return PhoenixInterferenceState::Bound;
  }
  if (abs_delta < 0x30000000U) {
    return PhoenixInterferenceState::Related;
  }
  if (abs_delta < 0x50000000U) {
    return PhoenixInterferenceState::Independent;
  }
  if (abs_delta < 0x70000000U) {
    return PhoenixInterferenceState::Contrasting;
  }
  return PhoenixInterferenceState::Conflicting;
}

/**
 * @brief Snaps a 32-bit cyclic phase to the nearest 8-fold Laser Phoenix Lattice harmonic anchor
 * (multiples of 45 deg = 0x20000000 on Z_{2^32}).
 */
[[nodiscard]] inline uint32_t snap_phase_to_phoenix_lattice(uint32_t phase_q32) noexcept
{
  return (phase_q32 + 0x10000000U) & 0xE0000000U;
}

/**
 * @brief Zero-float Laser Phoenix Azimuth Phase-Locked Loop (PLL) with Marine Encoder-Jitter
 * Salience Gate for spinning LiDAR optical encoders.
 *
 * Smooths high-frequency optical encoder jitter and prevents backward azimuth glitches across
 * ScanCutter cut boundaries while tracking motor angular velocity in Z_{2^32}.
 */
class LaserPhoenixAzimuthPll
{
public:
  struct StepResult
  {
    int32_t locked_azimuth;
    uint32_t locked_phase_q32;
    int32_t angular_velocity_q32;
    uint16_t coherence_q8;  // [0, 256] where 256 == 1.0
    PhoenixInterferenceState interference_state;
    bool jitter_clipped;
  };

  explicit LaserPhoenixAzimuthPll(int32_t max_angle = 36000) noexcept : max_angle_(max_angle) {}

  void reset() noexcept
  {
    initialized_ = false;
    has_velocity_ = false;
    phase_q32_ = 0U;
    velocity_q32_ = 0;
    prev_raw_phase_q32_ = 0U;
    prev_raw_vel_q32_ = 0;
    jitter_history_.fill(0U);
    jitter_idx_ = 0;
    jitter_count_ = 0;
  }

  [[nodiscard]] StepResult step(int32_t raw_azimuth) noexcept
  {
    const uint32_t raw_phase = azimuth_to_phase_q32(raw_azimuth, max_angle_);

    if (!initialized_) {
      initialized_ = true;
      phase_q32_ = raw_phase;
      prev_raw_phase_q32_ = raw_phase;
      return StepResult{
        normalize_angle(raw_azimuth, max_angle_),
        phase_q32_,
        0,
        256,
        PhoenixInterferenceState::Bound,
        false};
    }

    // Signed modular velocity on Z_{2^32}
    const int32_t raw_vel = static_cast<int32_t>(raw_phase - prev_raw_phase_q32_);
    prev_raw_phase_q32_ = raw_phase;

    if (!has_velocity_) {
      has_velocity_ = true;
      velocity_q32_ = raw_vel;
      prev_raw_vel_q32_ = raw_vel;
      phase_q32_ = raw_phase;
      return StepResult{
        phase_q32_to_azimuth(phase_q32_, max_angle_),
        phase_q32_,
        velocity_q32_,
        256,
        PhoenixInterferenceState::Bound,
        false};
    }

    // Second-order phase-acceleration jitter |raw_vel - prev_raw_vel|
    const int64_t accel_diff =
      static_cast<int64_t>(raw_vel) - static_cast<int64_t>(prev_raw_vel_q32_);
    prev_raw_vel_q32_ = raw_vel;
    const uint32_t abs_accel =
      static_cast<uint32_t>(accel_diff < 0 ? -accel_diff : accel_diff);

    jitter_history_[jitter_idx_] = abs_accel;
    jitter_idx_ = (jitter_idx_ + 1) & 7U;
    if (jitter_count_ < 8U) {
      ++jitter_count_;
    }

    uint64_t jitter_sum = 0;
    for (size_t i = 0; i < jitter_count_; ++i) {
      jitter_sum += jitter_history_[i];
    }
    const uint64_t mean_jitter = jitter_sum / jitter_count_;

    // Reference jitter scale: ~0.5 deg in Z_{2^32} = 2^32 / 720 ~= 5,965,232
    constexpr uint64_t kJitterRefQ32 = 5965232ULL;
    const uint64_t penalty = (mean_jitter * 256ULL) / kJitterRefQ32;
    const uint16_t coherence_q8 =
      (penalty >= 256ULL) ? 0U : static_cast<uint16_t>(256ULL - penalty);

    // Marine Salience Gate: if coherence < 0.35 (90 / 256), halve the PLL innovation gain
    const bool jitter_clipped = (coherence_q8 < 90U);

    // Predict next phase using flywheel angular velocity
    const uint32_t predicted_phase =
      phase_q32_ + static_cast<uint32_t>(velocity_q32_);
    const int32_t phase_error = static_cast<int32_t>(raw_phase - predicted_phase);
    const PhoenixInterferenceState state = classify_phoenix_interference(phase_error);

    // Golden-ratio inverse gain alpha = 1/phi = 2584 / 4181 (~0.618), halved under Marine gate
    const int64_t alpha_num = jitter_clipped ? (kPhiQ / 2) : kPhiQ;
    const int64_t alpha_den = kPhiP;

    const int32_t phase_correction =
      static_cast<int32_t>((static_cast<int64_t>(phase_error) * alpha_num) / alpha_den);
    phase_q32_ = predicted_phase + static_cast<uint32_t>(phase_correction);

    // Update flywheel velocity with beta = 1/phi^2 = (4181 - 2584)/4181 = 1597 / 4181 (~0.382)
    constexpr int64_t kBetaNum = kPhiP - kPhiQ;  // 1597
    const int64_t beta_num = jitter_clipped ? (kBetaNum / 4) : kBetaNum;
    const int64_t vel_error =
      static_cast<int64_t>(raw_vel) - static_cast<int64_t>(velocity_q32_);
    velocity_q32_ = static_cast<int32_t>(
      static_cast<int64_t>(velocity_q32_) + (vel_error * beta_num) / alpha_den);

    return StepResult{
      phase_q32_to_azimuth(phase_q32_, max_angle_),
      phase_q32_,
      velocity_q32_,
      coherence_q8,
      state,
      jitter_clipped};
  }

private:
  int32_t max_angle_{36000};
  bool initialized_{false};
  bool has_velocity_{false};
  uint32_t phase_q32_{0U};
  int32_t velocity_q32_{0};
  uint32_t prev_raw_phase_q32_{0U};
  int32_t prev_raw_vel_q32_{0};
  std::array<uint32_t, 8> jitter_history_{};
  size_t jitter_idx_{0};
  size_t jitter_count_{0};
};

/**
 * @brief Exact integer p/q Catmull-Rom cubic Hermite evaluation for local step k in [0, span].
 *
 * Evaluates:
 *   q(t) = 0.5 * ((2*p1) + (-p0 + p2)*t + (2*p0 - 5*p1 + 4*p2 - p3)*t^2 + (-p0 + 3*p1 - 3*p2 + p3)*t^3)
 * with t = k / span using pure 64-bit integer arithmetic.
 */
[[nodiscard]] inline int64_t eval_catmull_rom_pq(
  int64_t p0, int64_t p1, int64_t p2, int64_t p3, int64_t k, int64_t span) noexcept
{
  if (span <= 0) {
    return p1;
  }
  const int64_t s2 = span * span;
  const int64_t s3 = s2 * span;
  const int64_t k2 = k * k;
  const int64_t k3 = k2 * k;

  const int64_t a0 = 2 * p1;
  const int64_t a1 = -p0 + p2;
  const int64_t a2 = 2 * p0 - 5 * p1 + 4 * p2 - p3;
  const int64_t a3 = -p0 + 3 * p1 - 3 * p2 + p3;

  const int64_t num = a0 * s3 + a1 * k * s2 + a2 * k2 * span + a3 * k3;
  const int64_t den = 2 * s3;
  if (num >= 0) {
    return (num + den / 2) / den;
  }
  return (num - den / 2) / den;
}

/**
 * @brief Restores dropped LiDAR packet azimuths (marked by `kAzimuthDropoutSentinel = -1`)
 * using circular Z_{2^32} phase unwrapping and non-uniform rational p/q Catmull-Rom cubic Hermite
 * interpolation.
 *
 * Works seamlessly across the 360 deg -> 0 deg encoder wrap-around boundary without branches
 * or floating-point operations.
 */
inline size_t restore_dropped_azimuths_catmull_rom_pq(
  std::vector<int32_t> & azimuths, int32_t max_angle = 36000)
{
  const size_t n = azimuths.size();
  std::vector<size_t> valid_indices;
  valid_indices.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    if (azimuths[i] != kAzimuthDropoutSentinel) {
      azimuths[i] = normalize_angle(azimuths[i], max_angle);
      valid_indices.push_back(i);
    }
  }

  if (valid_indices.empty() || valid_indices.size() == n) {
    return 0;
  }

  // Convert valid anchor azimuths to unwrapped cumulative phase in Z_{2^32} (stored in int64_t)
  std::vector<int64_t> unwrapped_q32(valid_indices.size(), 0);
  uint32_t prev_q32 = azimuth_to_phase_q32(azimuths[valid_indices[0]], max_angle);
  unwrapped_q32[0] = static_cast<int64_t>(prev_q32);

  for (size_t k = 1; k < valid_indices.size(); ++k) {
    const uint32_t curr_q32 = azimuth_to_phase_q32(azimuths[valid_indices[k]], max_angle);
    // Hardware signed 32-bit modular difference automatically unwraps 360 deg -> 0 deg!
    const int32_t delta_q32 = static_cast<int32_t>(curr_q32 - prev_q32);
    unwrapped_q32[k] = unwrapped_q32[k - 1] + static_cast<int64_t>(delta_q32);
    prev_q32 = curr_q32;
  }

  size_t restored_count = 0;
  size_t anchor_pos = 0;

  for (size_t i = 0; i < n; ++i) {
    if (azimuths[i] != kAzimuthDropoutSentinel) {
      while (anchor_pos + 1 < valid_indices.size() && valid_indices[anchor_pos + 1] <= i) {
        ++anchor_pos;
      }
      continue;
    }

    if (i < valid_indices.front()) {
      azimuths[i] = azimuths[valid_indices.front()];
      ++restored_count;
      continue;
    }
    if (i > valid_indices.back()) {
      azimuths[i] = azimuths[valid_indices.back()];
      ++restored_count;
      continue;
    }

    while (anchor_pos + 1 < valid_indices.size() && valid_indices[anchor_pos + 1] < i) {
      ++anchor_pos;
    }

    const size_t k1 = anchor_pos;
    const size_t k2 = (anchor_pos + 1 < valid_indices.size()) ? (anchor_pos + 1) : anchor_pos;
    const size_t k0 = (k1 > 0) ? (k1 - 1) : k1;
    const size_t k3 = (k2 + 1 < valid_indices.size()) ? (k2 + 1) : k2;

    const int64_t idx0 = static_cast<int64_t>(valid_indices[k0]);
    const int64_t idx1 = static_cast<int64_t>(valid_indices[k1]);
    const int64_t idx2 = static_cast<int64_t>(valid_indices[k2]);
    const int64_t idx3 = static_cast<int64_t>(valid_indices[k3]);

    const int64_t span = idx2 - idx1;
    const int64_t step_k = static_cast<int64_t>(i) - idx1;

    const int64_t y0 = unwrapped_q32[k0];
    const int64_t p1 = unwrapped_q32[k1];
    const int64_t p2 = unwrapped_q32[k2];
    const int64_t y3 = unwrapped_q32[k3];

    // Non-uniform Bessel-Overhauser chord-normalized tangents mapped to virtual control points
    const int64_t m1 = (idx2 > idx0) ? (((p2 - y0) * span) / (idx2 - idx0)) : (p2 - p1);
    const int64_t m2 = (idx3 > idx1) ? (((y3 - p1) * span) / (idx3 - idx1)) : (p2 - p1);
    const int64_t p0 = p2 - 2 * m1;
    const int64_t p3 = p1 + 2 * m2;

    const int64_t interp_q32 = eval_catmull_rom_pq(p0, p1, p2, p3, step_k, span);
    const uint32_t wrapped_q32 = static_cast<uint32_t>(static_cast<uint64_t>(interp_q32));
    azimuths[i] = phase_q32_to_azimuth(wrapped_q32, max_angle);
    ++restored_count;
  }

  return restored_count;
}

}  // namespace nebula::drivers::phoenix_pq
