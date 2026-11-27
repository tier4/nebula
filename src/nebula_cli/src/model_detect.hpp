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

#include <cstdint>
#include <string>
#include <vector>

namespace nebula::cli
{

/// @brief Guess the sensor model (as accepted by `--model`) from one scan packet.
/// Hesai: by packet size. Velodyne: by the product ID byte (all models use 1206-byte packets).
/// Robosense cannot be told apart (same packet layout and size). Throws std::runtime_error if the
/// model cannot be determined.
std::string detect_model(const std::string & vendor, const std::vector<uint8_t> & packet);

/// @brief Read the return mode (as accepted by `--return-mode`) from one scan packet of a known
/// model. Supported for Hesai (packet tail) and Velodyne (factory byte). Throws
/// std::runtime_error if it cannot be determined.
std::string detect_return_mode(
  const std::string & vendor, const std::string & model, const std::vector<uint8_t> & packet);

}  // namespace nebula::cli
