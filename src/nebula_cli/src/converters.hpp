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

#include "params.hpp"

#include <nebula_core_common/point_types.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace nebula::cli
{

/// @brief One raw sensor packet taken out of a bag message
struct Packet
{
  int64_t stamp_ns;
  std::vector<uint8_t> data;
};

/// @brief Vendor-independent view of a packet decoder: packets in, point clouds out via callback
class Converter
{
public:
  using CloudCallback = std::function<void(const drivers::NebulaPointCloudPtr &, double)>;

  virtual ~Converter() = default;

  /// @brief Feed a scan (point data) packet. Calls the cloud callback when a scan completes.
  virtual void feed_scan(const Packet & packet) = 0;

  /// @brief Feed a sensor info packet (only used by Robosense, which gets calibration from it)
  virtual void feed_info(const Packet & /* packet */) {}

  /// @brief Number of packets that could not be decoded
  virtual uint64_t dropped_packets() const = 0;
};

/// @brief Create the converter for a vendor ("hesai", "velodyne" or "robosense").
/// Throws std::runtime_error on invalid configuration.
std::unique_ptr<Converter> make_converter(
  const std::string & vendor, const Params & params, Converter::CloudCallback callback);

}  // namespace nebula::cli
