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

#include "nebula_common/point_types.hpp"

#include <tuple>
#include <vector>

namespace nebula::drivers
{
/// @brief Base class for Robosense LiDAR decoder
class RobosenseScanDecoder
{
public:
  RobosenseScanDecoder(RobosenseScanDecoder && c) = delete;
  RobosenseScanDecoder & operator=(RobosenseScanDecoder && c) = delete;
  RobosenseScanDecoder(const RobosenseScanDecoder & c) = delete;
  RobosenseScanDecoder & operator=(const RobosenseScanDecoder & c) = delete;

  virtual ~RobosenseScanDecoder() = default;
  RobosenseScanDecoder() = default;

  /// @brief Parses RobosensePacket and add its points to the point cloud
  /// @param msop_packet The incoming MsopPacket
  /// @return The last azimuth processed
  virtual int unpack(const std::vector<uint8_t> & msop_packet) = 0;

  /// @brief Indicates whether one full scan is ready
  /// @return Whether a scan is ready
  virtual bool has_scanned() = 0;

  /// @brief Returns the point cloud and timestamp of the last scan
  /// @return A tuple of point cloud and timestamp in nanoseconds
  virtual std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() = 0;
};

}  // namespace nebula::drivers
