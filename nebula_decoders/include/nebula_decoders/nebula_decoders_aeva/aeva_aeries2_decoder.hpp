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

#include <nebula_common/aeva/packet_types.hpp>
#include <nebula_common/aeva/point_types.hpp>
#include <nebula_common/nebula_common.hpp>

#include <pcl/point_cloud.h>
#include <sys/types.h>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

namespace nebula::drivers
{

class AevaAeries2Decoder
{
public:
  using AevaPoint = aeva::PointXYZVIRCAEDT;
  using AevaPointCloud = pcl::PointCloud<AevaPoint>;
  using AevaPointCloudUniquePtr = std::unique_ptr<AevaPointCloud>;

  using callback_t = std::function<void(AevaPointCloudUniquePtr, uint64_t)>;

  AevaAeries2Decoder() : cloud_state_({std::make_unique<AevaPointCloud>(), 0}) {}

  void process_pointcloud_message(const aeva::PointCloudMessage & message);

  void register_point_cloud_callback(callback_t callback);

  void on_parameter_change(ReturnMode return_mode);

private:
  struct DecoderState
  {
    int32_t new_frame_index;
    uint64_t time_per_marker_point_ns;
    size_t line_index;
    size_t point_index;
    uint64_t absolute_time_ns;
  };

  struct PointcloudState
  {
    std::unique_ptr<AevaPointCloud> cloud;
    uint64_t timestamp;
  };

  [[nodiscard]] ReturnType get_return_type(uint32_t peak_id) const;

  callback_t callback_;
  std::atomic<ReturnMode> return_mode_{ReturnMode::UNKNOWN};
  PointcloudState cloud_state_{};
};
}  // namespace nebula::drivers
