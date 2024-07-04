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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>

namespace nebula::drivers
{

class AevaAries2Decoder
{
public:
  using AevaPoint = aeva::PointXYZVIRCAEDT;
  using AevaPointCloud = pcl::PointCloud<AevaPoint>;
  using AevaPointCloudUniquePtr = std::unique_ptr<AevaPointCloud>;

  using callback_t = std::function<void(AevaPointCloudUniquePtr, uint64_t)>;

  AevaAries2Decoder() : cloud_state_({std::make_unique<AevaPointCloud>(), 0}) {}

  void processPointcloudMessage(const aeva::PointCloudMessage & message);

  void registerPointCloudCallback(callback_t callback);

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

  callback_t callback_;
  PointcloudState cloud_state_{};

  std::mutex mtx_callback_;
};
}  // namespace nebula::drivers
