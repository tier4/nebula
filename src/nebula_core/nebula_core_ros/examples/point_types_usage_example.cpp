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

#include "nebula_core_common/nebula_common.hpp"
// # --8<-- [start:include]
#include "nebula_core_common/point_types.hpp"
// # --8<-- [end:include]
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions/pcl_conversions.h>
#include <sys/types.h>

#include <cassert>
#include <cstdint>
#include <memory>

namespace nebula::ros::examples
{

void point_types_usage_example()
{
  // # --8<-- [start:usage]
  auto cloud = std::make_shared<drivers::NebulaPointCloud>();
  cloud->reserve(2);

  nebula::drivers::NebulaPoint point{};
  point.x = 1.0F;
  point.y = 2.0F;
  point.z = 3.0F;
  point.intensity = 10U;
  point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::STRONGEST);
  point.channel = 5U;
  point.azimuth = 0.0F;
  point.elevation = 0.1F;
  point.distance = 3.74F;
  point.time_stamp = 42U;
  cloud->push_back(point);

  const auto cloud_xyzir = nebula::drivers::convert_point_xyzircaedt_to_point_xyzir(cloud);
  assert(cloud_xyzir != nullptr);

  sensor_msgs::msg::PointCloud2 cloud_ros{};
  pcl::toROSMsg(*cloud_xyzir, cloud_ros);

  // my_publisher->publish(cloud_ros);
  // # --8<-- [end:usage]

  (void)cloud;
  (void)cloud_xyzir;
}

}  // namespace nebula::ros::examples
