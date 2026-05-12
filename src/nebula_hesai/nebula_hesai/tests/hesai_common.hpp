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

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <nebula_hesai_decoders/decoders/hesai_scan_decoder.hpp>
#include <nebula_hesai_decoders/hesai_driver.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>

namespace nebula::test
{

inline void check_pcds(
  const nebula::drivers::NebulaPointCloudPtr & pc,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pc_ref)
{
  ASSERT_EQ(pc->points.size(), pc_ref->points.size()) << "The point clouds are not the same size.";
  auto bound = std::min(pc->points.size(), pc_ref->points.size());
  for (uint32_t i = 0; i < bound; i++) {
    auto p = pc->points[i];
    auto p_ref = pc_ref->points[i];

    EXPECT_FLOAT_EQ(p.x, p_ref.x);
    EXPECT_FLOAT_EQ(p.y, p_ref.y);
    EXPECT_FLOAT_EQ(p.z, p_ref.z);

    // Prevent thousands of outputs when point clouds do not align
    ASSERT_TRUE(p.x == p_ref.x && p.y == p_ref.y && p.z == p_ref.z)
      << "The point clouds are the same size, but at least one point is geometrically different.";
  }
}

}  // namespace nebula::test
