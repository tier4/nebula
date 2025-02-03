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
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp>
#include <nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <iostream>

namespace nebula::test
{

inline void check_pcds(
  nebula::drivers::NebulaPointCloudPtr pc, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ref)
{
  ASSERT_GT(pc->points.size(), 0);
  EXPECT_EQ(pc->points.size(), pc_ref->points.size());
  auto bound = std::min(pc->points.size(), pc_ref->points.size());
  for (uint32_t i = 0; i < bound; i++) {
    auto p = pc->points[i];
    auto p_ref = pc_ref->points[i];

    EXPECT_FLOAT_EQ(p.x, p_ref.x);
    EXPECT_FLOAT_EQ(p.y, p_ref.y);
    EXPECT_FLOAT_EQ(p.z, p_ref.z);

    // Prevent thousands of outputs when point clouds do not align
    if (p.x != p_ref.x || p.y != p_ref.y || p.z != p_ref.z) {
      return;
    }
  }
}

inline void check_pcds(
  nebula::drivers::NebulaPointCloudPtr pp1, nebula::drivers::NebulaPointCloudPtr pp2)
{
  EXPECT_EQ(pp1->points.size(), pp2->points.size());
  for (uint32_t i = 0; i < pp1->points.size(); i++) {
    auto p1 = pp1->points[i];
    auto p2 = pp2->points[i];
    EXPECT_FLOAT_EQ(p1.x, p2.x);
    EXPECT_FLOAT_EQ(p1.y, p2.y);
    EXPECT_FLOAT_EQ(p1.z, p2.z);
    EXPECT_FLOAT_EQ(p1.intensity, p2.intensity);
    EXPECT_EQ(p1.channel, p2.channel);
    EXPECT_FLOAT_EQ(p1.azimuth, p2.azimuth);
    EXPECT_EQ(p1.return_type, p2.return_type);
    EXPECT_DOUBLE_EQ(p1.time_stamp, p2.time_stamp);
  }
}

inline void print_pcd(nebula::drivers::NebulaPointCloudPtr pp)
{
  for (auto p : pp->points) {
    std::cout << "(" << p.x << ", " << p.y << "," << p.z << "): " << p.intensity << ", "
              << p.channel << ", " << p.azimuth << ", " << p.return_type << ", " << p.time_stamp
              << std::endl;
  }
}

}  // namespace nebula::test
