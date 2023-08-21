#pragma once

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp"
#include "nebula_ros/common/nebula_driver_ros_wrapper_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <gtest/gtest.h>

namespace nebula
{
namespace ros
{

void checkPCDs(nebula::drivers::NebulaPointCloudPtr pc, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ref)
{
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

}  // namespace ros
}  // namespace nebula