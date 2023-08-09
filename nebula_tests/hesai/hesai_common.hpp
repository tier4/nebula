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

void checkPCDs(nebula::drivers::NebulaPointCloudPtr pc, pcl::PointCloud<pcl::PointXYZ>::Ptr ref_pc)
{
  EXPECT_EQ(pc->points.size(), ref_pc->points.size());
  for (uint32_t i = 0; i < pc->points.size(); i++) {
    auto p = pc->points[i];
    auto p_ref = ref_pc->points[i];

    EXPECT_FLOAT_EQ(p.x, p_ref.x);
    EXPECT_FLOAT_EQ(p.y, p_ref.y);
    EXPECT_FLOAT_EQ(p.z, p_ref.z);
  }
}

}  // namespace ros
}  // namespace nebula