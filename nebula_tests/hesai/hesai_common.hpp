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
  auto bound = std::min(pc->points.size(), ref_pc->points.size());
  for (uint32_t i = 0; i < bound; i++) {
    auto p1 = pc->points[i];
    auto p2 = ref_pc->points[i];

    auto azi1 = std::atan2(p1.x, p1.y);
    auto azi2 = std::atan2(p2.x, p2.y);

    std::cout << "<<<" << azi1 << ", " << azi2 << ">>>" << std::endl;

    EXPECT_NEAR(
      p1.x * p1.x + p1.y * p1.y + p1.z * p1.z, p2.x * p2.x + p2.y * p2.y + p2.z * p2.z, .01 * .01);
  }
}

}  // namespace ros
}  // namespace nebula