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

static constexpr float DISTANCE_THRESHOLD = 0.01;  // m
static constexpr float ANGLE_THRESHOLD = 1e-3;     // deg

void checkPCDs(nebula::drivers::NebulaPointCloudPtr pc, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ref)
{
  EXPECT_EQ(pc->points.size(), pc_ref->points.size());
  auto bound = std::min(pc->points.size(), pc_ref->points.size());
  for (uint32_t i = 0; i < bound; i++) {
    auto p = pc->points[i];
    auto p_ref = pc_ref->points[i];

    auto d_2 = p.x * p.x + p.y * p.y + p.z * p.z;
    auto d_ref_2 = p_ref.x * p_ref.x + p_ref.y * p_ref.y + p_ref.z * p_ref.z;

    auto ele = asin(p.z / sqrt(d_2));
    auto ele_ref = asin(p_ref.z / sqrt(d_ref_2));

    auto azi = atan2(p.x, p.y);
    auto azi_ref = atan2(p_ref.x, p_ref.y);

    // Regularize azimuths to be within 2*PI of each other
    if (azi - azi_ref > M_PI)
      azi -= 2 * M_PI;
    else if (azi_ref - azi > M_PI)
      azi += 2 * M_PI;

    std::cout << "{"
              << "'d':" << sqrt(d_ref_2) << ", 'd_diff': " << sqrt(abs(d_2 - d_ref_2))
              << ", 'a':" << azi_ref << ", 'a_diff': " << abs(azi - azi_ref)
              << ", 'e':" << ele_ref << ", 'e_diff': " << abs(ele - ele_ref) << "}"
              << std::endl;

    EXPECT_NEAR(d_2, d_ref_2, DISTANCE_THRESHOLD * DISTANCE_THRESHOLD);
    EXPECT_NEAR(ele, ele_ref, ANGLE_THRESHOLD);
    EXPECT_NEAR(azi, azi_ref, ANGLE_THRESHOLD);
  }
}

}  // namespace ros
}  // namespace nebula