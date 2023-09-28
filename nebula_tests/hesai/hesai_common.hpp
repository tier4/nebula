#pragma once

#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_scan_decoder.hpp"
#include "nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp"
#include "nebula_ros/common/nebula_driver_ros_wrapper_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include <gtest/gtest.h>
#include <time.h>

#include <algorithm>

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

void checkPCDs(nebula::drivers::NebulaPointCloudPtr pp1, nebula::drivers::NebulaPointCloudPtr pp2)
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

void printPCD(nebula::drivers::NebulaPointCloudPtr pp)
{
  for (auto p : pp->points) {
    std::cout << "(" << p.x << ", " << p.y << "," << p.z << "): " << p.intensity << ", "
              << p.channel << ", " << p.azimuth << ", " << p.return_type << ", " << p.time_stamp
              << std::endl;
  }
}

void checkTimestamp(
  pandar_msgs::msg::PandarScan & raw_scan, nebula::drivers::HesaiScanDecoder & decoder)
{
  // first: self-check
  putenv("TZ=GMT");
  tzset();
  auto gmt = timezone;
  putenv("TZ=JST");
  tzset();
  auto jst = timezone;

  EXPECT_NE(gmt, jst);

  for (auto packet : raw_scan.packets) {
    decoder.unpack(packet);
  }
}

}  // namespace ros
}  // namespace nebula