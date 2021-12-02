#pragma once

#include "hesai_common.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace nebula
{
namespace drivers
{
struct PointXYZIR
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointXYZIRADT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float azimuth;
  float distance;
  uint8_t return_type;
  double time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

using PclPointCloudXYZIRAD = pcl::PointCloud<PointXYZIRADT>;
using PclPointCloudXYZIRADTPtr = pcl::PointCloud<PointXYZIRADT>::Ptr;

}  // namespace drivers
}  // namespace nebula

POINT_CLOUD_REGISTER_POINT_STRUCT(
  nebula::drivers::PointXYZIR,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  nebula::drivers::PointXYZIRADT,
  (float, x,
   x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(float, azimuth, azimuth)(float, distance, distance)(std::uint8_t, return_type, return_type)(double, time_stamp, time_stamp))