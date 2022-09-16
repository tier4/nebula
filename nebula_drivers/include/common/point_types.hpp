#pragma once

//Galactic
//#include <pcl-1.10/pcl/point_cloud.h>
//#include <pcl-1.10/pcl/point_types.h>
//Humble
//#include <pcl-1.12/pcl/point_cloud.h>
//#include <pcl-1.12/pcl/point_types.h>
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

using PointXYZIRADTPtr = std::shared_ptr<PointXYZIRADT>;
using PointCloudXYZIRADT = pcl::PointCloud<PointXYZIRADT>;
using PointCloudXYZIRADTPtr = pcl::PointCloud<PointXYZIRADT>::Ptr;

class DataContainerBase
{
public:
  virtual void addPoint(
    const float & x, const float & y, const float & z,
    const uint8_t & return_type, const uint16_t & ring,
    const uint16_t & azimuth, const float & distance, const float & intensity,
    const double & time_stamp) = 0;
};

}  // namespace drivers
}  // namespace nebula

POINT_CLOUD_REGISTER_POINT_STRUCT(
  nebula::drivers::PointXYZIR,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  nebula::drivers::PointXYZIRADT,
  (float, x,
   x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(float, azimuth, azimuth)(float, distance, distance)(std::uint8_t, return_type, return_type)(double, time_stamp, time_stamp))