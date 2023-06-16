#ifndef NEBULA_POINT_TYPES_H
#define NEBULA_POINT_TYPES_H

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

struct PointXYZICATR
{
  PCL_ADD_POINT4D;
  std::uint8_t intensity;
  std::uint16_t channel;
  float azimuth;
  std::uint32_t time_stamp;
  std::uint8_t return_type;
};
/**
 * This point type is not using PCL_ADD_POINT4D to avoid the addition of a 32-bit dummy word.
 * The fields are ordered to meet the SSE alignment.
 */
struct PointXYZIRCAEDT
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
  float azimuth;
  float elevation;
  float distance;
  std::uint32_t time_stamp;
};

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

using NebulaPoint = PointXYZIRCAEDT;
using NebulaPointPtr = std::shared_ptr<NebulaPoint>;
using NebulaPointCloud = pcl::PointCloud<NebulaPoint>;
using NebulaPointCloudPtr = pcl::PointCloud<NebulaPoint>::Ptr;

}  // namespace drivers
}  // namespace nebula

POINT_CLOUD_REGISTER_POINT_STRUCT(nebula::drivers::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t,
                                                                                                       ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(nebula::drivers::PointXYZIRADT,
                                  (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (std::uint16_t, ring, ring)
                                    (float, azimuth, azimuth)
                                    (float, distance, distance)
                                    (std::uint8_t, return_type, return_type)
                                    (double, time_stamp, time_stamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  nebula::drivers::PointXYZICATR,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
    std::uint16_t, channel, channel)(float, azimuth, azimuth)(
    std::uint32_t, time_stamp, time_stamp)(std::uint8_t, return_type, return_type))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  nebula::drivers::PointXYZIRCAEDT,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
    std::uint8_t, return_type,
    return_type)(std::uint16_t, channel, channel)(float, azimuth, azimuth)(
    float, elevation, elevation)(float, distance, distance)(std::uint32_t, time_stamp, time_stamp))

#endif
