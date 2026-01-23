// Copyright 2024 TIER IV, Inc.

#include <nebula_core_common/nebula_common.hpp>

namespace nebula::drivers
{

PointCloud<PointXYZIR> convert_point_xyzircaedt_to_point_xyzir(
  const PointCloud<PointXYZIRCAEDT> & input_pointcloud)
{
  PointCloud<PointXYZIR> output_pointcloud;
  output_pointcloud.reserve(input_pointcloud.size());
  for (const auto & p : input_pointcloud) {
    output_pointcloud.emplace_back(
      PointXYZIR{p.x, p.y, p.z, {}, static_cast<float>(p.intensity), p.channel});
  }

  return output_pointcloud;
}

PointCloud<PointXYZIRADT> convert_point_xyzircaedt_to_point_xyziradt(
  const PointCloud<PointXYZIRCAEDT> & input_pointcloud, double stamp)
{
  PointCloud<PointXYZIRADT> output_pointcloud;
  output_pointcloud.reserve(input_pointcloud.size());
  for (const auto & p : input_pointcloud) {
    output_pointcloud.emplace_back(
      PointXYZIRADT{
        p.x,
        p.y,
        p.z,
        {},
        static_cast<float>(p.intensity),
        p.channel,
        rad2deg(p.azimuth) * 100.0F,
        p.distance,
        static_cast<uint8_t>(p.return_type),
        stamp + static_cast<double>(p.time_stamp) * 1e-9});
  }

  return output_pointcloud;
}
}  // namespace nebula::drivers
