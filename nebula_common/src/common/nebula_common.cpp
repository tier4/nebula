// Copyright 2024 TIER IV, Inc.

#include <nebula_common/nebula_common.hpp>

namespace nebula::drivers
{
[[maybe_unused]] pcl::PointCloud<PointXYZIR>::Ptr convert_point_xyziradt_to_point_xyzir(
  const pcl::PointCloud<PointXYZIRADT>::ConstPtr & input_pointcloud)
{
  pcl::PointCloud<PointXYZIR>::Ptr output_pointcloud(new pcl::PointCloud<PointXYZIR>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  PointXYZIR point{};
  for (const auto & p : input_pointcloud->points) {
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point.ring = p.ring;
    output_pointcloud->points.emplace_back(point);
  }

  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<PointXYZIR>::Ptr convert_point_xyzircaedt_to_point_xyzir(
  const pcl::PointCloud<PointXYZIRCAEDT>::ConstPtr & input_pointcloud)
{
  pcl::PointCloud<PointXYZIR>::Ptr output_pointcloud(new pcl::PointCloud<PointXYZIR>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  PointXYZIR point{};
  for (const auto & p : input_pointcloud->points) {
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point.ring = p.channel;
    output_pointcloud->points.emplace_back(point);
  }

  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<PointXYZIRADT>::Ptr convert_point_xyzircaedt_to_point_xyziradt(
  const pcl::PointCloud<PointXYZIRCAEDT>::ConstPtr & input_pointcloud, const double stamp)
{
  pcl::PointCloud<PointXYZIRADT>::Ptr output_pointcloud(new pcl::PointCloud<PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  PointXYZIRADT point{};
  for (const auto & p : input_pointcloud->points) {
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point.ring = p.channel;
    point.azimuth = rad2deg(p.azimuth) * 100.0;
    point.distance = p.distance;
    point.time_stamp = stamp + static_cast<double>(p.time_stamp) * 1e-9;
    output_pointcloud->points.emplace_back(point);
  }

  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}
}  // namespace nebula::drivers
