// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <nebula_core_common/point_cloud.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>

namespace nebula::ros
{

namespace detail
{

/// @brief Convert internal PointField::DataType to sensor_msgs PointField datatype constant
inline uint8_t to_ros_datatype(drivers::PointField::DataType datatype)
{
  switch (datatype) {
    case drivers::PointField::DataType::Int8:
      return sensor_msgs::msg::PointField::INT8;
    case drivers::PointField::DataType::UInt8:
      return sensor_msgs::msg::PointField::UINT8;
    case drivers::PointField::DataType::Int16:
      return sensor_msgs::msg::PointField::INT16;
    case drivers::PointField::DataType::UInt16:
      return sensor_msgs::msg::PointField::UINT16;
    case drivers::PointField::DataType::Int32:
      return sensor_msgs::msg::PointField::INT32;
    case drivers::PointField::DataType::UInt32:
      return sensor_msgs::msg::PointField::UINT32;
    case drivers::PointField::DataType::Float32:
      return sensor_msgs::msg::PointField::FLOAT32;
    case drivers::PointField::DataType::Float64:
      return sensor_msgs::msg::PointField::FLOAT64;
    default:
      throw std::runtime_error("Invalid PointField::DataType");
  }
}

/// @brief Get the size in bytes of a PointField::DataType
inline uint32_t datatype_size(drivers::PointField::DataType datatype)
{
  switch (datatype) {
    case drivers::PointField::DataType::Int8:
    case drivers::PointField::DataType::UInt8:
      return 1;
    case drivers::PointField::DataType::Int16:
    case drivers::PointField::DataType::UInt16:
      return 2;
    case drivers::PointField::DataType::Int32:
    case drivers::PointField::DataType::UInt32:
    case drivers::PointField::DataType::Float32:
      return 4;
    case drivers::PointField::DataType::Float64:
      return 8;
    default:
      throw std::runtime_error("Invalid PointField::DataType");
  }
}

}  // namespace detail

/// @brief Convert a PointCloud to a ROS sensor_msgs::msg::PointCloud2 message
/// @tparam PointT The point type (must have a static fields() method)
/// @param cloud The input point cloud
/// @return The converted ROS message
template <typename PointT>
sensor_msgs::msg::PointCloud2 to_ros_msg(const drivers::PointCloud<PointT> & cloud)
{
  sensor_msgs::msg::PointCloud2 msg;

  // Set dimensions
  msg.height = 1;
  msg.width = static_cast<uint32_t>(cloud.size());

  // Build fields from point type metadata
  const auto point_fields = PointT::fields();
  msg.fields.reserve(point_fields.size());

  for (const auto & field : point_fields) {
    sensor_msgs::msg::PointField ros_field;
    ros_field.name = field.name;
    ros_field.offset = field.offset;
    ros_field.datatype = detail::to_ros_datatype(field.datatype);
    ros_field.count = field.count;
    msg.fields.push_back(ros_field);
  }

  // Set point step (size of one point)
  msg.point_step = static_cast<uint32_t>(sizeof(PointT));
  msg.row_step = msg.point_step * msg.width;

  // Set endianness (assume little endian for x86/ARM)
  msg.is_bigendian = false;

  // Assume dense point cloud (no invalid points)
  msg.is_dense = true;

  // Copy point data
  const size_t data_size = cloud.size() * sizeof(PointT);
  msg.data.resize(data_size);
  if (!cloud.empty()) {
    std::memcpy(msg.data.data(), cloud.data(), data_size);
  }

  return msg;
}

/// @brief Convert a shared_ptr PointCloud to a ROS sensor_msgs::msg::PointCloud2 message
/// @tparam PointT The point type (must have a static fields() method)
/// @param cloud The input point cloud shared_ptr
/// @return The converted ROS message
template <typename PointT>
sensor_msgs::msg::PointCloud2 to_ros_msg(const std::shared_ptr<drivers::PointCloud<PointT>> & cloud)
{
  if (!cloud) {
    return sensor_msgs::msg::PointCloud2{};
  }
  return to_ros_msg(*cloud);
}

/// @brief Convert a shared_ptr to const PointCloud to a ROS sensor_msgs::msg::PointCloud2 message
/// @tparam PointT The point type (must have a static fields() method)
/// @param cloud The input point cloud shared_ptr
/// @return The converted ROS message
template <typename PointT>
sensor_msgs::msg::PointCloud2 to_ros_msg(
  const std::shared_ptr<const drivers::PointCloud<PointT>> & cloud)
{
  if (!cloud) {
    return sensor_msgs::msg::PointCloud2{};
  }
  return to_ros_msg(*cloud);
}

}  // namespace nebula::ros
