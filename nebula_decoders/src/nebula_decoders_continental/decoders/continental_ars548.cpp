// Copyright 2023 Tier IV, Inc.
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

#include "nebula_decoders/nebula_decoders_continental/decoders/continental_ars548.hpp"

namespace nebula
{
namespace drivers
{
namespace continental_ars548
{

pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointcloud(
  const continental_msgs::msg::ContinentalArs548DetectionList & msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  output_pointcloud->reserve(msg.detections.size());

  pcl::PointXYZ point{};
  for (const auto & detection : msg.detections) {
    point.x =
      std::cos(detection.elevation_angle) * std::cos(detection.azimuth_angle) * detection.range;
    point.y =
      std::cos(detection.elevation_angle) * std::sin(detection.azimuth_angle) * detection.range;
    point.z = std::sin(detection.elevation_angle) * detection.range;

    output_pointcloud->points.emplace_back(point);
  }

  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointcloud(
  const continental_msgs::msg::ContinentalArs548ObjectList & msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  output_pointcloud->reserve(msg.objects.size());

  pcl::PointXYZ point{};
  for (const auto & detection : msg.objects) {
    point.x = static_cast<float>(detection.position.x);
    point.y = static_cast<float>(detection.position.y);
    point.z = static_cast<float>(detection.position.z);

    output_pointcloud->points.emplace_back(point);
  }

  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

radar_msgs::msg::RadarScan convertToRadarScan(
  const continental_msgs::msg::ContinentalArs548DetectionList & msg)
{
  radar_msgs::msg::RadarScan output_msg;
  output_msg.header = msg.header;
  output_msg.returns.reserve(msg.detections.size());

  radar_msgs::msg::RadarReturn return_msg;
  for (const auto & detection : msg.detections) {
    if (
      detection.invalid_azimuth || detection.invalid_distance || detection.invalid_elevation ||
      detection.invalid_range_rate) {
      continue;
    }

    return_msg.range = detection.range;
    return_msg.azimuth = detection.azimuth_angle;
    return_msg.elevation = detection.elevation_angle;
    return_msg.doppler_velocity = detection.range_rate;
    return_msg.amplitude = detection.rcs;
    output_msg.returns.emplace_back(return_msg);
  }

  return output_msg;
}

radar_msgs::msg::RadarTracks convertToRadarTracks(
  const continental_msgs::msg::ContinentalArs548ObjectList & msg)
{
  radar_msgs::msg::RadarTracks output_msg;
  output_msg.tracks.reserve(msg.objects.size());
  output_msg.header = msg.header;

  constexpr int16_t UNKNOWN_ID = 32000;
  constexpr int16_t CAR_ID = 32001;
  constexpr int16_t TRUCK_ID = 32002;
  constexpr int16_t MOTORCYCLE_ID = 32005;
  constexpr int16_t BICYCLE_ID = 32006;
  constexpr int16_t PEDESTRIAN_ID = 32007;

  radar_msgs::msg::RadarTrack track_msg;
  for (const auto & detection : msg.objects) {
    track_msg.uuid.uuid[0] = static_cast<uint8_t>(detection.object_id & 0xff);
    track_msg.uuid.uuid[1] = static_cast<uint8_t>((detection.object_id >> 8) & 0xff);
    track_msg.uuid.uuid[2] = static_cast<uint8_t>((detection.object_id >> 16) & 0xff);
    track_msg.uuid.uuid[3] = static_cast<uint8_t>((detection.object_id >> 24) & 0xff);
    track_msg.position = detection.position;
    track_msg.velocity = detection.absolute_velocity;
    track_msg.acceleration = detection.absolute_acceleration;
    track_msg.size.x = detection.shape_length_edge_mean;
    track_msg.size.y = detection.shape_width_edge_mean;
    track_msg.size.z = 1.f;

    uint8_t max_score = detection.classification_unknown;
    track_msg.classification = UNKNOWN_ID;

    if (detection.classification_car > max_score) {
      max_score = detection.classification_car;
      track_msg.classification = CAR_ID;
    }
    if (detection.classification_truck > max_score) {
      max_score = detection.classification_truck;
      track_msg.classification = TRUCK_ID;
    }
    if (detection.classification_motorcycle > max_score) {
      max_score = detection.classification_motorcycle;
      track_msg.classification = MOTORCYCLE_ID;
    }
    if (detection.classification_bicycle > max_score) {
      max_score = detection.classification_bicycle;
      track_msg.classification = BICYCLE_ID;
    }
    if (detection.classification_pedestrian > max_score) {
      max_score = detection.classification_pedestrian;
      track_msg.classification = PEDESTRIAN_ID;
    }

    track_msg.position_covariance[0] = static_cast<float>(detection.position_std.x);
    track_msg.position_covariance[1] = detection.position_covariance_xy;
    track_msg.position_covariance[2] = 0.f;
    track_msg.position_covariance[3] = static_cast<float>(detection.position_std.y);
    track_msg.position_covariance[4] = 0.f;
    track_msg.position_covariance[5] = static_cast<float>(detection.position_std.z);

    track_msg.velocity_covariance[0] = static_cast<float>(detection.absolute_velocity_std.x);
    track_msg.velocity_covariance[1] = detection.absolute_velocity_covariance_xy;
    track_msg.velocity_covariance[2] = 0.f;
    track_msg.velocity_covariance[3] = static_cast<float>(detection.absolute_velocity_std.y);
    track_msg.velocity_covariance[4] = 0.f;
    track_msg.velocity_covariance[5] = static_cast<float>(detection.absolute_velocity_std.z);

    track_msg.acceleration_covariance[0] =
      static_cast<float>(detection.absolute_acceleration_std.x);
    track_msg.acceleration_covariance[1] = detection.absolute_acceleration_covariance_xy;
    track_msg.acceleration_covariance[2] = 0.f;
    track_msg.acceleration_covariance[3] =
      static_cast<float>(detection.absolute_acceleration_std.y);
    track_msg.acceleration_covariance[4] = 0.f;
    track_msg.acceleration_covariance[5] =
      static_cast<float>(detection.absolute_acceleration_std.z);

    output_msg.tracks.emplace_back(track_msg);
  }

  return output_msg;
}

}  // namespace continental_ars548
}  // namespace drivers
}  // namespace nebula
