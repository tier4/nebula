#include "velodyne/decoders/vlp16_decoder.hpp"

#include <cmath>
#include <utility>

namespace nebula
{
namespace drivers
{
namespace vlp16
{
Vlp16Decoder::Vlp16Decoder(
  const std::shared_ptr<drivers::VelodyneSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::VelodyneCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  calibration_configuration_ = calibration_configuration;

  scan_pc_.reset(new PointCloudXYZIRADT);
  overflow_pc_.reset(new PointCloudXYZIRADT);

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }

  phase_ = (uint16_t)round(sensor_configuration_->scan_phase * 100);
}

bool Vlp16Decoder::hasScanned() { return has_scanned_; }

drivers::PointCloudXYZIRADTPtr Vlp16Decoder::get_pointcloud()
{
  int phase = (uint16_t)round(sensor_configuration_->scan_phase * 100);
  if (!scan_pc_->points.empty()) {
    uint16_t current_azimuth = (int)scan_pc_->points.back().azimuth * 100;
    uint16_t phase_diff = (36000 + current_azimuth - phase) % 36000;
    while (phase_diff < 18000 && scan_pc_->points.size() > 0) {
      overflow_pc_->points.push_back(scan_pc_->points.back());
      scan_pc_->points.pop_back();
      current_azimuth = (int)scan_pc_->points.back().azimuth * 100;
      phase_diff = (36000 + current_azimuth - phase) % 36000;
    }
    overflow_pc_->width = overflow_pc_->points.size();
  }
  return scan_pc_;
}

int Vlp16Decoder::pointsPerPacket()
{
  return BLOCKS_PER_PACKET * VLP16_FIRINGS_PER_BLOCK * VLP16_SCANS_PER_FIRING;
}

void Vlp16Decoder::reset_pointcloud(size_t n_pts)
{
  scan_pc_->points.clear();
  max_pts_ = n_pts * pointsPerPacket();
  scan_pc_->points.reserve(max_pts_);
  reset_overflow();  // transfer existing overflow points to the cleared pointcloud
}

void Vlp16Decoder::reset_overflow()
{
  // Add the overflow buffer points
  for (size_t i = 0; i < overflow_pc_->points.size(); i++) {
    scan_pc_->points.emplace_back(overflow_pc_->points[i]);
  }
  overflow_pc_->points.clear();
  overflow_pc_->points.reserve(max_pts_);
}

void Vlp16Decoder::unpack(const velodyne_msgs::msg::VelodynePacket & velodyne_packet)
{
  const raw_packet_t * raw = (const raw_packet_t *)&velodyne_packet.data[0];
  float last_azimuth_diff = 0;
  uint16_t azimuth_next;
  const uint8_t return_mode = velodyne_packet.data[1204];
  const bool dual_return = (return_mode == RETURN_MODE_DUAL);

  for (uint block = 0; block < BLOCKS_PER_PACKET; block++) {
    // Cache block for use.
    const raw_block_t & current_block = raw->blocks[block];
    if (UPPER_BANK != raw->blocks[block].header) {
      // Do not flood the log with messages, only issue at most one
      // of these warnings per minute.
      return;  // bad packet: skip the rest
    }

    float azimuth_diff;
    uint16_t azimuth;

    // Calculate difference between current and next block's azimuth angle.
    if (block == 0) {
      azimuth = current_block.rotation;
    } else {
      azimuth = azimuth_next;
    }
    if (block < static_cast<uint>(BLOCKS_PER_PACKET - (1 + dual_return))) {
      // Get the next block rotation to calculate how far we rotate between blocks.
      azimuth_next = raw->blocks[block + (1 + dual_return)].rotation;

      // Finds the difference between two sucessive blocks.
      azimuth_diff = static_cast<float>((36000 + azimuth_next - azimuth) % 36000);

      // This is used when the last block is next to predict rotation amount
      last_azimuth_diff = azimuth_diff;
    } else {
      // This makes the assumption the difference between the last block and the next packet is the
      // same as the last to the second to last.
      // Assumes RPM doesn't change much between blocks.
      azimuth_diff =
        (block == static_cast<uint>(BLOCKS_PER_PACKET - dual_return - 1) ? 0 : last_azimuth_diff);
    }

    // Condition added to avoid calculating points which are not in the interesting defined area
    // (min_angle < area < max_angle).
    if (
      (sensor_configuration_->cloud_min_angle < sensor_configuration_->cloud_max_angle &&
       //         azimuth >= sensor_configuration_->cloud_min_angle &&
       azimuth >= sensor_configuration_->cloud_min_angle * 100 &&
       azimuth <= sensor_configuration_->cloud_max_angle * 100) ||
      //         azimuth <= sensor_configuration_->cloud_max_angle) ||
      (sensor_configuration_->cloud_min_angle > sensor_configuration_->cloud_max_angle)) {
      for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; ++firing) {
        for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE) {
          union two_bytes current_return;
          union two_bytes other_return;
          // Distance extraction.
          current_return.bytes[0] = current_block.data[k];
          current_return.bytes[1] = current_block.data[k + 1];

          if (dual_return) {
            other_return.bytes[0] =
              block % 2 ? raw->blocks[block - 1].data[k] : raw->blocks[block + 1].data[k];
            other_return.bytes[1] =
              block % 2 ? raw->blocks[block - 1].data[k + 1] : raw->blocks[block + 1].data[k + 1];
          }
          // Do not process if there is no return, or in dual return mode and the first and last echos are the same.
          if (
            (current_return.bytes[0] == 0 && current_return.bytes[1] == 0) ||
            (dual_return && block % 2 && other_return.bytes[0] == current_return.bytes[0] &&
             other_return.bytes[1] == current_return.bytes[1])) {
            continue;
          }
          {
            VelodyneLaserCorrection & corrections =
              calibration_configuration_->velodyne_calibration.laser_corrections[dsr];
            float distance = current_return.uint *
                             calibration_configuration_->velodyne_calibration.distance_resolution_m;
            if (distance > 1e-6) {
              distance += corrections.dist_correction;
            }

            if (
              distance > sensor_configuration_->min_range &&
              distance < sensor_configuration_->max_range) {
              // Correct for the laser rotation as a function of timing during the firings.
              const float azimuth_corrected_f =
                azimuth +
                (azimuth_diff * ((dsr * VLP16_DSR_TOFFSET) + (firing * VLP16_FIRING_TOFFSET)) /
                 VLP16_BLOCK_TDURATION);
              const uint16_t azimuth_corrected =
                (static_cast<uint16_t>(round(azimuth_corrected_f))) % 36000;

              // Condition added to avoid calculating points which are not in the interesting defined area (min_angle < area < max_angle).
              if (
                //                (azimuth_corrected >= sensor_configuration_->cloud_min_angle &&
                //                 azimuth_corrected <= sensor_configuration_->cloud_max_angle &&
                (azimuth_corrected >= sensor_configuration_->cloud_min_angle * 100 &&
                 azimuth_corrected <= sensor_configuration_->cloud_max_angle * 100 &&
                 sensor_configuration_->cloud_min_angle < sensor_configuration_->cloud_max_angle) ||
                (sensor_configuration_->cloud_min_angle > sensor_configuration_->cloud_max_angle &&
                 (azimuth_corrected <= sensor_configuration_->cloud_max_angle * 100 ||
                  azimuth_corrected >= sensor_configuration_->cloud_min_angle * 100))) {
                //                 (azimuth_corrected <= sensor_configuration_->cloud_max_angle ||
                //                  azimuth_corrected >= sensor_configuration_->cloud_min_angle))) {
                // Convert polar coordinates to Euclidean XYZ.
                const float cos_vert_angle = corrections.cos_vert_correction;
                const float sin_vert_angle = corrections.sin_vert_correction;
                const float cos_rot_correction = corrections.cos_rot_correction;
                const float sin_rot_correction = corrections.sin_rot_correction;

                const float cos_rot_angle = cos_rot_table_[azimuth_corrected] * cos_rot_correction +
                                            sin_rot_table_[azimuth_corrected] * sin_rot_correction;
                const float sin_rot_angle = sin_rot_table_[azimuth_corrected] * cos_rot_correction -
                                            cos_rot_table_[azimuth_corrected] * sin_rot_correction;

                // Compute the distance in the xy plane (w/o accounting for rotation).
                const float xy_distance = distance * cos_vert_angle;

                // Use standard ROS coordinate system (right-hand rule).
                const float x_coord = xy_distance * cos_rot_angle;     // velodyne y
                const float y_coord = -(xy_distance * sin_rot_angle);  // velodyne x
                const float z_coord = distance * sin_vert_angle;       // velodyne z
                const float intensity = current_block.data[k + 2];

                const double time_stamp = (block * 2 + firing) * 55.296 / 1000.0 / 1000.0 +
                                          dsr * 2.304 / 1000.0 / 1000.0 +
                                          rclcpp::Time(velodyne_packet.stamp).seconds();

                // Determine return type.
                uint8_t return_type;
                switch (return_mode) {
                  case RETURN_MODE_DUAL:
                    if (
                      (other_return.bytes[0] == 0 && other_return.bytes[1] == 0) ||
                      (other_return.bytes[0] == current_return.bytes[0] &&
                       other_return.bytes[1] == current_return.bytes[1])) {
                      return_type = RETURN_TYPE::DUAL_ONLY;
                    } else {
                      const float other_intensity = block % 2 ? raw->blocks[block - 1].data[k + 2]
                                                              : raw->blocks[block + 1].data[k + 2];
                      bool first = other_return.uint < current_return.uint ? 0 : 1;
                      bool strongest = other_intensity < intensity ? 1 : 0;
                      if (other_intensity == intensity) {
                        strongest = first ? 0 : 1;
                      }
                      if (first && strongest) {
                        return_type = RETURN_TYPE::DUAL_STRONGEST_FIRST;
                      } else if (!first && strongest) {
                        return_type = RETURN_TYPE::DUAL_STRONGEST_LAST;
                      } else if (first && !strongest) {
                        return_type = RETURN_TYPE::DUAL_WEAK_FIRST;
                      } else if (!first && !strongest) {
                        return_type = RETURN_TYPE::DUAL_WEAK_LAST;
                      } else {
                        return_type = RETURN_TYPE::INVALID;
                      }
                    }
                    break;
                  case RETURN_MODE_STRONGEST:
                    return_type = RETURN_TYPE::SINGLE_STRONGEST;
                    break;
                  case RETURN_MODE_LAST:
                    return_type = RETURN_TYPE::SINGLE_LAST;
                    break;
                  default:
                    return_type = RETURN_TYPE::INVALID;
                }
                drivers::PointXYZIRADT current_point{};
                current_point.x = x_coord;
                current_point.y = y_coord;
                current_point.z = z_coord;
                current_point.return_type = return_type;
                current_point.ring = corrections.laser_ring;
                current_point.azimuth = azimuth_corrected;
                current_point.distance = distance;
                current_point.time_stamp = time_stamp;
                current_point.intensity = intensity;
                scan_pc_->points.emplace_back(current_point);
              }
            }
          }
        }
      }
    }
  }
}

bool Vlp16Decoder::parsePacket(const velodyne_msgs::msg::VelodynePacket & velodyne_packet)
{
  return 0;
}

}  // namespace vlp16
}  // namespace drivers
}  // namespace nebula