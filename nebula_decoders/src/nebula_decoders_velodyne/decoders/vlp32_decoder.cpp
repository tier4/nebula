#include "nebula_decoders/nebula_decoders_velodyne/decoders/vlp32_decoder.hpp"

#include <cmath>
#include <utility>

namespace nebula
{
namespace drivers
{
namespace vlp32
{
Vlp32Decoder::Vlp32Decoder(
  const std::shared_ptr<drivers::VelodyneSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::VelodyneCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  calibration_configuration_ = calibration_configuration;

  first_timestamp = std::numeric_limits<double>::max();

  scan_pc_.reset(new NebulaPointCloud);
  overflow_pc_.reset(new NebulaPointCloud);

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }
  phase_ = (uint16_t)round(sensor_configuration_->scan_phase * 100);
}

bool Vlp32Decoder::hasScanned() { return has_scanned_; }

std::tuple<drivers::NebulaPointCloudPtr, double> Vlp32Decoder::get_pointcloud()
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
  return std::make_tuple(scan_pc_, first_timestamp);
}

int Vlp32Decoder::pointsPerPacket() { return BLOCKS_PER_PACKET * SCANS_PER_BLOCK; }

void Vlp32Decoder::reset_pointcloud(size_t n_pts)
{
  //  scan_pc_.reset(new NebulaPointCloud);
  scan_pc_->points.clear();
  max_pts_ = n_pts * pointsPerPacket();
  scan_pc_->points.reserve(max_pts_);
  reset_overflow();  // transfer existing overflow points to the cleared pointcloud
  first_timestamp = std::numeric_limits<double>::max();
}

void Vlp32Decoder::reset_overflow()
{
  // Add the overflow buffer points
  for (size_t i = 0; i < overflow_pc_->points.size(); i++) {
    scan_pc_->points.emplace_back(overflow_pc_->points[i]);
  }
  overflow_pc_->points.clear();
  overflow_pc_->points.reserve(max_pts_);
}

void Vlp32Decoder::unpack(const velodyne_msgs::msg::VelodynePacket & velodyne_packet)
{
  const raw_packet_t * raw = (const raw_packet_t *)&velodyne_packet.data[0];
  uint8_t return_mode = velodyne_packet.data[RETURN_MODE_INDEX];
  const bool dual_return = (return_mode == RETURN_MODE_DUAL);

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    int bank_origin = 0;
    if (raw->blocks[i].header == LOWER_BANK) {
      // lower bank lasers are [32..63]
      bank_origin = 32;
    }
    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
      float x, y, z;
      float intensity;
      const uint8_t laser_number = j + bank_origin;

      const VelodyneLaserCorrection & corrections =
        calibration_configuration_->velodyne_calibration.laser_corrections[laser_number];

      /** Position Calculation */
      const raw_block_t & block = raw->blocks[i];
      union two_bytes current_return;
      current_return.bytes[0] = block.data[k];
      current_return.bytes[1] = block.data[k + 1];

      union two_bytes other_return;
      if (dual_return) {
        other_return.bytes[0] = i % 2 ? raw->blocks[i - 1].data[k] : raw->blocks[i + 1].data[k];
        other_return.bytes[1] =
          i % 2 ? raw->blocks[i - 1].data[k + 1] : raw->blocks[i + 1].data[k + 1];
      }
      // Do not process if there is no return, or in dual return mode and the first and last echos
      // are the same.
      if (
        (current_return.bytes[0] == 0 && current_return.bytes[1] == 0) ||
        (dual_return && i % 2 && other_return.bytes[0] == current_return.bytes[0] &&
         other_return.bytes[1] == current_return.bytes[1])) {
        continue;
      }

      float distance = current_return.uint *
                       calibration_configuration_->velodyne_calibration.distance_resolution_m;
      if (distance > 1e-6) {
        distance += corrections.dist_correction;
      }

      if (
        distance > sensor_configuration_->min_range &&
        distance < sensor_configuration_->max_range) {
        /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
        if (
          (block.rotation >= sensor_configuration_->cloud_min_angle * 100 &&
           block.rotation <= sensor_configuration_->cloud_max_angle * 100 &&
           sensor_configuration_->cloud_min_angle < sensor_configuration_->cloud_max_angle) ||
          (sensor_configuration_->cloud_min_angle > sensor_configuration_->cloud_max_angle &&
           (raw->blocks[i].rotation <= sensor_configuration_->cloud_max_angle * 100 ||
            raw->blocks[i].rotation >= sensor_configuration_->cloud_min_angle * 100))) {
          const float cos_vert_angle = corrections.cos_vert_correction;
          const float sin_vert_angle = corrections.sin_vert_correction;
          const float cos_rot_correction = corrections.cos_rot_correction;
          const float sin_rot_correction = corrections.sin_rot_correction;

          const float cos_rot_angle = cos_rot_table_[block.rotation] * cos_rot_correction +
                                      sin_rot_table_[block.rotation] * sin_rot_correction;
          const float sin_rot_angle = sin_rot_table_[block.rotation] * cos_rot_correction -
                                      cos_rot_table_[block.rotation] * sin_rot_correction;

          const float horiz_offset = corrections.horiz_offset_correction;
          const float vert_offset = corrections.vert_offset_correction;

          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) {
            xx = -xx;
          }
          if (yy < 0) {
            yy = -yy;
          }

          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = (corrections.dist_correction - corrections.dist_correction_x) *
                                (xx - 2.4) / (25.04 - 2.4) +
                              corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = (corrections.dist_correction - corrections.dist_correction_y) *
                                (yy - 1.93) / (25.04 - 1.93) +
                              corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }

          const float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
          /// the expression wiht '-' is proved to be better than the one with '+'
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

          const float distance_y = distance + distance_corr_y;
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

          /** Use standard ROS coordinate system (right-hand rule) */
          const float x_coord = y;
          const float y_coord = -x;
          const float z_coord = z;

          /** Intensity Calculation */
          const float min_intensity = corrections.min_intensity;
          const float max_intensity = corrections.max_intensity;

          intensity = raw->blocks[i].data[k + 2];

          const float focal_offset = 256 * (1 - corrections.focal_distance / 13100) *
                                     (1 - corrections.focal_distance / 13100);
          const float focal_slope = corrections.focal_slope;
          float sqr = (1 - static_cast<float>(current_return.uint) / 65535) *
                      (1 - static_cast<float>(current_return.uint) / 65535);
          intensity += focal_slope * (std::abs(focal_offset - 256 * sqr));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          double time_stamp = i * 55.296 / 1000.0 / 1000.0 + j * 2.304 / 1000.0 / 1000.0;  // +
          auto ts = rclcpp::Time(velodyne_packet.stamp).seconds();
          if (ts < first_timestamp) {
            first_timestamp = ts;
          }

          nebula::drivers::ReturnType return_type;
          switch (return_mode) {
            case RETURN_MODE_DUAL:
              if (
                (other_return.bytes[0] == 0 && other_return.bytes[1] == 0) ||
                (other_return.bytes[0] == current_return.bytes[0] &&
                 other_return.bytes[1] == current_return.bytes[1])) {
                return_type = drivers::ReturnType::IDENTICAL;
              } else {
                const float other_intensity =
                  i % 2 ? raw->blocks[i - 1].data[k + 2] : raw->blocks[i + 1].data[k + 2];
                bool first = other_return.uint < current_return.uint ? 0 : 1;
                bool strongest = other_intensity < intensity ? 1 : 0;
                if (other_intensity == intensity) {
                  strongest = first ? 0 : 1;
                }
                if (first && strongest) {
                  return_type = drivers::ReturnType::FIRST_STRONGEST;
                } else if (!first && strongest) {
                  return_type = drivers::ReturnType::LAST_STRONGEST;
                } else if (first && !strongest) {
                  return_type = drivers::ReturnType::FIRST_WEAK;
                } else if (!first && !strongest) {
                  return_type = drivers::ReturnType::LAST_WEAK;
                } else {
                  return_type = drivers::ReturnType::UNKNOWN;
                }
              }
              break;
            case RETURN_MODE_STRONGEST:
              return_type = drivers::ReturnType::STRONGEST;
              break;
            case RETURN_MODE_LAST:
              return_type = drivers::ReturnType::LAST;
              break;
            default:
              return_type = drivers::ReturnType::UNKNOWN;
          }
          drivers::NebulaPoint current_point{};
          current_point.x = x_coord;
          current_point.y = y_coord;
          current_point.z = z_coord;
          current_point.return_type = static_cast<uint8_t>(return_type);
          current_point.channel = corrections.laser_ring;
          current_point.azimuth = raw->blocks[i].rotation;
          current_point.time_stamp = time_stamp;
          current_point.intensity = intensity;
          scan_pc_->points.emplace_back(current_point);
        }
      }
    }
  }
}

bool Vlp32Decoder::parsePacket(
  [[maybe_unused]] const velodyne_msgs::msg::VelodynePacket & velodyne_packet)
{
  return 0;
}

}  // namespace vlp32
}  // namespace drivers
}  // namespace nebula
