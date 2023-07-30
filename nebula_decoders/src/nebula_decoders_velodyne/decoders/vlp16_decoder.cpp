#include "nebula_decoders/nebula_decoders_velodyne/decoders/vlp16_decoder.hpp"

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

  scan_timestamp_ = -1;

  scan_pc_.reset(new NebulaPointCloud);
  overflow_pc_.reset(new NebulaPointCloud);

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    rotation_radians_[rot_index] = rotation;
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }
  // timing table calculation, from velodyne user manual p.64
  timing_offsets_.resize(BLOCKS_PER_PACKET);
  for (size_t i=0; i < timing_offsets_.size(); ++i){
    timing_offsets_[i].resize(32);
  }
  double full_firing_cycle_s = 55.296 * 1e-6;
  double single_firing_s = 2.304 * 1e-6;
  double data_block_index, data_point_index;
  bool dual_mode = sensor_configuration_->return_mode==ReturnMode::DUAL;
  // compute timing offsets
  for (size_t x = 0; x < timing_offsets_.size(); ++x){
    for (size_t y = 0; y < timing_offsets_[x].size(); ++y){
      if (dual_mode){
        data_block_index = (x - (x % 2)) + (y / 16);
      }
      else{
        data_block_index = (x * 2) + (y / 16);
      }
      data_point_index = y % 16;
      timing_offsets_[x][y] = (full_firing_cycle_s * data_block_index) + (single_firing_s * data_point_index);
    }
  }

  phase_ = (uint16_t)round(sensor_configuration_->scan_phase * 100);
}

bool Vlp16Decoder::hasScanned() { return has_scanned_; }

std::tuple<drivers::NebulaPointCloudPtr, double> Vlp16Decoder::get_pointcloud()
{
  double phase = angles::from_degrees(sensor_configuration_->scan_phase);
  if (!scan_pc_->points.empty()) {
    auto current_azimuth = scan_pc_->points.back().azimuth;
    auto phase_diff = static_cast<size_t>(angles::to_degrees(2*M_PI + current_azimuth - phase)) % 360;
    while (phase_diff < M_PI_2 && !scan_pc_->points.empty()) {
      overflow_pc_->points.push_back(scan_pc_->points.back());
      scan_pc_->points.pop_back();
      current_azimuth = scan_pc_->points.back().azimuth;
      phase_diff = static_cast<size_t>(angles::to_degrees(2*M_PI + current_azimuth - phase)) % 360;
    }
    overflow_pc_->width = overflow_pc_->points.size();
    scan_pc_->width = scan_pc_->points.size();
    scan_pc_->height = 1;
  }
  return std::make_tuple(scan_pc_, scan_timestamp_);
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
  scan_timestamp_ = -1;
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
  const uint8_t return_mode = velodyne_packet.data[RETURN_MODE_INDEX];
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

      // Finds the difference between two successive blocks.
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
       azimuth >= sensor_configuration_->cloud_min_angle * 100 &&
       azimuth <= sensor_configuration_->cloud_max_angle * 100) ||
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
          // Apply timestamp if this is the first new packet in the scan.
          auto block_timestamp = rclcpp::Time(velodyne_packet.stamp).seconds();
          if (scan_timestamp_ < 0) {
            scan_timestamp_ = block_timestamp;
          }
          // Do not process if there is no return, or in dual return mode and the first and last
          // echos are the same.
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
                 VLP16_BLOCK_DURATION);
              const uint16_t azimuth_corrected =
                (static_cast<uint16_t>(round(azimuth_corrected_f))) % 36000;

              // Condition added to avoid calculating points which are not in the interesting
              // defined area (min_angle < area < max_angle).
              if (
                (azimuth_corrected >= sensor_configuration_->cloud_min_angle * 100 &&
                 azimuth_corrected <= sensor_configuration_->cloud_max_angle * 100 &&
                 sensor_configuration_->cloud_min_angle < sensor_configuration_->cloud_max_angle) ||
                (sensor_configuration_->cloud_min_angle > sensor_configuration_->cloud_max_angle &&
                 (azimuth_corrected <= sensor_configuration_->cloud_max_angle * 100 ||
                  azimuth_corrected >= sensor_configuration_->cloud_min_angle * 100))) {
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
                const uint8_t intensity = current_block.data[k + 2];

                double point_time_offset =  timing_offsets_[block][firing * 16 + dsr];

                // Determine return type.
                uint8_t return_type;
                switch (return_mode) {
                  case RETURN_MODE_DUAL:
                    if (
                      (other_return.bytes[0] == 0 && other_return.bytes[1] == 0) ||
                      (other_return.bytes[0] == current_return.bytes[0] &&
                       other_return.bytes[1] == current_return.bytes[1])) {
                      return_type = static_cast<uint8_t>(drivers::ReturnType::IDENTICAL);
                    } else {
                      const uint8_t other_intensity = block % 2 ? raw->blocks[block - 1].data[k + 2]
                                                              : raw->blocks[block + 1].data[k + 2];
                      bool first = current_return.uint > other_return.uint ;
                      bool strongest = intensity > other_intensity;
                      if (other_intensity == intensity) {
                        strongest = !first;
                      }
                      if (first && strongest) {
                        return_type = static_cast<uint8_t>(drivers::ReturnType::FIRST_STRONGEST);
                      } else if (!first && strongest) {
                        return_type = static_cast<uint8_t>(drivers::ReturnType::LAST_STRONGEST);
                      } else if (first && !strongest) {
                        return_type = static_cast<uint8_t>(drivers::ReturnType::FIRST_WEAK);
                      } else if (!first && !strongest) {
                        return_type = static_cast<uint8_t>(drivers::ReturnType::LAST_WEAK);
                      } else {
                        return_type = static_cast<uint8_t>(drivers::ReturnType::UNKNOWN);
                      }
                    }
                    break;
                  case RETURN_MODE_STRONGEST:
                    return_type = static_cast<uint8_t>(drivers::ReturnType::STRONGEST);
                    break;
                  case RETURN_MODE_LAST:
                    return_type = static_cast<uint8_t>(drivers::ReturnType::LAST);
                    break;
                  default:
                    return_type = static_cast<uint8_t>(drivers::ReturnType::UNKNOWN);
                }
                drivers::NebulaPoint current_point{};
                current_point.x = x_coord;
                current_point.y = y_coord;
                current_point.z = z_coord;
                current_point.return_type = return_type;
                current_point.channel = corrections.laser_ring;
                current_point.azimuth = rotation_radians_[azimuth_corrected];
                current_point.elevation = sin_vert_angle;
                auto point_ts = block_timestamp - scan_timestamp_ + point_time_offset;
                if (point_ts < 0)
                  point_ts = 0;
                current_point.time_stamp = static_cast<uint32_t>(point_ts*1e9);
                current_point.intensity = intensity;
                current_point.distance = distance;
                scan_pc_->points.emplace_back(current_point);
              }
            }
          }
        }
      }
    }
  }
}

bool Vlp16Decoder::parsePacket(
  [[maybe_unused]] const velodyne_msgs::msg::VelodynePacket & velodyne_packet)
{
  return 0;
}

}  // namespace vlp16
}  // namespace drivers
}  // namespace nebula
