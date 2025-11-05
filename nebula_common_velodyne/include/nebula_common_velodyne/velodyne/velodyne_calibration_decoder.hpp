/**
 * \file  calibration.h
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology, University of Texas at Austin
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:25:34 AM piyushk $
 */

#ifndef NEBULA_VELODYNE_CALIBRATION_DECODER_H
#define NEBULA_VELODYNE_CALIBRATION_DECODER_H

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace nebula::drivers
{
struct VelodyneLaserCorrection
{
  float rot_correction;
  float vert_correction;
  float dist_correction;
  bool two_pt_correction_available;
  float dist_correction_x;
  float dist_correction_y;
  float vert_offset_correction;
  float horiz_offset_correction;
  int max_intensity;
  int min_intensity;
  float focal_distance;
  float focal_slope;

  /** cached values calculated when the calibration file is read */
  float cos_rot_correction;   ///< cosine of rot_correction
  float sin_rot_correction;   ///< sine of rot_correction
  float cos_vert_correction;  ///< cosine of vert_correction
  float sin_vert_correction;  ///< sine of vert_correction

  int laser_ring;  ///< ring number for this laser
};

/** \brief Calibration information for the entire device. */
class VelodyneCalibration
{
public:
  float distance_resolution_m;
  std::map<int, VelodyneLaserCorrection> laser_corrections_map;
  std::vector<VelodyneLaserCorrection> laser_corrections;
  int num_lasers{};
  bool initialized;

public:
  VelodyneCalibration() : distance_resolution_m(0.002f), initialized(false) {}
  explicit VelodyneCalibration(const std::string & calibration_file) : distance_resolution_m(0.002f)
  {
    read(calibration_file);
  }

  void read(const std::string & calibration_file);
  void write(const std::string & calibration_file);
};

}  // namespace nebula::drivers

#endif  // NEBULA_VELODYNE_CALIBRATION_DECODER_H
