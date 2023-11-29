/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not
agree to this license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
and the following disclaimer in the documentation and/or other materials provided with the
distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other
contributors may be used to endorse or promote products derived from this software without specific
prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <nebula_common/robosense/rs_log.hpp>

#include <map>
#include <string>

namespace nebula
{
namespace drivers
{

enum SplitFrameMode { SPLIT_BY_ANGLE = 1, SPLIT_BY_FIXED_BLKS, SPLIT_BY_CUSTOM_BLKS };

struct RSDecoderParam  ///< LiDAR decoder parameter
{
  bool config_from_file = false;  ///< Internal use only for debugging
  std::string angle_path = "";    ///< Internal use only for debugging
  bool wait_for_difop = true;     ///< true: start sending point cloud until receive difop packet
  float min_distance =
    0.0f;  ///< min/max distances of point cloud range. valid if min distance or max distance > 0
  float max_distance = 0.0f;
  float start_angle = 0.0f;  ///< Start angle of point cloud
  float end_angle = 360.0f;  ///< End angle of point cloud
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;
  ///< 1: Split frames by split_angle;
  ///< 2: Split frames by fixed number of blocks;
  ///< 3: Split frames by custom number of blocks (num_blks_split)
  float split_angle =
    0.0f;  ///< Split angle(degree) used to split frame, only be used when split_frame_mode=1
  uint16_t num_blks_split =
    1;  ///< Number of packets in one frame, only be used when split_frame_mode=3
  bool use_lidar_clock =
    false;  ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  bool dense_points = false;  ///< true: discard NAN points; false: reserve NAN points
  bool ts_first_point =
    false;  ///< true: time-stamp point cloud with the first point; false: with the last point;

  void print() const
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Decoder Parameters " << RS_REND;
    RS_INFOL << "wait_for_difop: " << wait_for_difop << RS_REND;
    RS_INFOL << "min_distance: " << min_distance << RS_REND;
    RS_INFOL << "max_distance: " << max_distance << RS_REND;
    RS_INFOL << "start_angle: " << start_angle << RS_REND;
    RS_INFOL << "end_angle: " << end_angle << RS_REND;
    RS_INFOL << "use_lidar_clock: " << use_lidar_clock << RS_REND;
    RS_INFOL << "dense_points: " << dense_points << RS_REND;
    RS_INFOL << "config_from_file: " << config_from_file << RS_REND;
    RS_INFOL << "angle_path: " << angle_path << RS_REND;
    RS_INFOL << "split_frame_mode: " << split_frame_mode << RS_REND;
    RS_INFOL << "split_angle: " << split_angle << RS_REND;
    RS_INFOL << "num_blks_split: " << num_blks_split << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
  }
};

struct DeviceStatus
{
  float voltage = 0.0f;
};

}  // namespace drivers
}  // namespace nebula
