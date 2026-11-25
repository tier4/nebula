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

#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_decoders/point_filters/downsample_mask.hpp>

#include <memory>
#include <string>

namespace nebula::drivers
{

/// @brief The downsample mask filter, as the decoder of `SensorT` applies it
/// @param export_dithered_mask Whether to write the dithered mask next to the mask file
/// @throws std::runtime_error if the mask cannot be read or does not fit the sensor
template <typename SensorT>
point_filters::DownsampleMaskFilter make_downsample_mask_filter(
  const std::string & path, const std::shared_ptr<loggers::Logger> & logger,
  bool export_dithered_mask)
{
  return point_filters::DownsampleMaskFilter(
    path, SensorT::fov_mdeg.azimuth, SensorT::peak_resolution_mdeg.azimuth,
    SensorT::packet_t::n_channels, logger, export_dithered_mask, SensorT{}.get_dither_transform());
}

/// @brief The downsample mask filter, as the decoder of `sensor_model` applies it. Unlike the
/// decoder, it does not write the dithered mask.
/// @throws std::invalid_argument if the sensor model is not supported
/// @throws std::runtime_error if the mask cannot be read or does not fit the sensor
point_filters::DownsampleMaskFilter make_downsample_mask_filter(
  SensorModel sensor_model, const std::string & path,
  const std::shared_ptr<loggers::Logger> & logger);

}  // namespace nebula::drivers
