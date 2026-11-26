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

#include "nebula_hesai_decoders/decoders/pandar_128e4x.hpp"
#include "nebula_hesai_decoders/decoders/pandar_40.hpp"
#include "nebula_hesai_decoders/decoders/pandar_64.hpp"
#include "nebula_hesai_decoders/decoders/pandar_at128.hpp"
#include "nebula_hesai_decoders/decoders/pandar_qt128.hpp"
#include "nebula_hesai_decoders/decoders/pandar_qt64.hpp"
#include "nebula_hesai_decoders/decoders/pandar_xt16.hpp"
#include "nebula_hesai_decoders/decoders/pandar_xt32.hpp"
#include "nebula_hesai_decoders/decoders/pandar_xt32m.hpp"
#include "nebula_hesai_decoders/point_filters.hpp"

#include <nebula_core_common/loggers/console_logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_decoders/point_filters/downsample_mask.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

namespace nebula::drivers
{

namespace
{

std::shared_ptr<loggers::Logger> logger()
{
  return std::make_shared<loggers::ConsoleLogger>("test_point_filters");
}

/// Expect that the filter for `sensor_model` reads a mask of `SensorT`'s size, and looks points up
/// in it
template <typename SensorT>
void expect_reads_the_mask_of(SensorModel sensor_model)
{
  SCOPED_TRACE(sensor_model_to_string(sensor_model));
  constexpr auto azimuth_fov = SensorT::fov_mdeg.azimuth;

  // A white mask keeps every point, whatever the dither pattern
  point_filters::impl::GrayscaleImage mask(
    azimuth_fov.extent() / SensorT::peak_resolution_mdeg.azimuth, SensorT::packet_t::n_channels);
  for (size_t y = 0; y < mask.get_height(); ++y) {
    for (size_t x = 0; x < mask.get_width(); ++x) {
      mask.set_pixel(x, y, 255);
    }
  }
  const auto path = std::filesystem::temp_directory_path() /
                    ("nebula_test_point_filters_" + sensor_model_to_string(sensor_model) + ".png");
  mask.write(path);

  const auto filter = make_downsample_mask_filter(sensor_model, path.string(), logger());
  std::filesystem::remove(path);

  NebulaPoint point{};
  point.azimuth = static_cast<float>(deg2rad((azimuth_fov.start + azimuth_fov.end) / 2000.));
  point.channel = 0;
  EXPECT_FALSE(filter.excluded(point));
}

}  // namespace

TEST(HesaiDownsampleMaskFilter, ReadsTheMaskOfEachSensorModel)
{
  expect_reads_the_mask_of<Pandar64>(SensorModel::HESAI_PANDAR64);
  expect_reads_the_mask_of<Pandar40>(SensorModel::HESAI_PANDAR40P);
  expect_reads_the_mask_of<Pandar40>(SensorModel::HESAI_PANDAR40M);
  expect_reads_the_mask_of<PandarQT64>(SensorModel::HESAI_PANDARQT64);
  expect_reads_the_mask_of<PandarQT128>(SensorModel::HESAI_PANDARQT128);
  expect_reads_the_mask_of<PandarXT16>(SensorModel::HESAI_PANDARXT16);
  expect_reads_the_mask_of<PandarXT32>(SensorModel::HESAI_PANDARXT32);
  expect_reads_the_mask_of<PandarXT32M>(SensorModel::HESAI_PANDARXT32M);
  expect_reads_the_mask_of<PandarAT128>(SensorModel::HESAI_PANDARAT128);
  expect_reads_the_mask_of<PandarAT128P>(SensorModel::HESAI_AT128P);
  expect_reads_the_mask_of<Pandar128E4X>(SensorModel::HESAI_PANDAR128_E4X);
}

TEST(HesaiDownsampleMaskFilter, RejectsUnsupportedSensorModels)
{
  EXPECT_THROW(
    make_downsample_mask_filter(SensorModel::VELODYNE_VLP16, "mask.png", logger()),
    std::invalid_argument);
}

}  // namespace nebula::drivers
