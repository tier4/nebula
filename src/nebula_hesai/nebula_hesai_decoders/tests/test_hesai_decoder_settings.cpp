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

#include "nebula_hesai_decoders/hesai_decoder_settings.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace nebula::drivers
{

namespace
{

/// The decoder settings of `Pandar128E4X.param.yaml`
HesaiSensorConfiguration valid_configuration()
{
  HesaiSensorConfiguration config{};
  config.sensor_model = SensorModel::HESAI_PANDAR128_E4X;
  config.min_range = 0.3;
  config.max_range = 300.0;
  config.dual_return_distance_threshold = 0.1;
  config.rotation_speed = 600;
  config.cloud_min_angle = 0;
  config.cloud_max_angle = 360;
  config.cut_angle = 0.0;
  return config;
}

}  // namespace

TEST(HesaiDecoderSettings, AcceptsValidSettings)
{
  EXPECT_TRUE(validate_decoder_settings(valid_configuration()).has_value());
}

TEST(HesaiDecoderSettings, RejectsMissingDownsampleMask)
{
  auto config = valid_configuration();
  config.downsample_mask_path = "/some/mask.png";

  const auto result = validate_decoder_settings(config);
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("/some/mask.png"), std::string::npos);
}

TEST(HesaiDecoderSettings, RejectsUnknownSensorModel)
{
  auto config = valid_configuration();
  config.sensor_model = SensorModel::UNKNOWN;
  EXPECT_FALSE(validate_decoder_settings(config).has_value());
}

TEST(HesaiDecoderSettings, RejectsFovOutsideOfFullCircle)
{
  auto config = valid_configuration();
  config.cloud_max_angle = 361;
  EXPECT_FALSE(validate_decoder_settings(config).has_value());
}

TEST(HesaiDecoderSettings, RejectsCutAngleOutsideOfFov)
{
  auto config = valid_configuration();
  config.cloud_min_angle = 90;
  config.cloud_max_angle = 270;

  config.cut_angle = 45.0;
  EXPECT_FALSE(validate_decoder_settings(config).has_value());

  config.cut_angle = 180.0;
  EXPECT_TRUE(validate_decoder_settings(config).has_value());
}

TEST(HesaiDecoderSettings, RejectsCutAngleOutsideOfWrappedFov)
{
  auto config = valid_configuration();
  config.cloud_min_angle = 270;
  config.cloud_max_angle = 90;

  config.cut_angle = 180.0;
  EXPECT_FALSE(validate_decoder_settings(config).has_value());

  config.cut_angle = 0.0;
  EXPECT_TRUE(validate_decoder_settings(config).has_value());

  config.cut_angle = 90.0;
  EXPECT_TRUE(validate_decoder_settings(config).has_value());
}

TEST(HesaiDecoderSettings, RejectsCutAngleOutsideOfFullCircle)
{
  const std::array<std::pair<uint16_t, uint16_t>, 3> fields_of_view{
    {{0, 360}, {270, 90}, {90, 90}}};
  const std::array<double, 5> cut_angles{
    -1.0, 361.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()};

  for (const auto & [fov_start, fov_end] : fields_of_view) {
    for (const double cut_angle : cut_angles) {
      auto config = valid_configuration();
      config.cloud_min_angle = fov_start;
      config.cloud_max_angle = fov_end;
      config.cut_angle = cut_angle;
      EXPECT_FALSE(validate_decoder_settings(config).has_value())
        << "FoV " << fov_start << " to " << fov_end << " deg, cut angle " << cut_angle;
    }
  }
}

TEST(HesaiDecoderSettings, RejectsCutAtFovStartForPartialFov)
{
  auto config = valid_configuration();
  config.cloud_min_angle = 90;
  config.cloud_max_angle = 270;

  config.cut_angle = 90.0;
  EXPECT_FALSE(validate_decoder_settings(config).has_value());

  config.cut_angle = 270.0;
  EXPECT_TRUE(validate_decoder_settings(config).has_value());
}

TEST(HesaiDecoderSettings, RejectsCutAt360ForFullFov)
{
  auto config = valid_configuration();
  config.cut_angle = 360.0;
  EXPECT_FALSE(validate_decoder_settings(config).has_value());
}

}  // namespace nebula::drivers
