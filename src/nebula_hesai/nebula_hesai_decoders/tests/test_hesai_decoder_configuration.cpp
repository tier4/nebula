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

#include "nebula_hesai_decoders/hesai_decoder_configuration.hpp"

#include <gtest/gtest.h>

#include <string>

namespace nebula::drivers
{

namespace
{

/// The parameters of `Pandar128E4X.param.yaml`, including the ones the decoder does not use.
nlohmann::json full_parameter_tree()
{
  return nlohmann::json::parse(R"({
    "host_ip": "192.168.1.10",
    "sensor_ip": "192.168.1.201",
    "data_port": 2368,
    "launch_hw": true,
    "frame_id": "hesai",
    "min_range": 0.3,
    "max_range": 300.0,
    "cloud_min_angle": 0,
    "cloud_max_angle": 360,
    "sync_angle": 0,
    "cut_angle": 0.0,
    "sensor_model": "Pandar128E4X",
    "rotation_speed": 600,
    "return_mode": "Dual",
    "ptp_profile": "automotive",
    "dual_return_distance_threshold": 0.1,
    "hires_mode": true,
    "diagnostics": {"packet_loss": {"error_threshold": 10}}
  })");
}

HesaiDecoderConfiguration valid_configuration()
{
  return full_parameter_tree().get<HesaiDecoderConfiguration>();
}

}  // namespace

TEST(HesaiDecoderConfiguration, ParsesDecoderSettingsFromFullParameterTree)
{
  const auto config = valid_configuration();

  EXPECT_EQ(config.sensor_model, SensorModel::HESAI_PANDAR128_E4X);
  EXPECT_DOUBLE_EQ(config.min_range, 0.3);
  EXPECT_DOUBLE_EQ(config.max_range, 300.0);
  EXPECT_DOUBLE_EQ(config.dual_return_distance_threshold, 0.1);
  EXPECT_EQ(config.rotation_speed, 600);
  EXPECT_EQ(config.cloud_min_angle, 0);
  EXPECT_EQ(config.cloud_max_angle, 360);
  EXPECT_DOUBLE_EQ(config.cut_angle, 0.0);
  EXPECT_FALSE(config.downsample_mask_path.has_value());
  EXPECT_TRUE(validate(config).has_value());
}

TEST(HesaiDecoderConfiguration, ParsesDownsampleMaskPath)
{
  auto j = full_parameter_tree();

  j["point_filters"]["downsample_mask"]["path"] = "";
  EXPECT_FALSE(j.get<HesaiDecoderConfiguration>().downsample_mask_path.has_value());

  j["point_filters"]["downsample_mask"]["path"] = "/some/mask.png";
  EXPECT_EQ(j.get<HesaiDecoderConfiguration>().downsample_mask_path, "/some/mask.png");
}

TEST(HesaiDecoderConfiguration, ThrowsOnMissingOrMistypedSetting)
{
  auto missing = full_parameter_tree();
  missing.erase("min_range");
  EXPECT_THROW(missing.get<HesaiDecoderConfiguration>(), nlohmann::json::exception);

  auto mistyped = full_parameter_tree();
  mistyped["rotation_speed"] = "fast";
  EXPECT_THROW(mistyped.get<HesaiDecoderConfiguration>(), nlohmann::json::exception);
}

TEST(HesaiDecoderConfiguration, RoundTripsThroughJson)
{
  auto config = valid_configuration();
  config.cloud_min_angle = 90;
  config.cloud_max_angle = 270;
  config.cut_angle = 270.0;
  config.downsample_mask_path = "/some/mask.png";

  const auto round_tripped = nlohmann::json(config).get<HesaiDecoderConfiguration>();

  EXPECT_EQ(round_tripped.sensor_model, config.sensor_model);
  EXPECT_DOUBLE_EQ(round_tripped.min_range, config.min_range);
  EXPECT_DOUBLE_EQ(round_tripped.max_range, config.max_range);
  EXPECT_DOUBLE_EQ(
    round_tripped.dual_return_distance_threshold, config.dual_return_distance_threshold);
  EXPECT_EQ(round_tripped.rotation_speed, config.rotation_speed);
  EXPECT_EQ(round_tripped.cloud_min_angle, config.cloud_min_angle);
  EXPECT_EQ(round_tripped.cloud_max_angle, config.cloud_max_angle);
  EXPECT_DOUBLE_EQ(round_tripped.cut_angle, config.cut_angle);
  EXPECT_EQ(round_tripped.downsample_mask_path, config.downsample_mask_path);
}

TEST(HesaiDecoderConfiguration, RejectsUnknownSensorModel)
{
  auto j = full_parameter_tree();
  j["sensor_model"] = "NotASensor";

  EXPECT_FALSE(validate(j.get<HesaiDecoderConfiguration>()).has_value());
}

TEST(HesaiDecoderConfiguration, RejectsFovOutsideOfFullCircle)
{
  auto config = valid_configuration();
  config.cloud_max_angle = 361;

  EXPECT_FALSE(validate(config).has_value());
}

TEST(HesaiDecoderConfiguration, RejectsCutAngleOutsideOfFov)
{
  auto config = valid_configuration();
  config.cloud_min_angle = 90;
  config.cloud_max_angle = 270;

  config.cut_angle = 45.0;
  EXPECT_FALSE(validate(config).has_value());

  config.cut_angle = 180.0;
  EXPECT_TRUE(validate(config).has_value());
}

TEST(HesaiDecoderConfiguration, RejectsCutAtFovStartForPartialFov)
{
  auto config = valid_configuration();
  config.cloud_min_angle = 90;
  config.cloud_max_angle = 270;

  config.cut_angle = 90.0;
  EXPECT_FALSE(validate(config).has_value());

  config.cut_angle = 270.0;
  EXPECT_TRUE(validate(config).has_value());
}

TEST(HesaiDecoderConfiguration, RejectsCutAt360ForFullFov)
{
  auto config = valid_configuration();
  config.cut_angle = 360.0;

  EXPECT_FALSE(validate(config).has_value());
}

TEST(HesaiDecoderConfiguration, RejectsMissingDownsampleMask)
{
  auto config = valid_configuration();
  config.downsample_mask_path = "/this/file/does/not/exist.png";

  const auto result = validate(config);
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("/this/file/does/not/exist.png"), std::string::npos);
}

TEST(HesaiDecoderConfiguration, ExtractsDecoderSettingsFromSensorConfiguration)
{
  HesaiSensorConfiguration sensor_config{};
  sensor_config.sensor_model = SensorModel::HESAI_PANDARXT32;
  sensor_config.min_range = 0.5;
  sensor_config.max_range = 120.0;
  sensor_config.dual_return_distance_threshold = 0.2;
  sensor_config.rotation_speed = 1200;
  sensor_config.cloud_min_angle = 10;
  sensor_config.cloud_max_angle = 350;
  sensor_config.cut_angle = 350.0;
  sensor_config.downsample_mask_path = "/some/mask.png";

  const auto config = make_decoder_configuration(sensor_config);

  EXPECT_EQ(config.sensor_model, SensorModel::HESAI_PANDARXT32);
  EXPECT_DOUBLE_EQ(config.min_range, 0.5);
  EXPECT_DOUBLE_EQ(config.max_range, 120.0);
  EXPECT_DOUBLE_EQ(config.dual_return_distance_threshold, 0.2);
  EXPECT_EQ(config.rotation_speed, 1200);
  EXPECT_EQ(config.cloud_min_angle, 10);
  EXPECT_EQ(config.cloud_max_angle, 350);
  EXPECT_DOUBLE_EQ(config.cut_angle, 350.0);
  EXPECT_EQ(config.downsample_mask_path, "/some/mask.png");
}

}  // namespace nebula::drivers
