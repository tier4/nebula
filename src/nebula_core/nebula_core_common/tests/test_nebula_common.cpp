// Copyright 2025 TIER IV, Inc.
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

#include "nebula_core_common/nebula_common.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <sstream>
#include <string>

using nebula::drivers::deg2rad;
using nebula::drivers::rad2deg;
using nebula::drivers::return_mode_from_string;
using nebula::drivers::ReturnMode;
using nebula::drivers::ReturnType;
using nebula::drivers::rpm2hz;
using nebula::drivers::sensor_model_from_string;
using nebula::drivers::sensor_model_to_string;
using nebula::drivers::SensorModel;

// ============================================================================
// sensor_model_from_string tests
// ============================================================================

TEST(NebulaCommonTest, SensorModelFromStringHesai)
{
  EXPECT_EQ(sensor_model_from_string("Pandar64"), SensorModel::HESAI_PANDAR64);
  EXPECT_EQ(sensor_model_from_string("Pandar40P"), SensorModel::HESAI_PANDAR40P);
  EXPECT_EQ(sensor_model_from_string("Pandar40M"), SensorModel::HESAI_PANDAR40M);
  EXPECT_EQ(sensor_model_from_string("PandarXT16"), SensorModel::HESAI_PANDARXT16);
  EXPECT_EQ(sensor_model_from_string("PandarXT32"), SensorModel::HESAI_PANDARXT32);
  EXPECT_EQ(sensor_model_from_string("PandarXT32M"), SensorModel::HESAI_PANDARXT32M);
  EXPECT_EQ(sensor_model_from_string("PandarAT128"), SensorModel::HESAI_PANDARAT128);
  EXPECT_EQ(sensor_model_from_string("PandarQT64"), SensorModel::HESAI_PANDARQT64);
  EXPECT_EQ(sensor_model_from_string("PandarQT128"), SensorModel::HESAI_PANDARQT128);
  EXPECT_EQ(sensor_model_from_string("Pandar128E4X"), SensorModel::HESAI_PANDAR128_E4X);
}

TEST(NebulaCommonTest, SensorModelFromStringVelodyne)
{
  EXPECT_EQ(sensor_model_from_string("VLS128"), SensorModel::VELODYNE_VLS128);
  EXPECT_EQ(sensor_model_from_string("HDL64"), SensorModel::VELODYNE_HDL64);
  EXPECT_EQ(sensor_model_from_string("VLP32"), SensorModel::VELODYNE_VLP32);
  EXPECT_EQ(sensor_model_from_string("VLP32MR"), SensorModel::VELODYNE_VLP32MR);
  EXPECT_EQ(sensor_model_from_string("HDL32"), SensorModel::VELODYNE_HDL32);
  EXPECT_EQ(sensor_model_from_string("VLP16"), SensorModel::VELODYNE_VLP16);
}

TEST(NebulaCommonTest, SensorModelFromStringRobosense)
{
  EXPECT_EQ(sensor_model_from_string("Helios"), SensorModel::ROBOSENSE_HELIOS);
  EXPECT_EQ(sensor_model_from_string("Bpearl"), SensorModel::ROBOSENSE_BPEARL_V4);
  EXPECT_EQ(sensor_model_from_string("Bpearl_V4"), SensorModel::ROBOSENSE_BPEARL_V4);
  EXPECT_EQ(sensor_model_from_string("Bpearl_V3"), SensorModel::ROBOSENSE_BPEARL_V3);
}

TEST(NebulaCommonTest, SensorModelFromStringContinental)
{
  EXPECT_EQ(sensor_model_from_string("ARS548"), SensorModel::CONTINENTAL_ARS548);
  EXPECT_EQ(sensor_model_from_string("SRR520"), SensorModel::CONTINENTAL_SRR520);
}

TEST(NebulaCommonTest, SensorModelFromStringUnknown)
{
  EXPECT_EQ(sensor_model_from_string("InvalidSensor"), SensorModel::UNKNOWN);
  EXPECT_EQ(sensor_model_from_string(""), SensorModel::UNKNOWN);
  EXPECT_EQ(sensor_model_from_string("pandar64"), SensorModel::UNKNOWN);  // Case sensitive
}

// ============================================================================
// sensor_model_to_string tests
// ============================================================================

TEST(NebulaCommonTest, SensorModelToStringHesai)
{
  EXPECT_EQ(sensor_model_to_string(SensorModel::HESAI_PANDAR64), "Pandar64");
  EXPECT_EQ(sensor_model_to_string(SensorModel::HESAI_PANDAR40P), "Pandar40P");
  EXPECT_EQ(sensor_model_to_string(SensorModel::HESAI_PANDAR40M), "Pandar40M");
  EXPECT_EQ(sensor_model_to_string(SensorModel::HESAI_PANDARXT16), "PandarXT16");
  EXPECT_EQ(sensor_model_to_string(SensorModel::HESAI_PANDARXT32), "PandarXT32");
  EXPECT_EQ(sensor_model_to_string(SensorModel::HESAI_PANDARXT32M), "PandarXT32M");
  EXPECT_EQ(sensor_model_to_string(SensorModel::HESAI_PANDARAT128), "PandarAT128");
  EXPECT_EQ(sensor_model_to_string(SensorModel::HESAI_PANDARQT64), "PandarQT64");
  EXPECT_EQ(sensor_model_to_string(SensorModel::HESAI_PANDARQT128), "PandarQT128");
  EXPECT_EQ(sensor_model_to_string(SensorModel::HESAI_PANDAR128_E4X), "Pandar128E4X");
}

TEST(NebulaCommonTest, SensorModelToStringVelodyne)
{
  EXPECT_EQ(sensor_model_to_string(SensorModel::VELODYNE_VLS128), "VLS128");
  EXPECT_EQ(sensor_model_to_string(SensorModel::VELODYNE_HDL64), "HDL64");
  EXPECT_EQ(sensor_model_to_string(SensorModel::VELODYNE_VLP32), "VLP32");
  EXPECT_EQ(sensor_model_to_string(SensorModel::VELODYNE_VLP32MR), "VLP32MR");
  EXPECT_EQ(sensor_model_to_string(SensorModel::VELODYNE_HDL32), "HDL32");
  EXPECT_EQ(sensor_model_to_string(SensorModel::VELODYNE_VLP16), "VLP16");
}

TEST(NebulaCommonTest, SensorModelToStringRobosense)
{
  EXPECT_EQ(sensor_model_to_string(SensorModel::ROBOSENSE_HELIOS), "Helios");
  EXPECT_EQ(sensor_model_to_string(SensorModel::ROBOSENSE_BPEARL_V3), "Bpearl_V3");
  EXPECT_EQ(sensor_model_to_string(SensorModel::ROBOSENSE_BPEARL_V4), "Bpearl_V4");
}

TEST(NebulaCommonTest, SensorModelToStringContinental)
{
  EXPECT_EQ(sensor_model_to_string(SensorModel::CONTINENTAL_ARS548), "ARS548");
  EXPECT_EQ(sensor_model_to_string(SensorModel::CONTINENTAL_SRR520), "SRR520");
}

TEST(NebulaCommonTest, SensorModelToStringUnknown)
{
  EXPECT_EQ(sensor_model_to_string(SensorModel::UNKNOWN), "UNKNOWN");
}

// ============================================================================
// sensor_model roundtrip tests
// ============================================================================

TEST(NebulaCommonTest, SensorModelRoundtrip)
{
  // Test that to_string -> from_string -> to_string is consistent
  auto test_roundtrip = [](SensorModel model) {
    std::string str = sensor_model_to_string(model);
    SensorModel parsed = sensor_model_from_string(str);
    EXPECT_EQ(parsed, model) << "Roundtrip failed for: " << str;
  };

  test_roundtrip(SensorModel::HESAI_PANDAR64);
  test_roundtrip(SensorModel::HESAI_PANDAR40P);
  test_roundtrip(SensorModel::VELODYNE_VLP16);
  test_roundtrip(SensorModel::VELODYNE_VLS128);
  test_roundtrip(SensorModel::ROBOSENSE_HELIOS);
  test_roundtrip(SensorModel::CONTINENTAL_ARS548);
}

// ============================================================================
// return_mode_from_string tests
// ============================================================================

TEST(NebulaCommonTest, ReturnModeFromString)
{
  EXPECT_EQ(return_mode_from_string("SingleFirst"), ReturnMode::SINGLE_FIRST);
  EXPECT_EQ(return_mode_from_string("SingleStrongest"), ReturnMode::SINGLE_STRONGEST);
  EXPECT_EQ(return_mode_from_string("SingleLast"), ReturnMode::SINGLE_LAST);
  EXPECT_EQ(return_mode_from_string("Dual"), ReturnMode::DUAL_ONLY);
}

TEST(NebulaCommonTest, ReturnModeFromStringUnknown)
{
  EXPECT_EQ(return_mode_from_string("InvalidMode"), ReturnMode::UNKNOWN);
  EXPECT_EQ(return_mode_from_string(""), ReturnMode::UNKNOWN);
  EXPECT_EQ(return_mode_from_string("singlefirst"), ReturnMode::UNKNOWN);  // Case sensitive
}

// ============================================================================
// deg2rad / rad2deg tests
// ============================================================================

TEST(NebulaCommonTest, Deg2Rad)
{
  EXPECT_FLOAT_EQ(deg2rad(0.0), 0.0f);
  EXPECT_FLOAT_EQ(deg2rad(180.0), static_cast<float>(M_PI));
  EXPECT_FLOAT_EQ(deg2rad(360.0), static_cast<float>(2 * M_PI));
  EXPECT_FLOAT_EQ(deg2rad(90.0), static_cast<float>(M_PI / 2));
  EXPECT_FLOAT_EQ(deg2rad(-180.0), static_cast<float>(-M_PI));
}

TEST(NebulaCommonTest, Rad2Deg)
{
  EXPECT_FLOAT_EQ(rad2deg(0.0), 0.0f);
  EXPECT_FLOAT_EQ(rad2deg(M_PI), 180.0f);
  EXPECT_FLOAT_EQ(rad2deg(2 * M_PI), 360.0f);
  EXPECT_FLOAT_EQ(rad2deg(M_PI / 2), 90.0f);
  EXPECT_FLOAT_EQ(rad2deg(-M_PI), -180.0f);
}

TEST(NebulaCommonTest, Deg2RadRad2DegRoundtrip)
{
  for (double deg = -360.0; deg <= 360.0; deg += 45.0) {
    float rad = deg2rad(deg);
    float back = rad2deg(rad);
    EXPECT_NEAR(back, deg, 1e-5) << "Roundtrip failed for: " << deg;
  }
}

// ============================================================================
// rpm2hz tests
// ============================================================================

TEST(NebulaCommonTest, Rpm2Hz)
{
  EXPECT_DOUBLE_EQ(rpm2hz(60.0), 1.0);
  EXPECT_DOUBLE_EQ(rpm2hz(120.0), 2.0);
  EXPECT_DOUBLE_EQ(rpm2hz(600.0), 10.0);
  EXPECT_DOUBLE_EQ(rpm2hz(0.0), 0.0);
  EXPECT_DOUBLE_EQ(rpm2hz(30.0), 0.5);
}

// ============================================================================
// Stream operator tests for ReturnType
// ============================================================================

TEST(NebulaCommonTest, ReturnTypeStreamOperator)
{
  auto to_string = [](ReturnType rt) {
    std::stringstream ss;
    ss << rt;
    return ss.str();
  };

  EXPECT_EQ(to_string(ReturnType::UNKNOWN), "Unknown");
  EXPECT_EQ(to_string(ReturnType::LAST), "Last");
  EXPECT_EQ(to_string(ReturnType::FIRST), "First");
  EXPECT_EQ(to_string(ReturnType::STRONGEST), "Strongest");
  EXPECT_EQ(to_string(ReturnType::FIRST_WEAK), "FirstWeak");
  EXPECT_EQ(to_string(ReturnType::LAST_WEAK), "LastWeak");
  EXPECT_EQ(to_string(ReturnType::IDENTICAL), "Identical");
  EXPECT_EQ(to_string(ReturnType::SECOND), "Second");
  EXPECT_EQ(to_string(ReturnType::SECONDSTRONGEST), "SecondStrongest");
  EXPECT_EQ(to_string(ReturnType::FIRST_STRONGEST), "FirstStrongest");
  EXPECT_EQ(to_string(ReturnType::LAST_STRONGEST), "LastStrongest");
}

// ============================================================================
// Stream operator tests for ReturnMode
// ============================================================================

TEST(NebulaCommonTest, ReturnModeStreamOperator)
{
  auto to_string = [](ReturnMode rm) {
    std::stringstream ss;
    ss << rm;
    return ss.str();
  };

  EXPECT_EQ(to_string(ReturnMode::UNKNOWN), "Unknown");
  EXPECT_EQ(to_string(ReturnMode::SINGLE_FIRST), "SingleFirst");
  EXPECT_EQ(to_string(ReturnMode::SINGLE_STRONGEST), "SingleStrongest");
  EXPECT_EQ(to_string(ReturnMode::SINGLE_LAST), "SingleLast");
  EXPECT_EQ(to_string(ReturnMode::DUAL_ONLY), "Dual");
  EXPECT_EQ(to_string(ReturnMode::DUAL_FIRST), "DualFirst");
  EXPECT_EQ(to_string(ReturnMode::DUAL_LAST), "DualLast");
  EXPECT_EQ(to_string(ReturnMode::TRIPLE), "Triple");
}

// ============================================================================
// Stream operator tests for SensorModel
// ============================================================================

TEST(NebulaCommonTest, SensorModelStreamOperator)
{
  auto to_string = [](SensorModel sm) {
    std::stringstream ss;
    ss << sm;
    return ss.str();
  };

  EXPECT_EQ(to_string(SensorModel::UNKNOWN), "Sensor Unknown");
  EXPECT_EQ(to_string(SensorModel::HESAI_PANDAR64), "Pandar64");
  EXPECT_EQ(to_string(SensorModel::VELODYNE_VLP16), "VLP16");
  EXPECT_EQ(to_string(SensorModel::ROBOSENSE_HELIOS), "HELIOS");
  EXPECT_EQ(to_string(SensorModel::CONTINENTAL_ARS548), "ARS548");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
