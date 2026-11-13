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

#include <nebula_core_common/util/string_conversions.hpp>
#include <nebula_hesai_common/hesai_common.hpp>

#include <gtest/gtest.h>

#include <cstdint>

namespace
{

using nebula::drivers::get_scan_rate_hz;
using nebula::drivers::SensorModel;

struct ScanRateTestParams
{
  SensorModel sensor_model;
  /// The rotation speed at which the sensor outputs scans at 10 Hz
  uint16_t rpm_at_10_hz;
};

class ScanRateTest : public testing::TestWithParam<ScanRateTestParams>
{
};

TEST_P(ScanRateTest, ScanRateMatchesRotationSpeed)
{
  const auto & [sensor_model, rpm_at_10_hz] = GetParam();

  EXPECT_DOUBLE_EQ(get_scan_rate_hz(sensor_model, rpm_at_10_hz), 10.0);
  EXPECT_DOUBLE_EQ(get_scan_rate_hz(sensor_model, static_cast<uint16_t>(2 * rpm_at_10_hz)), 20.0);
}

// All Hesai sensors output one scan per rotation, except for the AT128, which outputs one scan per
// face of its three-sided mirror
INSTANTIATE_TEST_SUITE_P(
  TestHesaiScanRate, ScanRateTest,
  testing::Values(
    ScanRateTestParams{SensorModel::HESAI_PANDAR64, 600},
    ScanRateTestParams{SensorModel::HESAI_PANDAR40P, 600},
    ScanRateTestParams{SensorModel::HESAI_PANDAR40M, 600},
    ScanRateTestParams{SensorModel::HESAI_PANDARQT64, 600},
    ScanRateTestParams{SensorModel::HESAI_PANDARQT128, 600},
    ScanRateTestParams{SensorModel::HESAI_PANDARXT16, 600},
    ScanRateTestParams{SensorModel::HESAI_PANDARXT32, 600},
    ScanRateTestParams{SensorModel::HESAI_PANDARXT32M, 600},
    ScanRateTestParams{SensorModel::HESAI_PANDARAT128, 200},
    ScanRateTestParams{SensorModel::HESAI_PANDAR128_E3X, 600},
    ScanRateTestParams{SensorModel::HESAI_PANDAR128_E4X, 600}),
  [](const testing::TestParamInfo<ScanRateTestParams> & info) {
    return nebula::util::to_string(info.param.sensor_model);
  });

}  // namespace
