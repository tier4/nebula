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

#include <nebula_hesai_common/hesai_common.hpp>

#include <gtest/gtest.h>

using nebula::drivers::get_scan_rate_hz;
using nebula::drivers::SensorModel;

TEST(TestHesaiScanRate, RotatingSensorsOutputOneScanPerRotation)
{
  EXPECT_DOUBLE_EQ(get_scan_rate_hz(SensorModel::HESAI_PANDAR128_E4X, 600), 10.0);
  EXPECT_DOUBLE_EQ(get_scan_rate_hz(SensorModel::HESAI_PANDARQT128, 1200), 20.0);
}

TEST(TestHesaiScanRate, AT128OutputsOneScanPerMirrorFace)
{
  EXPECT_DOUBLE_EQ(get_scan_rate_hz(SensorModel::HESAI_PANDARAT128, 200), 10.0);
  EXPECT_DOUBLE_EQ(get_scan_rate_hz(SensorModel::HESAI_PANDARAT128, 400), 20.0);
}
