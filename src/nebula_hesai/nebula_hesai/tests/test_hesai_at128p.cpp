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

#include "nebula_hesai_decoders/decoders/hesai_decoder.hpp"
#include "nebula_hesai_decoders/decoders/pandar_at128.hpp"

#include <nebula_hesai_common/hesai_common.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using nebula::Status;
using nebula::drivers::DecodeError;
using nebula::drivers::HesaiCorrection;
using nebula::drivers::HesaiDecoder;
using nebula::drivers::HesaiSensorConfiguration;
using nebula::drivers::PandarAT128P;
using nebula::drivers::ReturnType;
namespace return_mode = nebula::drivers::hesai_packet::return_mode;
using Unit = PandarAT128P::packet_t::body_t::block_t::unit_t;

namespace
{

const char g_correction_path[] = _SRC_CALIBRATION_DIR_PATH "AT128P.dat";

class CountingLogger : public nebula::drivers::loggers::Logger
{
public:
  void debug(const std::string &) override {}
  void info(const std::string &) override {}
  void warn(const std::string &) override {}
  void error(const std::string &) override { ++n_errors; }
  std::shared_ptr<Logger> child(const std::string &) override
  {
    return std::make_shared<CountingLogger>();
  }

  size_t n_errors{0};
};

std::shared_ptr<HesaiSensorConfiguration> make_config()
{
  auto config = std::make_shared<HesaiSensorConfiguration>();
  config->sensor_model = nebula::drivers::SensorModel::HESAI_AT128P;
  config->return_mode = nebula::drivers::ReturnMode::DUAL_LAST_STRONGEST;
  config->min_range = 0.3;
  config->max_range = 300;
  config->cloud_min_angle = 30;
  config->cloud_max_angle = 150;
  config->cut_angle = 150;
  return config;
}

}  // namespace

TEST(TestPandarAT128PDecoder, IgnoresFaultMessagePackets)
{
  auto correction = std::make_shared<HesaiCorrection>();
  ASSERT_EQ(correction->load_from_file(g_correction_path), Status::OK);
  auto logger = std::make_shared<CountingLogger>();
  // The decoder holds large angle lookup tables, so keep it off the stack
  auto decoder = std::make_shared<HesaiDecoder<PandarAT128P>>(
    make_config(), correction, logger, nullptr, nullptr, nullptr);

  std::vector<uint8_t> fault_message(99, 0);
  fault_message[0] = 0xCD;
  fault_message[1] = 0xDC;
  auto fault_result = decoder->unpack(fault_message);
  ASSERT_FALSE(fault_result.metadata_or_error.has_value());
  EXPECT_EQ(fault_result.metadata_or_error.error(), DecodeError::PACKET_IGNORED);
  EXPECT_EQ(logger->n_errors, 0U);

  // Other truncated packets are still reported
  std::vector<uint8_t> truncated_packet(99, 0);
  truncated_packet[0] = 0xEE;
  truncated_packet[1] = 0xFF;
  auto truncated_result = decoder->unpack(truncated_packet);
  ASSERT_FALSE(truncated_result.metadata_or_error.has_value());
  EXPECT_EQ(truncated_result.metadata_or_error.error(), DecodeError::PACKET_PARSE_FAILED);
  EXPECT_EQ(logger->n_errors, 1U);
}

TEST(TestPandarAT128P, StrongestFirstStoresStrongestInFirstBlock)
{
  PandarAT128P sensor;
  Unit strongest{1000, 200, 0};
  Unit first{500, 50, 0};
  const std::vector<const Unit *> units{&strongest, &first};

  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 0, units), ReturnType::STRONGEST);
  EXPECT_EQ(sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 1, units), ReturnType::FIRST);
}

TEST(TestPandarAT128P, StrongestFirstWhenFirstIsAlsoStrongest)
{
  // If the first return is also the strongest, the first block holds the second strongest return
  PandarAT128P sensor;
  Unit second_strongest{1000, 40, 0};
  Unit first_strongest{500, 90, 0};
  const std::vector<const Unit *> units{&second_strongest, &first_strongest};

  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 0, units),
    ReturnType::SECONDSTRONGEST);
  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 1, units),
    ReturnType::FIRST_STRONGEST);
}

TEST(TestPandarAT128P, LastStrongestIsUnchanged)
{
  PandarAT128P sensor;
  Unit last{1000, 10, 0};
  Unit strongest{500, 90, 0};
  const std::vector<const Unit *> units{&last, &strongest};

  EXPECT_EQ(sensor.get_return_type(return_mode::DUAL_LAST_STRONGEST, 0, units), ReturnType::LAST);
  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_LAST_STRONGEST, 1, units), ReturnType::STRONGEST);
}

TEST(TestPandarAT128P, IdenticalReturnsAreReportedAsIdentical)
{
  PandarAT128P sensor;
  Unit a{1000, 90, 0};
  Unit b{1000, 90, 0};
  const std::vector<const Unit *> units{&a, &b};

  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 0, units), ReturnType::IDENTICAL);
  EXPECT_EQ(
    sensor.get_return_type(return_mode::DUAL_FIRST_STRONGEST, 1, units), ReturnType::IDENTICAL);
}
