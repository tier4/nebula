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
using nebula::drivers::PandarAT128;

namespace
{

const char g_correction_path[] = _SRC_CALIBRATION_DIR_PATH "PandarAT128.dat";

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
  config->sensor_model = nebula::drivers::SensorModel::HESAI_PANDARAT128;
  config->return_mode = nebula::drivers::ReturnMode::DUAL_LAST_STRONGEST;
  config->min_range = 0.3;
  config->max_range = 300;
  config->cloud_min_angle = 30;
  config->cloud_max_angle = 150;
  config->cut_angle = 150;
  return config;
}

}  // namespace

TEST(TestHesaiDecoder, IgnoresFaultMessagePackets)
{
  auto correction = std::make_shared<HesaiCorrection>();
  ASSERT_EQ(correction->load_from_file(g_correction_path), Status::OK);
  auto logger = std::make_shared<CountingLogger>();
  // The decoder holds large angle lookup tables, so keep it off the stack
  auto decoder = std::make_shared<HesaiDecoder<PandarAT128>>(
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
