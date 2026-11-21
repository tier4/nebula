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
#include "nebula_hesai_decoders/decoders/pandar_128e4x.hpp"
#include "nebula_hesai_decoders/decoders/pandar_40.hpp"
#include "nebula_hesai_decoders/decoders/pandar_64.hpp"
#include "nebula_hesai_decoders/decoders/pandar_at128.hpp"
#include "nebula_hesai_decoders/decoders/pandar_qt128.hpp"
#include "nebula_hesai_decoders/decoders/pandar_qt64.hpp"
#include "nebula_hesai_decoders/decoders/pandar_xt16.hpp"
#include "nebula_hesai_decoders/decoders/pandar_xt32.hpp"
#include "nebula_hesai_decoders/decoders/pandar_xt32m.hpp"

#include <nebula_hesai_common/hesai_common.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers
{

namespace
{

class SilentLogger : public loggers::Logger
{
public:
  void debug(const std::string &) override {}
  void info(const std::string &) override {}
  void warn(const std::string &) override {}
  void error(const std::string &) override {}
  std::shared_ptr<Logger> child(const std::string &) override
  {
    return std::make_shared<SilentLogger>();
  }
};

/// Whether a return mode is one of the return modes defined by Hesai, and the packet format has
/// room for its returns
bool is_supported(uint8_t return_mode, size_t max_returns)
{
  switch (return_mode) {
    case 0x33:
    case 0x34:
    case 0x37:
    case 0x38:
      return true;
    case 0x39:
    case 0x3a:
    case 0x3b:
    case 0x3c:
    case 0x3e:
      return max_returns >= 2;
    case 0x3d:
      return max_returns >= 3;
    default:
      return false;
  }
}

/// Decode a packet with every possible return mode. Only supported ones may be decoded, the others
/// have to be reported as parse errors.
template <typename SensorT>
void expect_only_supported_return_modes_to_be_decoded(
  SensorModel sensor_model, const std::string & calibration_file, uint16_t fov_start = 0,
  uint16_t fov_end = 360, double cut_angle = 0)
{
  auto config = std::make_shared<HesaiSensorConfiguration>();
  config->sensor_model = sensor_model;
  config->min_range = 0.3;
  config->max_range = 300;
  config->cloud_min_angle = fov_start;
  config->cloud_max_angle = fov_end;
  config->cut_angle = cut_angle;

  auto calibration = std::make_shared<typename SensorT::angle_corrector_t::correction_data_t>();
  ASSERT_EQ(calibration->load_from_file(_SRC_CALIBRATION_DIR_PATH + calibration_file), Status::OK);

  // The decoder holds large angle lookup tables, so keep it off the stack
  auto decoder = std::make_shared<HesaiDecoder<SensorT>>(
    config, calibration, std::make_shared<SilentLogger>(), nullptr, nullptr, nullptr);

  using packet_t = typename SensorT::packet_t;
  for (unsigned int return_mode = 0; return_mode <= 0xFF; ++return_mode) {
    SCOPED_TRACE(testing::Message() << "return mode 0x" << std::hex << return_mode);

    packet_t packet{};
    packet.tail.return_mode = static_cast<uint8_t>(return_mode);
    std::vector<uint8_t> bytes(sizeof(packet_t));
    std::memcpy(bytes.data(), &packet, sizeof(packet_t));

    try {
      const auto result = decoder->unpack(bytes);
      if (is_supported(return_mode, packet_t::max_returns)) {
        EXPECT_TRUE(result.metadata_or_error.has_value());
      } else if (result.metadata_or_error.has_value()) {
        ADD_FAILURE() << "Decoded a packet with an unsupported return mode";
      } else {
        EXPECT_EQ(result.metadata_or_error.error(), DecodeError::PACKET_PARSE_FAILED);
      }
    } catch (const std::exception & e) {
      ADD_FAILURE() << "unpack() threw: " << e.what();
    }
  }
}

}  // namespace

TEST(HesaiReturnMode, Pandar64)
{
  expect_only_supported_return_modes_to_be_decoded<Pandar64>(
    SensorModel::HESAI_PANDAR64, "Pandar64.csv");
}

TEST(HesaiReturnMode, Pandar40)
{
  expect_only_supported_return_modes_to_be_decoded<Pandar40>(
    SensorModel::HESAI_PANDAR40P, "Pandar40P.csv");
}

TEST(HesaiReturnMode, PandarQT64)
{
  expect_only_supported_return_modes_to_be_decoded<PandarQT64>(
    SensorModel::HESAI_PANDARQT64, "PandarQT64.csv");
}

TEST(HesaiReturnMode, PandarQT128)
{
  expect_only_supported_return_modes_to_be_decoded<PandarQT128>(
    SensorModel::HESAI_PANDARQT128, "PandarQT128.csv");
}

TEST(HesaiReturnMode, PandarXT16)
{
  expect_only_supported_return_modes_to_be_decoded<PandarXT16>(
    SensorModel::HESAI_PANDARXT16, "PandarXT16.csv");
}

TEST(HesaiReturnMode, PandarXT32)
{
  expect_only_supported_return_modes_to_be_decoded<PandarXT32>(
    SensorModel::HESAI_PANDARXT32, "PandarXT32.csv");
}

TEST(HesaiReturnMode, PandarXT32M)
{
  expect_only_supported_return_modes_to_be_decoded<PandarXT32M>(
    SensorModel::HESAI_PANDARXT32M, "PandarXT32M.csv");
}

TEST(HesaiReturnMode, PandarAT128)
{
  expect_only_supported_return_modes_to_be_decoded<PandarAT128>(
    SensorModel::HESAI_PANDARAT128, "PandarAT128.dat", 30, 150, 150);
}

TEST(HesaiReturnMode, PandarAT128P)
{
  expect_only_supported_return_modes_to_be_decoded<PandarAT128P>(
    SensorModel::HESAI_AT128P, "AT128P.dat", 30, 150, 150);
}

TEST(HesaiReturnMode, Pandar128E4X)
{
  expect_only_supported_return_modes_to_be_decoded<Pandar128E4X>(
    SensorModel::HESAI_PANDAR128_E4X, "Pandar128E4X.csv");
}

}  // namespace nebula::drivers
