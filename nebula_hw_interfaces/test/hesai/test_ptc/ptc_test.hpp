// Copyright 2024 TIER IV, Inc.
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

#include "hesai/test_ptc/tcp_mock.hpp"
#include "nebula_common/loggers/console_logger.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <utility>

namespace nebula::drivers
{
class PtcTest : public ::testing::TestWithParam<SensorModel>
{
protected:
  void SetUp() override {}

  void TearDown() override {}

  static auto make_mock_tcp_socket() { return std::make_shared<connections::MockTcpSocket>(); }

  static auto make_hw_interface(std::shared_ptr<connections::MockTcpSocket> tcp_socket)
  {
    auto model = GetParam();

    auto logger = std::make_shared<loggers::ConsoleLogger>("HwInterface");

    auto hw_interface = std::make_unique<HesaiHwInterface>(logger, std::move(tcp_socket));
    hw_interface->SetTargetModel(hw_interface->NebulaModelToHesaiModelNo(model));
    return hw_interface;
  }
};

}  // namespace nebula::drivers
