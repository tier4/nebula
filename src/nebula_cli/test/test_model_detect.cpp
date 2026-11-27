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

#include "model_detect.hpp"

#include <nebula_hesai_decoders/decoders/pandar_64.hpp>
#include <nebula_hesai_decoders/decoders/pandar_qt64.hpp>

#include <gtest/gtest.h>

using nebula::cli::detect_model;
using nebula::cli::detect_return_mode;

TEST(DetectModel, HesaiBySize)
{
  // Sizes measured on the decoder ground-truth bags
  const std::pair<size_t, const char *> cases[] = {
    {1198, "Pandar64"},   {1262, "Pandar40P"},   {1072, "PandarQT64"},
    {1127, "PandarQT128"}, {568, "PandarXT16"},  {1080, "PandarXT32"},
    {820, "PandarXT32M"}, {1118, "PandarAT128"}, {861, "Pandar128E4X"}};
  for (const auto & [size, model] : cases) {
    EXPECT_EQ(detect_model("hesai", std::vector<uint8_t>(size)), model) << size;
  }
  EXPECT_THROW(detect_model("hesai", std::vector<uint8_t>(100)), std::runtime_error);
}

TEST(DetectModel, VelodyneByProductId)
{
  std::vector<uint8_t> packet(1206);
  packet[1205] = 0x22;
  EXPECT_EQ(detect_model("velodyne", packet), "VLP16");
  packet[1205] = 0x28;
  EXPECT_EQ(detect_model("velodyne", packet), "VLP32");
  packet[1205] = 0xA1;
  EXPECT_EQ(detect_model("velodyne", packet), "VLS128");
  packet[1205] = 0x01;
  EXPECT_THROW(detect_model("velodyne", packet), std::runtime_error);
  EXPECT_THROW(detect_model("velodyne", std::vector<uint8_t>(10)), std::runtime_error);
}

TEST(DetectModel, RobosenseNotDetectable)
{
  EXPECT_THROW(detect_model("robosense", std::vector<uint8_t>(1248)), std::runtime_error);
}

TEST(DetectReturnMode, Hesai)
{
  using namespace nebula::drivers::hesai_packet;  // NOLINT(build/namespaces)
  std::vector<uint8_t> packet(sizeof(Packet64));
  auto & p64 = *reinterpret_cast<Packet64 *>(packet.data());
  p64.tail.return_mode = return_mode::SINGLE_STRONGEST;
  EXPECT_EQ(detect_return_mode("hesai", "Pandar64", packet), "Strongest");
  p64.tail.return_mode = return_mode::DUAL_LAST_STRONGEST;
  EXPECT_EQ(detect_return_mode("hesai", "Pandar64", packet), "LastStrongest");
  p64.tail.return_mode = return_mode::DUAL_FIRST_LAST;  // not a Pandar64 mode
  EXPECT_THROW(detect_return_mode("hesai", "Pandar64", packet), std::runtime_error);
  EXPECT_THROW(detect_return_mode("hesai", "Pandar64", {}), std::runtime_error);

  std::vector<uint8_t> qt(sizeof(PacketQT64));
  reinterpret_cast<PacketQT64 *>(qt.data())->tail.return_mode = return_mode::DUAL_FIRST_LAST;
  EXPECT_EQ(detect_return_mode("hesai", "PandarQT64", qt), "LastFirst");
}

TEST(DetectReturnMode, Velodyne)
{
  std::vector<uint8_t> packet(1206);
  packet[1204] = 55;
  EXPECT_EQ(detect_return_mode("velodyne", "VLP16", packet), "Strongest");
  packet[1204] = 56;
  EXPECT_EQ(detect_return_mode("velodyne", "VLP16", packet), "Last");
  packet[1204] = 57;
  EXPECT_EQ(detect_return_mode("velodyne", "VLP16", packet), "Dual");
  packet[1204] = 0;
  EXPECT_THROW(detect_return_mode("velodyne", "VLP16", packet), std::runtime_error);
}
