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

#include <nebula_core_runtime/packet_router.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <optional>
#include <vector>

namespace nebula::drivers::test
{

TEST(TestPacketRouter, RouteUdpPacket)
{
  PacketRouter router;

  std::vector<PacketChannelRequirement> requirements;
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.destination_port = 2368;
  requirements.push_back(req);

  router.configure(requirements);

  SensorPacket packet;
  packet.transport = SensorTransportKind::UDP;
  packet.destination = SensorEndpoint{"", 2368};

  SensorPacketView view = SensorPacketView::from(packet);
  EXPECT_TRUE(router.route(view));
  EXPECT_EQ(view.channel, SensorPacketChannel::Data);

  EXPECT_EQ(router.get_metrics().matched_packets, 1u);
  EXPECT_EQ(router.get_metrics().processed_packets, 1u);
}

TEST(TestPacketRouter, DropUnmatchedPacket)
{
  PacketRouter router;
  router.configure({});

  SensorPacket packet;
  packet.transport = SensorTransportKind::UDP;
  packet.destination = SensorEndpoint{"", 2368};

  SensorPacketView view = SensorPacketView::from(packet);
  EXPECT_FALSE(router.route(view));
  EXPECT_EQ(router.get_metrics().dropped_packets, 1u);
}

TEST(TestPacketRouter, RouteTcpPacket)
{
  PacketRouter router;
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::TCP;
  req.channel = SensorPacketChannel::Control;
  req.destination_port = 9347;

  router.configure({req});

  SensorPacket packet;
  packet.transport = SensorTransportKind::TCP;
  packet.destination = SensorEndpoint{"", 9347};

  SensorPacketView view = SensorPacketView::from(packet);
  EXPECT_TRUE(router.route(view));
  EXPECT_EQ(view.channel, SensorPacketChannel::Control);
}

TEST(TestPacketRouter, RoutePacketWithOffsetSignature)
{
  PacketRouter router;
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Info;
  req.destination_port = 2368;
  req.payload_signature = PayloadSignature{2, {0xbe, 0xef}, std::nullopt};

  router.configure({req});

  SensorPacket packet;
  packet.transport = SensorTransportKind::UDP;
  packet.destination = SensorEndpoint{"", 2368};
  packet.payload = {0xde, 0xad, 0xbe, 0xef};

  SensorPacketView view = SensorPacketView::from(packet);
  EXPECT_TRUE(router.route(view));
  EXPECT_EQ(view.channel, SensorPacketChannel::Info);
}

TEST(TestPacketRouter, RoutePacketWithMaskedSignature)
{
  PacketRouter router;
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Status;
  req.destination_port = 2368;
  req.payload_signature = PayloadSignature{0, {0xf0}, std::vector<uint8_t>{0xf0}};

  router.configure({req});

  SensorPacket packet;
  packet.transport = SensorTransportKind::UDP;
  packet.destination = SensorEndpoint{"", 2368};
  packet.payload = {0xf7};

  SensorPacketView view = SensorPacketView::from(packet);
  EXPECT_TRUE(router.route(view));
  EXPECT_EQ(view.channel, SensorPacketChannel::Status);
}

TEST(TestPacketRouter, DropPacketWhenSignatureOffsetWouldOverflow)
{
  PacketRouter router;
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Status;
  req.destination_port = 2368;
  req.payload_signature =
    PayloadSignature{std::numeric_limits<size_t>::max(), {0xbe}, std::nullopt};

  router.configure({req});

  SensorPacket packet;
  packet.transport = SensorTransportKind::UDP;
  packet.destination = SensorEndpoint{"", 2368};
  packet.payload = {0xbe};

  SensorPacketView view = SensorPacketView::from(packet);
  EXPECT_FALSE(router.route(view));
  EXPECT_EQ(view.channel, SensorPacketChannel::Unknown);
}

}  // namespace nebula::drivers::test
