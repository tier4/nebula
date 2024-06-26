// Copyright 2024 TIER IV, Inc.

#include "aeva_hw_interface_test.hpp"

#include "../common/mock_byte_stream.hpp"
#include "../data/aeva/tcp_stream.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/pointcloud.hpp"

#include <nebula_common/aeva/config_types.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/aeva_api.hpp>

#include <gtest/gtest.h>

using nebula::drivers::aeva::PointCloudMessage;
using nebula::drivers::connections::PointcloudParser;

TEST(TestParsing, Pointcloud)  // NOLINT
{
  size_t n_callback_invocations = 0;

  PointcloudParser::callback_t callback = [&](const PointCloudMessage & arg) {
    EXPECT_EQ(n_callback_invocations, 0);

    EXPECT_EQ(arg.header.aeva_marker, 0xAE5Au);
    EXPECT_EQ(arg.header.platform, 2);

    EXPECT_GT(arg.points.size(), 0);

    n_callback_invocations++;
  };

  auto mock_byte_stream = std::make_shared<nebula::test::MockByteStream>(STREAM);

  PointcloudParser parser(mock_byte_stream);
  parser.registerCallback(std::move(callback));

  EXPECT_EQ(n_callback_invocations, 1);
}
